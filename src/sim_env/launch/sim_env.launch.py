import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim_env = get_package_share_directory('sim_env')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup Ignition Gazebo resource paths directly in python
    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    sim_models_path = os.path.join(pkg_sim_env, 'models')
    sim_worlds_path = os.path.join(pkg_sim_env, 'worlds')
    sim_share_parent = os.path.abspath(os.path.join(pkg_sim_env, '..'))

    combined_resource_path = (
        f"{sim_models_path}:{sim_worlds_path}:{sim_share_parent}:{ign_resource_path}"
    )

    set_ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', combined_resource_path
    )
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', combined_resource_path
    )

    # Launch configurations
    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Declare arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='World file name (without .world)',
    )

    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value='turtlebot3_waffle',
        description='Robot model name (turtlebot3_waffle, turtlebot3_burger, turtlebot3_waffle_pi, nanocar)',
    )

    declare_x_cmd = DeclareLaunchArgument(
        'x_pos', default_value='0.0', description='Initial X position of the robot'
    )
    declare_y_cmd = DeclareLaunchArgument(
        'y_pos', default_value='0.0', description='Initial Y position of the robot'
    )
    declare_z_cmd = DeclareLaunchArgument(
        'z_pos', default_value='0.01', description='Initial Z position of the robot'
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Initial Yaw orientation of the robot'
    )

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true', description='Whether to launch RViz2'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value='sim_env_ros2.rviz',
        description='RViz configuration file name',
    )

    # World path and gz_args
    world_file = PathJoinSubstitution([
        pkg_sim_env,
        'worlds',
        [world, TextSubstitution(text='.world')],
    ])

    gz_args = [TextSubstitution(text='-r '), world_file]

    # Gazebo Fortress simulation launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Robot Description via Xacro
    robot_xacro = PathJoinSubstitution([
        pkg_sim_env,
        'urdf',
        robot,
        [robot, TextSubstitution(text='.xacro')],
    ])

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        robot_xacro,
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn Robot into Gazebo Fortress
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='ros_gz_create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            robot,
            '-x',
            x_pos,
            '-y',
            y_pos,
            '-z',
            z_pos,
            '-Y',
            yaw,
        ],
    )

    # ROS <-> Gazebo Fortress Topic Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([pkg_sim_env, 'rviz', rviz_config]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)

    ld.add_action(set_ign_resource_path)
    ld.add_action(set_gz_resource_path)

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(rviz_node)

    return ld
