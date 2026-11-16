import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    pkg_sim_env = get_package_share_directory('sim_env')

    # Try to read user_config.yaml from workspace if present
    default_world = 'warehouse'
    default_map = 'warehouse'
    default_robot = 'turtlebot3_waffle'
    default_planner = 'astar'
    default_x = '0.0'
    default_y = '0.0'
    default_z = '0.01'
    default_yaw = '0.0'

    try:
        possible_cfg_paths = [
            os.path.expanduser('~/ros2_motion_planning/src/user_config/user_config.yaml'),
            os.path.expanduser('~/ros_motion_planning/src/user_config/user_config.yaml'),
            os.path.join(os.getcwd(), 'src/user_config/user_config.yaml'),
            os.path.join(os.getcwd(), '../src/user_config/user_config.yaml'),
        ]
        for p in possible_cfg_paths:
            if os.path.exists(p):
                with open(p, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        default_world = str(data.get('world', default_world))
                        default_map = str(data.get('map', default_map))
                        robots = data.get('robots_config', [])
                        if robots and len(robots) > 0:
                            r0 = robots[0]
                            default_robot = str(r0.get('robot1_type', default_robot))
                            default_planner = str(r0.get('robot1_global_planner', default_planner))
                            default_x = str(r0.get('robot1_x_pos', default_x))
                            default_y = str(r0.get('robot1_y_pos', default_y))
                            default_z = str(r0.get('robot1_z_pos', default_z))
                            default_yaw = str(r0.get('robot1_yaw', default_yaw))
                break
    except Exception:
        pass

    # Launch configurations
    world = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map_name')
    robot = LaunchConfiguration('robot')
    planner = LaunchConfiguration('planner')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    yaw = LaunchConfiguration('yaw')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # Arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World file name (without .world)',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map_name',
        default_value=default_map,
        description='Map folder/name under maps/',
    )

    declare_robot_cmd = DeclareLaunchArgument(
        'robot',
        default_value=default_robot,
        description='Robot model name (turtlebot3_waffle, turtlebot3_burger, etc.)',
    )

    declare_planner_cmd = DeclareLaunchArgument(
        'planner',
        default_value=default_planner,
        description='Global planner algorithm (astar, dijkstra, jps, rrt_star, hybrid_astar, etc.)',
    )

    declare_x_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value=default_x,
        description='Initial X position of robot',
    )

    declare_y_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value=default_y,
        description='Initial Y position of robot',
    )

    declare_z_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value=default_z,
        description='Initial Z position of robot',
    )

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value=default_yaw,
        description='Initial Yaw orientation of robot in radians',
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to launch Nav2 stack',
    )

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2',
    )

    # Simulation environment launch
    sim_env_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_env, 'launch', 'sim_env.launch.py')
        ),
        launch_arguments={
            'world': world,
            'robot': robot,
            'x_pos': x_pos,
            'y_pos': y_pos,
            'z_pos': z_pos,
            'yaw': yaw,
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'rviz_config': 'sim_env_ros2.rviz',
        }.items(),
    )

    # Nav2 navigation launch
    map_yaml_path = PathJoinSubstitution([
        pkg_sim_env,
        'maps',
        map_name,
        [map_name, TextSubstitution(text='.yaml')],
    ])

    orig_params_path = os.path.join(pkg_sim_env, 'config', 'nav2_params.yaml')
    configured_params_path = os.path.join(pkg_sim_env, 'config', 'nav2_params_active.yaml')
    try:
        with open(orig_params_path, 'r') as pf:
            cfg = yaml.safe_load(pf)
        if 'planner_server' in cfg and 'ros__parameters' in cfg['planner_server']:
            cfg['planner_server']['ros__parameters']['GridBased']['planner_name'] = default_planner
        with open(configured_params_path, 'w') as pf:
            yaml.dump(cfg, pf)
    except Exception:
        configured_params_path = orig_params_path

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_env, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': configured_params_path,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(use_nav2),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_robot_cmd)
    ld.add_action(declare_planner_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)

    ld.add_action(sim_env_cmd)
    ld.add_action(nav2_cmd)

    return ld
