import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim_env = FindPackageShare('sim_env')

    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    declare_world_cmd = DeclareLaunchArgument('world', default_value='warehouse')
    declare_robot_cmd = DeclareLaunchArgument('robot', default_value='turtlebot3_waffle')
    declare_gui_cmd = DeclareLaunchArgument('gui', default_value='true')
    declare_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_rviz_cmd = DeclareLaunchArgument('rviz', default_value='true')

    sim_env_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_sim_env, 'launch', 'sim_env.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'robot': robot,
            'gui': gui,
            'use_sim_time': use_sim_time,
            'rviz': rviz,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(sim_env_cmd)

    return ld
