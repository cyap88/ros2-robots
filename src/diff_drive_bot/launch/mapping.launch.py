import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paths
    pkg_diff_drive = get_package_share_directory('diff_drive_bot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_path = os.path.join(pkg_diff_drive, 'config', 'slam_toolbox_mapping.yaml')
    robot_launch_path = os.path.join(pkg_diff_drive, 'launch', 'robot.launch.py')
    slam_launch_path = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz2.rviz',
        description='RViz config file'
    )

    # Include robot spawning
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Include SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path
        }.items()
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_diff_drive, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(rviz_node)

    return ld
