import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('diff_drive_bot')
    robot_launch_path = os.path.join(pkg, 'launch', 'robot.launch.py')

    models_path, _ = os.path.split(pkg)
    os.environ.setdefault('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_path

    use_sim_time_args = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    map_args = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg, 'map', 'my_map2.yaml']))
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'localization.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('nav2_bringup'),
                                  'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': PathJoinSubstitution(
                [pkg, 'config', 'amcl.yaml']),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':  'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_args)
    ld.add_action(robot_launch)
    ld.add_action(map_args)
    ld.add_action(rviz)
    ld.add_action(nav2_localization)
    return ld
