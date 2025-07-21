import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package = "diff_drive_bot"
    world = LaunchConfiguration('world')

    urdf_path = os.path.join(get_package_share_directory(package), "urdf", "robot.urdf.xacro")
    world_path = os.path.join(get_package_share_directory(package), "worlds", "boxed.world")

    rviz = LaunchConfiguration('rviz')

    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='true', description="Open rviz")

    world_arg = DeclareLaunchArgument(
        name='world', default_value=world_path, description='Load world'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )    
    
    gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('diff_drive_bot'),
        'rviz',
        'rviz1.rviz'
    ])

    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',)]
    )

    spawn_bot = Node(
            package='ros_gz_sim', 
            executable='create',
            arguments=['-topic', 'robot_description',
                        '-name', 'robot',
                        '-z', '0.2'],
            output='screen'
    ) 

    bridge = os.path.join(get_package_share_directory(package),'config','gz_ros_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge}',]
    )
   
    return LaunchDescription([rviz_arg, world_arg, rviz2, rsp, gazebo, ros_gz_bridge, spawn_bot])