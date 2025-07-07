from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.xacro')

    return LaunchDescription([

        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[xacro_file]
        ),

        Node(
            package='ros_ign_gazebo',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),
    ])
