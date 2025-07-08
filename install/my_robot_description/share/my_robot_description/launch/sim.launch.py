from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():

    robot_description_pkg = FindPackageShare('my_robot_description')
    urdf_file = PathJoinSubstitution([robot_description_pkg, 'urdf', 'my_robot.urdf'])
    controller_config_file = PathJoinSubstitution([robot_description_pkg, 'config', 'controller.yaml'])
    robot_description_content = Command(['xacro ', urdf_file])
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_ign_gazebo'),
            '/launch/ign_gazebo.launch.py'
        ]),
        launch_arguments={'ign_args': '-v 4 empty.sdf'}.items()
    )

    spawn_entity_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'my_robot'
        ],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controller_config_file],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo_launch,
        spawn_entity_node,
        ros2_control_node
    ])