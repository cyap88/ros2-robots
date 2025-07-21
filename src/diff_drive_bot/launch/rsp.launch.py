from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

def generate_launch_description():

    package = FindPackageShare("diff_drive_bot")

    urdf_path = PathJoinSubstitution([package, "urdf", "robot.urdf.xacro"])
    
    urdf = LaunchConfiguration('urdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true')

    urdf_arg = DeclareLaunchArgument(
            name='urdf', default_value=urdf_path,
            description='Path of robot description file')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': Command(['xacro ', urdf]),
        'publish_frequency': 30.0 }]
    )

    return LaunchDescription([urdf_arg,use_sim_time_arg,robot_state_publisher])