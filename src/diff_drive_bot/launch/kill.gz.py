from launch.actions import ExecuteProcess
from launch import LaunchDescription

def generate_launch_description():
    kill_gz = ExecuteProcess(
            cmd=["pkill", '-f', 'gz'], 
            shell=True, 
            output="screen")
            
    return LaunchDescription(
        [kill_gz]
    )   