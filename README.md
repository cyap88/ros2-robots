# Differential Drive Robot
A differential drive robot simulation using ROS 2 and Gazebo that runs on Simulataneous Localization and Mapping (SLAM) and AMCL to navigate custom environments. This robot can be teleoperated and visualized in RViz and Gazebo. 

## Features:
- URDF/xacro robot model designed with LiDAR and a differential drive
- Custom Gazebo world
- RViz visualization of robot and sensor data
- Teleoperation using teleop_twist_keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard
- SLAM for mapping and navigation
- AMCL for localization

## Built With:
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)
- RViz 2
- Python

## References:
- [MOGI-ROS Week 7-8 ROS2 Navigation](https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation)
