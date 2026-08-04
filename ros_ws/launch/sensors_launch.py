"""
sensors_launch.py
-----------------
Author: SUDHARSAN
ROS 2 Launch file for starting ToF Camera, YDLIDAR X2, and Sensor Fusion nodes.
Usage: ros2 launch tof_lidar_bringup sensors_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. ToF Camera Node
        Node(
            package='tof_lidar_bringup',
            executable='tof_camera_node',
            name='tof_camera_node',
            output='screen'
        ),
        # 2. YDLIDAR X2 Node
        Node(
            package='tof_lidar_bringup',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen'
        ),
        # 3. Sensor Fusion Node
        Node(
            package='tof_lidar_bringup',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen'
        ),
    ])
