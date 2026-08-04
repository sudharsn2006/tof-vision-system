"""
ydlidar_node.py
---------------
Author: SUDHARSAN
ROS 2 Node for YDLIDAR X2 Sensor.
Publishes standard sensor_msgs/LaserScan message on /scan topic.
"""

import sys
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class YdLidarROSNode:
    def __init__(self):
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node('ydlidar_node')
            self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
            print("[YdLidarROSNode] ROS 2 Node initialized on topic: /scan")
        else:
            print("[YdLidarROSNode] Running in ROS Bridge Mode (rclpy not detected).")

    def publish_scan(self, points, scan_time: float):
        if not ROS2_AVAILABLE or not points:
            return

        now = self.node.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = "laser_frame"

        scan_msg = LaserScan()
        scan_msg.header = header
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = (2 * np.pi) / len(points)
        scan_msg.scan_time = scan_time
        scan_msg.range_min = 0.05
        scan_msg.range_max = 10.0

        scan_msg.ranges = [float(p[1]) for p in points]
        scan_msg.intensities = [float(p[2]) for p in points]

        self.scan_pub.publish(scan_msg)


def main(args=None):
    node = YdLidarROSNode()
    print("[YdLidarROSNode] ROS LiDAR Publisher running.")


if __name__ == '__main__':
    main()
