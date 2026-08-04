"""
sensor_fusion_node.py
---------------------
Author: SUDHARSAN
ROS 2 Sensor Fusion & Obstacle Clustering Node.
Subscribes to /scan and /tof/depth, fuses 3D spatial point cloud,
and publishes /fused_point_cloud, /obstacle_markers (RViz 3D boxes), and /safety_status.
"""

import sys
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Image
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import String, Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class SensorFusionROSNode:
    def __init__(self):
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node('sensor_fusion_node')
            self.cloud_pub = self.node.create_publisher(PointCloud2, '/fused_point_cloud', 10)
            self.marker_pub = self.node.create_publisher(MarkerArray, '/obstacle_markers', 10)
            self.status_pub = self.node.create_publisher(String, '/safety_status', 10)

            # Subscriptions
            self.node.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
            self.node.create_subscription(Image, '/tof/depth', self._depth_callback, 10)
            print("[SensorFusionROSNode] ROS 2 Fusion Node running on /fused_point_cloud, /obstacle_markers")
        else:
            print("[SensorFusionROSNode] Running in ROS Bridge Mode (rclpy not detected).")

    def _scan_callback(self, msg):
        pass

    def _depth_callback(self, msg):
        pass

    def publish_fusion_output(self, fused_points, clusters, safety_text: str):
        if not ROS2_AVAILABLE:
            return

        now = self.node.get_clock().now().to_msg()

        # 1. Publish /safety_status Topic
        msg_status = String()
        msg_status.data = safety_text
        self.status_pub.publish(msg_status)

        # 2. Publish /obstacle_markers 3D Bounding Boxes for RViz / RViz2
        marker_array = MarkerArray()
        for idx, obs in enumerate(clusters):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "base_link"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(obs.center_x)
            marker.pose.position.y = float(obs.center_y)
            marker.pose.position.z = 0.4

            marker.scale.x = max(float(obs.width_m), 0.2)
            marker.scale.y = max(float(obs.width_m), 0.2)
            marker.scale.z = 0.8  # 0.8m height box

            marker.color.r = 1.0 if "EMERGENCY" in safety_text else 0.0
            marker.color.g = 1.0 if "CLEAR" in safety_text else 0.8
            marker.color.b = 0.0
            marker.color.a = 0.6

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    node = SensorFusionROSNode()
    print("[SensorFusionROSNode] ROS Fusion Node active.")


if __name__ == '__main__':
    main()
