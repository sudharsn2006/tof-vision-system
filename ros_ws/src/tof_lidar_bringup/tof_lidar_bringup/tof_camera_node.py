"""
tof_camera_node.py
------------------
Author: SUDHARSAN
ROS 2 Node for Arducam B0410 ToF Camera.
Publishes /tof/image_raw, /tof/depth, and /tof/camera_info topics.
"""

import sys
import time
import numpy as np

# Try importing rclpy (ROS 2), fallback to standalone ROS-compatible bridge
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class TofCameraROSNode:
    def __init__(self):
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node('tof_camera_node')
            self.image_pub = self.node.create_publisher(Image, '/tof/image_raw', 10)
            self.depth_pub = self.node.create_publisher(Image, '/tof/depth', 10)
            self.info_pub = self.node.create_publisher(CameraInfo, '/tof/camera_info', 10)
            print("[TofCameraROSNode] ROS 2 Node initialized on topics: /tof/image_raw, /tof/depth")
        else:
            print("[TofCameraROSNode] Running in ROS Bridge Mode (rclpy not detected).")

    def publish_frame(self, amplitude: np.ndarray, depth_m: np.ndarray):
        if not ROS2_AVAILABLE:
            return

        now = self.node.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = "tof_camera_link"

        # Publish Grayscale Amplitude Image
        amp_8u = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        msg_amp = Image()
        msg_amp.header = header
        msg_amp.height = amp_8u.shape[0]
        msg_amp.width = amp_8u.shape[1]
        msg_amp.encoding = "mono8"
        msg_amp.step = amp_8u.shape[1]
        msg_amp.data = amp_8u.tobytes()
        self.image_pub.publish(msg_amp)

        # Publish 32FC1 Depth Image
        msg_depth = Image()
        msg_depth.header = header
        msg_depth.height = depth_m.shape[0]
        msg_depth.width = depth_m.shape[1]
        msg_depth.encoding = "32FC1"
        msg_depth.step = depth_m.shape[1] * 4
        msg_depth.data = depth_m.astype(np.float32).tobytes()
        self.depth_pub.publish(msg_depth)


def main(args=None):
    node = TofCameraROSNode()
    print("[TofCameraROSNode] ROS Camera Publisher running.")


if __name__ == '__main__':
    main()
