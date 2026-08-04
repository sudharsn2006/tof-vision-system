# ROS & ROS 2 Working Operations Architecture

This folder contains standard **ROS (Robot Operating System)** and **ROS 2** nodes, package manifests, topics, and launch files for sensor fusion with the **Arducam B0410 ToF Camera** and **YDLIDAR X2**.

---

## 📡 Published ROS / ROS 2 Topics

| Topic Name | Message Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D 360° LiDAR Scan data ($10\text{m}$ max range) |
| `/tof/image_raw` | `sensor_msgs/msg/Image` (mono8) | 940nm VCSEL Grayscale IR Amplitude feed |
| `/tof/depth` | `sensor_msgs/msg/Image` (32FC1) | Calibrated metric depth map ($0.1 - 4.0\text{m}$) |
| `/tof/camera_info` | `sensor_msgs/msg/CameraInfo` | $70^\circ$ FOV pinhole camera intrinsics |
| `/fused_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D spatial point cloud fusing ToF depth + LiDAR |
| `/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | 3D bounding box markers for **RViz / RViz2** |
| `/safety_status` | `std_msgs/msg/String` | Real-time Emergency / Warning / Clear alert status |

---

### 1. Install Colcon Build Tool via Pip

```bash
# Activate your project virtual environment
cd ~/tof_lidar_project
source venv/bin/activate

# Install colcon build tool
pip install colcon-common-extensions
```

### 2. Build Workspace using Colcon

```bash
cd ~/tof_lidar_project/ros_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch All Sensor Nodes

Launch ToF camera, YDLIDAR X2, and 3D Sensor Fusion nodes simultaneously:

```bash
ros2 launch tof_lidar_bringup sensors_launch.py
```

### 3. Visualize in RViz2

Open **RViz2** to view the 3D Point Cloud and 3D Bounding Boxes:

```bash
rviz2
```

In RViz2:
- Set **Fixed Frame** to `base_link` or `laser_frame`.
- Add **PointCloud2** display listening to topic `/fused_point_cloud`.
- Add **MarkerArray** display listening to topic `/obstacle_markers`.
- Add **LaserScan** display listening to topic `/scan`.

---

## 🤖 ROS 1 (Noetic / Melodic) Setup

If running on ROS 1:

```bash
cd ~/tof_lidar_project/ros_ws
catkin_make
source devel/setup.bash
rosrun tof_lidar_bringup tof_camera_node.py
rosrun tof_lidar_bringup ydlidar_node.py
rosrun tof_lidar_bringup sensor_fusion_node.py
```

---

## 📁 Package Directory Structure

```
ros_ws/
├── README_ROS.md
├── launch/
│   └── sensors_launch.py         # ROS 2 Launch file for all nodes
└── src/
    └── tof_lidar_bringup/
        ├── package.xml           # ROS 2 / ROS package manifest
        ├── setup.py              # ROS 2 Python package installer
        └── tof_lidar_bringup/
            ├── __init__.py
            ├── tof_camera_node.py# ROS node for ToF Camera (/tof/depth, /tof/image_raw)
            ├── ydlidar_node.py   # ROS node for YDLIDAR X2 (/scan)
            └── sensor_fusion_node.py # ROS node for 3D Fusion & RViz markers (/fused_point_cloud)
```
