# ToF Camera & LiDAR Project — Runnable Files Execution Guide

**Author**: SUDHARSAN

This comprehensive guide lists all **runnable files, entry points, test suites, and ROS 2 nodes** in the repository for report documentation and testing purposes.

---

## 📋 Executive Summary of Runnable Files

| # | Application / Script | Location | Key Functionality | Primary Tech Stack |
|---|---|---|---|---|
| 1 | **Dual-Sensor Main App** | `main.py` | Integrated ToF Camera & LiDAR real-time object detection | Python, OpenCV, Arducam SDK, YDLIDAR SDK |
| 2 | **LiDAR 3D Viewer Pro GUI** | `Testing/main.py` | RViz-like 3D desktop visualizer, point clustering & hazard alerts | PySide6, PyOpenGL, PyGLM, PySerial |
| 3 | **Arducam ToF Test Suite** | `camera_test_suite/main_camera_test.py` | 6-window diagnostic tool, 3D pinhole point cloud & depth probe | Python, OpenCV, Arducam Depth SDK |
| 4 | **Human Detection Module** | `camera_test_suite/human_detection.py` | Standalone non-ML depth-contour human silhouette detection | Python, OpenCV |
| 5 | **YDLIDAR X2 Test Suite** | `lidar_test_suite/main_lidar_test.py` | 2D Radar scanner, 3D extruded pillars, sector HUD | Python, OpenCV, YDLIDAR SDK |
| 6 | **ROS 2 Sensor Fusion Node** | `ros_ws/.../sensor_fusion_node.py` | Multi-sensor point cloud & scan fusion node | ROS 2, Python, colcon |
| 7 | **ROS 2 ToF Camera Node** | `ros_ws/.../tof_camera_node.py` | ROS 2 publisher for depth point cloud | ROS 2, sensor_msgs |
| 8 | **ROS 2 YDLIDAR Node** | `ros_ws/.../ydlidar_node.py` | ROS 2 publisher for LaserScan data | ROS 2, LaserScan |

---

## 🛠️ Environment Prerequisites & Setup

### System Dependencies (Linux / Raspberry Pi OS 64-bit)

```bash
sudo apt update
```

```bash
sudo apt install -y python3-pip python3-venv cmake build-essential swig libopenblas-dev libgl1-mesa-dev libglu1-mesa-dev
```

### Serial Port Permissions

```bash
sudo usermod -aG dialout $USER
```

---

## 1️⃣ Main Dual-Sensor Application (`main.py`)

### Overview
Integrates the **Arducam B0410 ToF Camera** and **YDLIDAR X2** into a synchronized real-time processing loop. Uses depth thresholding, morphological filtering, and contour extraction to detect physical objects and render live telemetry overlay windows.

### Execution Commands (Separated)

Activate Virtual Environment:
```bash
source venv/bin/activate
```

Run Main Application (Linux / Pi):
```bash
python3 main.py
```

Run Main Application (Windows):
```bash
python main.py
```

### Keyboard Controls
- `Q`: Quit program
- `S`: Save snapshot image to `./captures/`
- `R`: Reset object detection tracking state
- `L`: Toggle LiDAR console status log

---

## 2️⃣ LiDAR 3D Viewer Pro GUI (`Testing/main.py`)

### Overview
A production-grade, RViz-like 3D desktop visualizer built with **PySide6** and **PyOpenGL**. Features a 3D orbit camera, ground grid network, 2D radar overlay, real-time Euclidean point cloud clustering with bounding spheres, proximity alarm sound beeps, and CSV scan recording. Includes an automatic fallback simulator when physical LiDAR hardware is not connected.

### Execution Commands (Separated)

Navigate to Testing Directory:
```bash
cd Testing
```

Install Dependencies (if needed):
```bash
pip install -r requirements.txt
```

Run 3D Visualizer Application:
```bash
python main.py
```

Run from Project Root:
```bash
python Testing/main.py
```

### Controls & Navigation
- **Left Mouse Drag**: Rotate 3D orbit camera
- **Middle Mouse Drag**: Pan viewport
- **Mouse Scroll**: Zoom camera in/out
- **Double Click Left Mouse**: Reset camera to isometric view
- **F11**: Toggle Fullscreen Mode
- **Q / Alt+F4**: Exit Application

---

## 3️⃣ Standalone Arducam ToF Test Suite (`camera_test_suite/main_camera_test.py`)

### Overview
A multi-tab diagnostic suite designed to test, visualize, and calibrate the Arducam B0410 ToF Depth Camera. Includes 6 interactive windows:
1. Grayscale 940nm IR Amplitude Stream (with CLAHE auto-gain)
2. Metric JET Depth Map (with click-to-probe depth inspection)
3. Contour Object Detector & Bounding Box Analysis
4. 3D Surface Point Cloud Visualizer (with $360^\circ$ auto-rotation)
5. RAW Phase & SNR Confidence Diagnostics
6. Hardware Telemetry & Control Panel

### Execution Commands (Separated)

Run Camera Diagnostic Test Suite:
```bash
python camera_test_suite/main_camera_test.py
```

Run with Python 3 explicit path:
```bash
python3 camera_test_suite/main_camera_test.py
```

### Interactive Controls
- **Left Mouse Click** (Tab 2): Probe exact distance ($m$/$cm$) at pixel location
- `M`: Toggle Range Mode (Near: $0.1-2.0m$ $\leftrightarrow$ Far: $0.2-4.0m$)
- `C`: Toggle Colormap (JET false-color $\leftrightarrow$ Grayscale)
- `A`: Toggle 3D Point Cloud Auto-Rotation
- `S`: Save high-res screenshot and JSON metadata
- `1` – `6`: Toggle specific window tabs on/off

---

## 4️⃣ Standalone Human Silhouette Detection (`camera_test_suite/human_detection.py`)

### Overview
A lightweight diagnostic module implementing non-machine-learning human body detection using depth-map segmentation, vertical aspect-ratio bounding boxes, and body area heuristics.

### Execution Commands (Separated)

Run Standalone Human Detection:
```bash
python camera_test_suite/human_detection.py
```

---

## 5️⃣ Standalone YDLIDAR X2 Test Suite (`lidar_test_suite/main_lidar_test.py`)

### Overview
Dedicated LiDAR evaluation suite featuring:
- High-definition 2D Polar Radar view (1m to 10m grid rings)
- Interactive 3D point cloud view with 3D extruded obstacle pillars
- Euclidean point clustering & physical object boundary estimation
- Proximity hazard warning banner (🚨 EMERGENCY < 0.5m, ⚠️ WARNING < 1.2m, ✅ CLEAR)
- 6-Sector directional distance breakdown panel

### Execution Commands (Separated)

Run Standalone LiDAR Test Suite:
```bash
python lidar_test_suite/main_lidar_test.py
```

Run with Python 3:
```bash
python3 lidar_test_suite/main_lidar_test.py
```

### Keyboard Controls
- `Q`: Quit application
- `S`: Save 2D/3D snapshot image and JSON raw scan data
- `A`: Toggle 3D Auto-Rotation
- `C`: Toggle Obstacle Clustering bounding boxes
- `Z`: Toggle Proximity Safety Banner & Sector HUD
- `2`: Toggle 2D Radar Window
- `3`: Toggle 3D Viewport Window

---

## 6️⃣ ROS 2 Sensor Fusion Bringup Package (`ros_ws`)

### Overview
ROS 2 nodes written in Python (`rclpy`) for publishing ToF camera depth clouds (`sensor_msgs/PointCloud2`), YDLIDAR laser scans (`sensor_msgs/LaserScan`), and fused spatial representations.

### Build Commands (Separated)

Navigate to ROS Workspace Root:
```bash
cd ros_ws
```

Build Package with Colcon:
```bash
colcon build
```

Source Workspace Setup File (Bash):
```bash
source install/setup.bash
```

Source Workspace Setup File (Zsh):
```bash
source install/setup.zsh
```

### Node Execution Commands (Separated)

Run Integrated Sensor Fusion Node:
```bash
ros2 run tof_lidar_bringup sensor_fusion_node
```

Run ToF Camera ROS 2 Publisher Node:
```bash
ros2 run tof_lidar_bringup tof_camera_node
```

Run YDLIDAR X2 ROS 2 Publisher Node:
```bash
ros2 run tof_lidar_bringup ydlidar_node
```

List Active ROS 2 Topics:
```bash
ros2 topic list
```

Echo Fused Telemetry Data:
```bash
ros2 topic echo /sensor_fusion/telemetry
```
