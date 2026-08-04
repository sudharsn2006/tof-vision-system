# ToF Depth Object Detector + LiDAR Sensor Test (Raspberry Pi 4)

**Author**: SUDHARSAN  
**Project**: Time-of-Flight (ToF) Camera & 2D/3D LiDAR Sensor Fusion & Diagnostic Suite

---

A lightweight sensor-testing/validation application for:

- **Arducam B0410 Time-of-Flight AI Camera** (CSI, depth-only object detection)
- **YDLIDAR X2** (USB, background scanning and 3D visualizer)

No AI/ML object detection models are required for core operations. Detection is done purely with depth thresholding, morphological filtering, and contour analysis.

---

## 📁 Project Structure

```
tof_lidar_project/
├── config.py              # All tunable parameters (ranges, thresholds, ports, etc.)
├── camera_manager.py       # Arducam ToF SDK wrapper (open/read/close depth frames)
├── depth_processor.py      # Noise removal, thresholding, colormap conversion
├── object_detector.py      # Contour detection + distance statistics
├── lidar_manager.py         # YDLIDAR X2 SDK wrapper, background scan thread
├── display_manager.py       # The two required OpenCV windows
├── main.py                 # Main application entry point / dual-sensor loop
├── RUNNABLE_FILES_README.md# Detailed guide for all runnable components
├── requirements.txt
└── README.md
```

---

## ⚡ Hardware Setup Commands

### 1. Check Serial Port Connection
```bash
ls /dev/ttyUSB*
```

### 2. Grant Serial Port Access Permissions
```bash
sudo usermod -aG dialout $USER
```

---

## 📦 Software Setup Commands (Separated)

### 1. System Package Updates
```bash
sudo apt update
```

### 2. System Dependencies Installation
```bash
sudo apt install -y python3-pip python3-venv cmake build-essential swig libopenblas-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 3. Create Python Virtual Environment
```bash
python3 -m venv venv
```

### 4. Activate Virtual Environment (Bash)
```bash
source venv/bin/activate
```

### 5. Install Required Python Dependencies
```bash
pip install -r requirements.txt
```

### 6. Install Arducam ToF Camera SDK Dependencies
```bash
git clone https://github.com/ArduCAM/Arducam_tof_camera.git
```
```bash
cd Arducam_tof_camera
```
```bash
./Install_dependencies.sh
```
```bash
pip install ArducamDepthCamera
```
```bash
cd ..
```

### 7. Install YDLIDAR SDK
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
```
```bash
cd YDLidar-SDK
```
```bash
mkdir build
```
```bash
cd build
```
```bash
cmake ..
```
```bash
make
```
```bash
sudo make install
```
```bash
cd ..
```
```bash
pip install .
```
```bash
cd ..
```

---

## 🚀 Runnable Applications & Execution Commands

### 1. Main Dual-Sensor Application
- **File**: `main.py`
- **Command**:
```bash
python main.py
```

### 2. 3D LiDAR Visualizer Desktop App (PySide6 + PyOpenGL)
- **File**: `Testing/main.py`
- **Command**:
```bash
python Testing/main.py
```

### 3. Standalone Camera Test Suite
- **File**: `camera_test_suite/main_camera_test.py`
- **Command**:
```bash
python camera_test_suite/main_camera_test.py
```

### 4. Standalone Human Silhouette Detection Script
- **File**: `camera_test_suite/human_detection.py`
- **Command**:
```bash
python camera_test_suite/human_detection.py
```

### 5. Standalone YDLIDAR X2 Diagnostic Test Suite
- **File**: `lidar_test_suite/main_lidar_test.py`
- **Command**:
```bash
python lidar_test_suite/main_lidar_test.py
```

### 6. ROS 2 Sensor Fusion Nodes (`ros_ws`)
- **Build Workspace**:
```bash
cd ros_ws
```
```bash
colcon build
```
```bash
source install/setup.bash
```
- **Run Sensor Fusion Node**:
```bash
ros2 run tof_lidar_bringup sensor_fusion_node
```

---

## ⌨️ Keyboard Controls (Main Application)

| Key | Action |
|-----|--------|
| `Q` | Quit the program |
| `S` | Save current depth image (PNG) to `./captures/` |
| `R` | Reset detection state (forces fresh status print) |
| `L` | Toggle periodic LiDAR status log on/off |

---

## ⚙️ Tuning Parameters

Edit `config.py`:

- `DETECTION_DISTANCE_MM`: Maximum distance threshold to trigger detection (default 2.0 m).
- `MIN_CONTOUR_AREA_PX`: Minimum pixel blob size to filter out noise.
- `TOF_INVALID_CONFIDENCE_THRESH`: Confidence filter for noisy/invalid pixels.
- `TARGET_FPS`: Target frame rate for Pi video processing.
