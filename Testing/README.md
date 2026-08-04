# LiDAR 3D Viewer Pro (Raspberry Pi 4 / USB LiDAR)

A production-grade, high-performance 3D RViz-like desktop visualizer built with **PySide6**, **PyOpenGL**, **PyGLM**, **NumPy**, and **PySerial** for Raspberry Pi 4 Model B (4GB RAM) running 64-bit Debian Bookworm.

---

## Key Features

1. **Hardware & Communication Driver:**
   - Native PySerial YDLIDAR X2 USB driver (`115200` baud) with packet checksum validation and automatic port detection (`/dev/ttyUSB*` or `COM*`).
   - Built-in fallback 360° LiDAR simulator mode for instant testing on any PC without physical hardware.

2. **GPU-Accelerated 3D Viewport:**
   - Built on `QOpenGLWidget` with GLSL shaders and Vertex Buffer Objects (VBOs).
   - Smooth 3D Orbit Camera (Left-click drag to rotate, Middle-click drag to pan, Scroll wheel to zoom, Double-click to reset).
   - Infinite 3D ground grid with 100 mm minor and 1000 mm major grid spacing.
   - XYZ Coordinate axes (Red = X, Green = Y, Blue = Z).
   - Robot 3D cylinder model with a forward red orientation arrow.
   - Concentric distance rings at 0.5 m, 1.0 m, 2.0 m, 3.0 m, and 4.0 m.
   - Live laser radar sweep beam animation.
   - Color-coded anti-aliased point cloud (Green = Safe, Yellow = Warning, Red = Collision < 0.5 m).

3. **Spatial Object & Hazard Detection:**
   - Real-time Euclidean point cloud clustering to detect discrete obstacle boundaries.
   - Dynamic 3D bounding circles with centroid metrics and object IDs.
   - Real-time collision monitoring within 0.5m with flashing UI alarms and audio warning beeps.

4. **Interactive Telemetry & Radar Overlay:**
   - Left side-panel: Real-time COM parameters, FPS, Scan Frequency (Hz), Sample rate (sps), and Pi system resources (CPU %, RAM %, CPU Temp °C).
   - Right side-panel: Nearest object distance, bearing angle, total object count, and hazard status.
   - Top-Right corner 2D radar minimap overlay.

5. **Data Export & Settings:**
   - PNG Screenshot capture of the 3D viewport using OpenCV.
   - Continuous timestamped CSV scan recording (`Timestamp`, `Angle_deg`, `Distance_mm`, `X_mm`, `Y_mm`, `Z_mm`, `Intensity`).
   - Modal settings dialog to tune Point Size, Collision Radius, Grid Extent, Target FPS limit, and Camera Speed.

---

## Hardware Setup (Raspberry Pi 4)

1. Connect the **YDLIDAR X2 USB LiDAR** to a USB port on the Raspberry Pi 4.
2. Verify USB serial device detection:
   ```bash
   ls /dev/ttyUSB*
   ```
3. Grant dialout serial port permissions:
   ```bash
   sudo usermod -aG dialout $USER
   ```
   *(Reboot or re-login for group changes to take effect).*

---

## Software Installation & Dependencies

```bash
# 1. System build tools and OpenGL drivers
sudo apt update
sudo apt install -y python3-pip python3-venv libgl1-mesa-dev libglu1-mesa-dev

# 2. Navigate to Testing folder
cd Testing

# 3. Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 4. Install Python dependencies
pip install -r requirements.txt
```

---

## Running the Application

```bash
python main.py
```

### Keyboard Shortcuts & Controls
- **Left Mouse Drag:** Orbit / rotate 3D camera
- **Middle Mouse Drag:** Pan camera target
- **Mouse Scroll Wheel:** Perspective zoom
- **Double Click Left Mouse:** Reset camera to default isometric view
- **F11:** Toggle Fullscreen Mode
- **Q / Alt+F4:** Quit Application
