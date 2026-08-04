# Standalone YDLIDAR X2 2D & 3D Test Suite

A dedicated, high-performance LiDAR testing suite built specifically for the **YDLIDAR X2 (USB)**.

> ℹ️ This suite is completely isolated in the `lidar_test_suite/` directory and does **not** touch or alter any files in the main project directory.

---

## 🌟 Key Features

1. **High-Definition 2D Radar View**:
   - Dynamic polar grid rings (1m to 10m).
   - Angle crosshairs (every 30°).
   - Real-time sweep animation line.
   - Color-coded laser range returns.

2. **Interactive 3D Point Cloud View**:
   - 3D perspective / isometric viewport of laser scans.
   - Smooth $360^\circ$ **Auto-Rotation Mode** (toggle with `A`).
   - 3D Ground Grid network ($X: -6\text{m} \to +6\text{m}$, $Y: 1\text{m} \to 10\text{m}$).
   - **3D Extruded Obstacle Pillars** rendered in height space.

3. **Obstacle Clustering & Analytics**:
   - Real-time Euclidean point cloud clustering (identifies distinct physical objects).
   - Bounding boxes, center coordinates, angular offset, and physical width estimations ($m$).

4. **Proximity Safety Warning System**:
   - Real-time safety status banner:
     - 🚨 **EMERGENCY** (< 0.5m red alert)
     - ⚠️ **WARNING** (< 1.2m yellow caution)
     - ✅ **CLEAR** (> 1.2m green status)

5. **6-Sector Directional Readings**:
   - Dedicated side-panel breakdown for Front, Front-Left, Front-Right, Left, Right, and Rear obstacle distances.

6. **Data Capture & Export**:
   - Press `S` to instantly export color PNG captures and full JSON raw point cloud metadata to `./lidar_test_suite/captures/`.

---

## 🚀 How to Run

Navigate into the project folder and activate your virtual environment:

```bash
cd ~/tof_lidar_project
source venv/bin/activate
```

Run the standalone LiDAR test suite:

```bash
python lidar_test_suite/main_lidar_test.py
```

---

## ⌨️ Keyboard Controls

| Key | Action |
|-----|--------|
| `Q` | Quit the application |
| `S` | Save a snapshot (PNG image + JSON scan data) |
| `A` | Toggle 3D Auto-Rotation on/off |
| `C` | Toggle Obstacle Cluster bounding boxes |
| `Z` | Toggle Proximity Safety Banner & Sector HUD |
| `2` | Toggle 2D Radar Window on/off |
| `3` | Toggle 3D Point Cloud Window on/off |

---

## 📁 File Structure

```
lidar_test_suite/
├── lidar_config.py         # Config parameters (ports, baudrates, range limits, thresholds)
├── standalone_lidar.py     # YDLIDAR SDK background scan thread & sector processor
├── obstacle_analyzer.py    # Euclidean point clustering & safety proximity evaluator
├── lidar_3d_renderer.py    # 2D Radar & 3D Perspective OpenGL-style OpenCV renderers
├── main_lidar_test.py      # Main application entry point
└── README_LIDAR.md         # Documentation
```
