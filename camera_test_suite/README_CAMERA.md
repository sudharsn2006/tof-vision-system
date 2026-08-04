# Standalone Arducam B0410 ToF Camera Test Suite

A dedicated multi-tab testing suite built specifically for the **Arducam B0410 Time-of-Flight (ToF) Depth Camera**.

> ℹ️ This suite is completely isolated in the `camera_test_suite/` directory and does **not** modify or alter any files in the main project folder.

---

## 📷 Arducam B0410 Camera Specifications

| Specification | Details |
|---|---|
| **Model** | Arducam B0410 |
| **Camera Type** | Time-of-Flight (ToF) Depth Camera |
| **Depth Technology** | Active Infrared Time-of-Flight |
| **Sensor Resolution** | **240 × 180 pixels** |
| **Sensor Size** | **1/6-inch** |
| **Max Sensor Frame Rate** | **120 FPS** (RAW) / **30 FPS** Depth (4-phase on RPi) |
| **Measurement Range** | **Near Mode:** 0.1–2.0 m \| **Far Mode:** 0.2–4.0 m |
| **Depth Accuracy** | **< ±2 cm** typical |
| **Field of View (FOV)** | **70° Diagonal** |
| **Infrared Light Source** | **940 nm VCSEL Laser** |
| **Modulation Frequency** | **75 MHz / 37.5 MHz** |
| **Output Formats** | RAW Phase Frames, Metric Depth Map, Grayscale Amplitude Frame |
| **Connection Interface** | MIPI CSI-2 (2-Lane), V4L2 Driver |

---

## 🌟 6 Dedicated Feature Windows / Tabs

1. **Tab 1: `Grayscale Amplitude Stream (940nm IR)`**
   - High-contrast 940nm VCSEL infrared intensity stream.
   - Real-time CLAHE histogram equalization & automatic gain control (AGC).

2. **Tab 2: `Metric Depth Map (JET & Grayscale)`**
   - Calibrated 3D depth map ($0.1\text{m} - 4.0\text{m}$) in false-color JET or fast normalized grayscale.
   - Live **Interactive Depth Probe**: Click anywhere on the window to view exact distance in meters & cm.

3. **Tab 3: `Contour & Distance Object Detector`**
   - Real-time contour analysis & depth thresholding.
   - Draws bounding boxes, center coordinates $(cx, cy)$, pixel area, average distance, and min/max depth.

4. **Tab 4: `3D Surface Point Cloud Visualizer`**
   - $70^\circ$ FOV pinhole spatial 3D surface point cloud reconstruction ($X, Y, Z$).
   - Interactive $360^\circ$ **Auto-Rotation Mode** (toggle with `A`).

5. **Tab 5: `RAW Phase & Confidence Diagnostics`**
   - Signal-to-noise ratio (SNR), confidence heatmap, valid pixel percentage, and noise diagnostics.

6. **Tab 6: `Camera Control & Telemetry Panel`**
   - Telemetry dashboard displaying live FPS, resolution (240x180), mode (Near/Far), VCSEL laser status, and full hardware spec table.

---

## 🚀 How to Run

Navigate into the project directory and activate the virtual environment:

```bash
cd ~/tof_lidar_project
source venv/bin/activate
```

Run the standalone camera test suite:

```bash
python camera_test_suite/main_camera_test.py
```

---

## ⌨️ Keyboard & Mouse Controls

| Key / Action | Function |
|---|---|
| **Left Mouse Click** (on Tab 2) | Probe exact depth distance at clicked coordinate |
| `Q` | Quit the application |
| `S` | Save snapshot (PNG image + JSON telemetry metadata) |
| `M` | Toggle Range Mode: **Near** (0.1–2.0m) $\leftrightarrow$ **Far** (0.2–4.0m) |
| `C` | Toggle Depth display: **JET Colormap** $\leftrightarrow$ **Grayscale Depth** |
| `A` | Toggle 3D Point Cloud Auto-Rotation |
| `1` – `6` | Toggle individual Feature Windows / Tabs on/off |

---

## 📁 File Structure

```
camera_test_suite/
├── camera_config.py          # Configuration parameters & window titles
├── standalone_camera.py      # Threaded Arducam SDK frame acquisition (Depth + Amplitude + Confidence)
├── camera_features.py        # Image processing, 3D pinhole projection, contour detection & diagnostics
├── camera_ui_renderer.py     # Renderer for all 6 feature windows
├── main_camera_test.py       # Main application entry point
└── README_CAMERA.md          # Documentation & specs table
```
