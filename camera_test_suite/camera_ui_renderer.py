"""
camera_ui_renderer.py
---------------------
Author: SUDHARSAN
Renders individual OpenCV feature windows for the Arducam B0410 ToF Camera Test Suite.
"""

from typing import Optional, Dict, Tuple
import numpy as np
import cv2

import camera_config as config
from camera_features import DetectedCameraObject


class CameraUIRenderer:
    def __init__(self):
        self.w = config.DISPLAY_WIDTH
        self.h = config.DISPLAY_HEIGHT
        self.yaw_3d = 0.0

    def render_amplitude_tab(self, amp_bgr: np.ndarray, fps: float) -> np.ndarray:
        """Tab 1: 940nm VCSEL IR Grayscale Amplitude Stream."""
        canvas = cv2.resize(amp_bgr, (self.w, self.h))

        # Overlay HUD
        cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
        cv2.putText(canvas, config.WIN_AMPLITUDE, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        info = f"940nm VCSEL IR | Res: 240x180 | Frame Rate: {fps:.1f} FPS"
        cv2.putText(canvas, info, (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        return canvas

    def render_depth_tab(
        self,
        depth_bgr: np.ndarray,
        fps: float,
        mode: str,
        probe_dist: Optional[float] = None,
        probe_pt: Optional[Tuple[int, int]] = None
    ) -> np.ndarray:
        """Tab 2: Metric Depth Map (JET & Grayscale) with Live Depth Probe."""
        canvas = cv2.resize(depth_bgr, (self.w, self.h))

        # Draw crosshair probe cursor if clicked
        if probe_pt is not None:
            px, py = probe_pt
            # Map probe point from 240x180 to display width/height
            disp_x = int(px * (self.w / config.SENSOR_WIDTH_PX))
            disp_y = int(py * (self.h / config.SENSOR_HEIGHT_PX))
            cv2.drawMarker(canvas, (disp_x, disp_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            if probe_dist is not None and probe_dist > 0:
                dist_str = f"Probe: {probe_dist:.2f} m ({probe_dist * 100:.1f} cm)"
                cv2.putText(canvas, dist_str, (disp_x + 10, disp_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

        # Header HUD
        cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
        cv2.putText(canvas, config.WIN_DEPTH, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        range_text = "0.2 - 4.0m (FAR)" if mode == "FAR" else "0.1 - 2.0m (NEAR)"
        info = f"Mode: {range_text} | FPS: {fps:.1f} | Click window to probe distance"
        cv2.putText(canvas, info, (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        return canvas

    def render_detection_tab(
        self,
        depth_bgr: np.ndarray,
        obj: Optional[DetectedCameraObject],
        fps: float
    ) -> np.ndarray:
        """Tab 3: Contour & Distance Object Detector."""
        canvas = cv2.resize(depth_bgr, (self.w, self.h))
        scale_x = self.w / config.SENSOR_WIDTH_PX
        scale_y = self.h / config.SENSOR_HEIGHT_PX

        if obj is not None:
            x, y, w_box, h_box = obj.bbox
            x_disp = int(x * scale_x)
            y_disp = int(y * scale_y)
            w_disp = int(w_box * scale_x)
            h_disp = int(h_box * scale_y)

            cx_disp = int(obj.center[0] * scale_x)
            cy_disp = int(obj.center[1] * scale_y)

            cv2.rectangle(canvas, (x_disp, y_disp), (x_disp + w_disp, y_disp + h_disp), (0, 255, 0), 2)
            cv2.circle(canvas, (cx_disp, cy_disp), 4, (0, 255, 0), -1)

            label = f"Distance: {obj.avg_depth_m:.2f} m"
            cv2.putText(canvas, label, (x_disp, max(y_disp - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        # Header HUD
        cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
        cv2.putText(canvas, config.WIN_DETECTION, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        if obj is not None:
            telemetry = f"Detected: {obj.avg_depth_m:.2f}m (Min: {obj.min_depth_m:.2f}m / Max: {obj.max_depth_m:.2f}m) | Area: {obj.area_px}px"
            cv2.putText(canvas, telemetry, (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(canvas, "NO OBJECT DETECTED IN THRESHOLD RANGE", (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)

        return canvas

    def render_3d_surface_tab(
        self,
        x_rot: np.ndarray,
        y_rot: np.ndarray,
        z_rot: np.ndarray,
        fps: float,
        auto_rotate: bool = True
    ) -> np.ndarray:
        """Tab 4: 3D Surface Point Cloud Visualizer."""
        canvas = np.full((self.h, self.w, 3), (12, 12, 18), dtype=np.uint8)

        if auto_rotate:
            self.yaw_3d = (self.yaw_3d + 1.0) % 360.0

        cx, cy = self.w // 2, self.h // 2 + 40
        focal_3d = 320.0

        if x_rot.size > 0:
            valid = y_rot > 0.1
            if np.any(valid):
                px = (cx + (focal_3d * x_rot[valid]) / y_rot[valid]).astype(int)
                py = (cy - (focal_3d * z_rot[valid]) / y_rot[valid]).astype(int)

                in_view = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
                px_v = px[in_view]
                py_v = py[in_view]

                # Fast vectorized 3D point plot
                canvas[py_v, px_v] = (0, 255, 255)

        # Header HUD
        cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
        cv2.putText(canvas, config.WIN_3D_SURFACE, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"70° FOV Spatial 3D Pinhole Model | FPS: {fps:.1f} | Press 'A' to toggle auto-rotate", (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        return canvas

    def render_diagnostics_tab(
        self,
        conf_heatmap: np.ndarray,
        stats: Dict[str, float],
        fps: float
    ) -> np.ndarray:
        """Tab 5: RAW Phase & Confidence Diagnostics."""
        canvas = cv2.resize(conf_heatmap, (self.w, self.h))

        # Header HUD
        cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
        cv2.putText(canvas, config.WIN_DIAGNOSTICS, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        info = f"Valid Pixels: {stats['valid_pixel_pct']}% | Avg Confidence: {stats['avg_confidence']} | Noise: {stats['noise_pct']}%"
        cv2.putText(canvas, info, (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        return canvas

    def render_telemetry_tab(self, fps: float, mode: str) -> np.ndarray:
        """Tab 6: Camera Control & Hardware Specifications Panel."""
        canvas = np.full((self.h, self.w, 3), (18, 20, 26), dtype=np.uint8)

        # Header Title
        cv2.putText(canvas, "ARDUCAM B0410 TOF CAMERA SPECIFICATIONS & CONTROL", (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        specs = [
            ("Camera Model", config.SENSOR_MODEL),
            ("Sensor Resolution", f"{config.SENSOR_WIDTH_PX} x {config.SENSOR_HEIGHT_PX} pixels (1/6-inch)"),
            ("Infrared VCSEL", "940 nm Laser Source (75 MHz / 37.5 MHz)"),
            ("Field of View (FOV)", f"{config.FOV_DIAGONAL_DEG}° Diagonal"),
            ("Max Depth FPS", "30 FPS (4-phase on RPi) / 120 FPS Raw"),
            ("Depth Range Modes", "Near (0.1-2.0m) | Far (0.2-4.0m)"),
            ("Current Range Mode", f"{mode} Mode"),
            ("Stream Performance", f"{fps:.1f} FPS Live"),
            ("Connection", "MIPI CSI-2 2-Lane (V4L2 Kernel Driver)")
        ]

        y0 = 85
        for i, (key, val) in enumerate(specs):
            cv2.putText(canvas, f"{key}:", (40, y0 + i * 36), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, str(val), (260, y0 + i * 36), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

        # Controls box
        cv2.rectangle(canvas, (30, self.h - 70), (self.w - 30, self.h - 20), (30, 35, 45), -1)
        cv2.putText(canvas, "Controls: Press 'M' to toggle Near/Far Mode | Press 'S' to Save Snapshot | 'Q' to Quit", (40, self.h - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        return canvas
