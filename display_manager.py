"""
display_manager.py
--------------------
Author: SUDHARSAN
Creates and updates the two required OpenCV windows:

  1. "Depth View"      - live false-color depth image with a simple
                          bounding box / center / distance overlay.
  2. "Object Distance" - a plain info panel summarizing the currently
                          detected object (bbox, center, current + average
                          distance).

No extra HUD, crosshairs, or decorative graphics are drawn, per spec.
"""

from typing import Optional, List, Tuple

import numpy as np
import cv2

import config
from object_detector import DetectedObject


class DisplayManager:
    def __init__(self):
        cv2.namedWindow(config.DEPTH_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.DEPTH_WINDOW_NAME, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)

        cv2.namedWindow(config.OBJECT_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.OBJECT_WINDOW_NAME, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)

        cv2.namedWindow(config.LIDAR_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.LIDAR_WINDOW_NAME, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)

        # Pre-render static radar background canvas for 60 FPS performance
        w, h = config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT
        self._lidar_background = np.full((h, w, 3), (15, 15, 20), dtype=np.uint8)
        self.cx, self.cy = w // 2, h // 2
        self.max_r_m = config.LIDAR_MAX_RANGE_M
        self.scale = (min(self.cx, self.cy) - 30) / self.max_r_m

        grid_ranges = [1.0, 2.0, 4.0, 6.0, 8.0]
        for r in grid_ranges:
            if r <= self.max_r_m:
                r_px = int(r * self.scale)
                cv2.circle(self._lidar_background, (self.cx, self.cy), r_px, (40, 40, 50), 1, cv2.LINE_AA)
                cv2.putText(
                    self._lidar_background, f"{r:.1f}m", (self.cx + 5, self.cy - r_px + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 120), 1, cv2.LINE_AA
                )

        cv2.line(self._lidar_background, (self.cx, 10), (self.cx, h - 10), (40, 40, 50), 1)
        cv2.line(self._lidar_background, (10, self.cy), (w - 10, self.cy), (40, 40, 50), 1)
        cv2.circle(self._lidar_background, (self.cx, self.cy), 5, (0, 0, 255), -1, cv2.LINE_AA)

    def update_depth_view(self, depth_color: np.ndarray, obj: Optional[DetectedObject]):
        """Draws the bounding box, center point, and distance text over the
        depth image (grayscale or colormap), then shows it in the depth window."""
        frame = depth_color.copy()

        if obj is not None:
            x, y, w, h = obj.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, obj.center, 4, (0, 255, 0), -1)

            text = f"{obj.avg_distance_m:.2f} m"
            cv2.putText(
                frame, text, (x, max(y - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA
            )

        cv2.imshow(config.DEPTH_WINDOW_NAME, frame)

    def update_object_window(self, obj: Optional[DetectedObject]):
        """Renders a simple, uncluttered info panel describing the current
        detection state into the 'Object Distance' window."""
        panel = np.zeros((config.DISPLAY_HEIGHT, config.DISPLAY_WIDTH, 3), dtype=np.uint8)

        if obj is None:
            cv2.putText(
                panel, "NO OBJECT DETECTED", (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA
            )
            cv2.imshow(config.OBJECT_WINDOW_NAME, panel)
            return

        x, y, w, h = obj.bbox
        lines = [
            "OBJECT DETECTED",
            "",
            f"Bounding Box: x={x}, y={y}, w={w}, h={h}",
            f"Center: ({obj.center[0]}, {obj.center[1]})",
            f"Current Distance: {obj.avg_distance_m:.2f} m",
            f"Min Distance: {obj.min_distance_m:.2f} m",
            f"Max Distance: {obj.max_distance_m:.2f} m",
            f"Object Area: {obj.area_px} px",
        ]

        y0 = 50
        for i, line in enumerate(lines):
            color = (0, 255, 0) if i == 0 else (255, 255, 255)
            scale = 0.9 if i == 0 else 0.6
            cv2.putText(
                panel, line, (30, y0 + i * 35),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA
            )

        cv2.imshow(config.OBJECT_WINDOW_NAME, panel)

    def update_lidar_window(self, points: List[Tuple[float, float, float]], connected: bool, scan_rate_hz: float):
        """Renders a 2D top-down polar radar scan visual in a separate window."""
        canvas = self._lidar_background.copy()
        w, h = config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT

        # Draw scan points
        if connected and points:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])

            valid = (ranges >= config.LIDAR_MIN_RANGE_M) & (ranges <= self.max_r_m)
            valid_angles = angles[valid]
            valid_ranges = ranges[valid]

            if len(valid_angles) > 0:
                # Auto-detect degrees vs radians
                if np.max(np.abs(valid_angles)) > 7.0:
                    valid_angles = np.radians(valid_angles)

                xs = self.cx + (valid_ranges * np.sin(valid_angles) * self.scale).astype(int)
                ys = self.cy - (valid_ranges * np.cos(valid_angles) * self.scale).astype(int)

                in_bounds = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
                xs_valid = xs[in_bounds]
                ys_valid = ys[in_bounds]

                # Draw high-visibility bright cyan dots (radius 2 filled)
                for px, py in zip(xs_valid, ys_valid):
                    cv2.circle(canvas, (int(px), int(py)), 2, (0, 255, 255), -1, cv2.LINE_AA)

        # Draw sensor center icon
        cv2.circle(canvas, (self.cx, self.cy), 5, (0, 0, 255), -1, cv2.LINE_AA)

        # Overlay status text
        status_str = f"LiDAR 2D View | Rate: {scan_rate_hz:.1f} Hz | Points: {len(points)}" if connected else "LiDAR 2D View | Disconnected"
        color = (0, 255, 255) if connected else (0, 0, 255)
        cv2.putText(canvas, status_str, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        cv2.imshow(config.LIDAR_WINDOW_NAME, canvas)

    def update_3d_window(self, depth_clean: Optional[np.ndarray], points: List[Tuple[float, float, float]], connected: bool):
        """Renders a real-time 3D Perspective Point Cloud fusing ToF Depth and LiDAR data."""
        w, h = config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT
        canvas = np.full((h, w, 3), (12, 12, 18), dtype=np.uint8)

        cx, cy = w // 2, h // 2 + 60
        focal = 320.0
        pitch = np.radians(35.0)  # 35 deg tilt
        cos_p, sin_p = np.cos(pitch), np.sin(pitch)

        # 1. Draw 3D Perspective Ground Grid (X: -4m to +4m, Y: 0.5m to 6.0m, Z=0)
        grid_color = (40, 45, 55)
        for y_m in np.arange(1.0, 6.5, 1.0):
            x1, y1, z1 = -4.0, y_m, 0.0
            x2, y2, z2 = 4.0, y_m, 0.0
            # Project line endpoints
            y_r1 = y1 * cos_p + z1 * sin_p
            z_r1 = -y1 * sin_p + z1 * cos_p
            px1 = int(cx + (focal * x1) / max(y_r1, 0.1))
            py1 = int(cy - (focal * z_r1) / max(y_r1, 0.1))

            y_r2 = y2 * cos_p + z2 * sin_p
            z_r2 = -y2 * sin_p + z2 * cos_p
            px2 = int(cx + (focal * x2) / max(y_r2, 0.1))
            py2 = int(cy - (focal * z_r2) / max(y_r2, 0.1))

            cv2.line(canvas, (px1, py1), (px2, py2), grid_color, 1)

        for x_m in np.arange(-4.0, 4.5, 1.0):
            y_r1 = 1.0 * cos_p
            z_r1 = -1.0 * sin_p
            px1 = int(cx + (focal * x_m) / max(y_r1, 0.1))
            py1 = int(cy - (focal * z_r1) / max(y_r1, 0.1))

            y_r2 = 6.0 * cos_p
            z_r2 = -6.0 * sin_p
            px2 = int(cx + (focal * x_m) / max(y_r2, 0.1))
            py2 = int(cy - (focal * z_r2) / max(y_r2, 0.1))

            cv2.line(canvas, (px1, py1), (px2, py2), grid_color, 1)

        # 2. Render 3D ToF Camera Point Cloud
        if depth_clean is not None and depth_clean.size > 0:
            dh, dw = depth_clean.shape[:2]
            step = 6  # Downsample for 60 FPS fast 3D projection
            v_coords, u_coords = np.mgrid[0:dh:step, 0:dw:step]
            depth_samples = depth_clean[::step, ::step]

            valid_cam = (depth_samples > 0.1) & (depth_samples <= 4.0)
            z_cam = depth_samples[valid_cam]

            if z_cam.size > 0:
                u_samp = u_coords[valid_cam]
                v_samp = v_coords[valid_cam]

                # Convert pixel + depth into 3D spatial points (x_cam, y_cam, z_cam)
                fx_cam, fy_cam = dw * 0.8, dw * 0.8
                cx_cam, cy_cam = dw / 2.0, dh / 2.0
                x_cam = (u_samp - cx_cam) * z_cam / fx_cam
                y_spatial = z_cam
                z_spatial = -(v_samp - cy_cam) * z_cam / fy_cam

                # Rotate into 3D perspective
                y_rot = y_spatial * cos_p + z_spatial * sin_p
                z_rot = -y_spatial * sin_p + z_spatial * cos_p

                valid_proj = y_rot > 0.1
                px_cam = (cx + (focal * x_cam[valid_proj]) / y_rot[valid_proj]).astype(int)
                py_cam = (cy - (focal * z_rot[valid_proj]) / y_rot[valid_proj]).astype(int)

                in_view = (px_cam >= 0) & (px_cam < w) & (py_cam >= 0) & (py_cam < h)
                px_v = px_cam[in_view]
                py_v = py_cam[in_view]

                # Draw warm orange/yellow 3D ToF points
                for px_i, py_i in zip(px_v, py_v):
                    canvas[py_i, px_i] = (0, 200, 255)

        # 3. Render 3D LiDAR Scan Points (X, Y, Z=0)
        if connected and points:
            angles = np.array([p[0] for p in points])
            ranges = np.array([p[1] for p in points])
            valid_lidar = (ranges >= config.LIDAR_MIN_RANGE_M) & (ranges <= config.LIDAR_MAX_RANGE_M)
            val_a = angles[valid_lidar]
            val_r = ranges[valid_lidar]

            if len(val_a) > 0:
                if np.max(np.abs(val_a)) > 7.0:
                    val_a = np.radians(val_a)

                x_lid = val_r * np.sin(val_a)
                y_lid = val_r * np.cos(val_a)
                z_lid = np.zeros_like(x_lid)

                y_rot_l = y_lid * cos_p + z_lid * sin_p
                z_rot_l = -y_lid * sin_p + z_lid * cos_p

                val_l = y_rot_l > 0.1
                px_lid = (cx + (focal * x_lid[val_l]) / y_rot_l[val_l]).astype(int)
                py_lid = (cy - (focal * z_rot_l[val_l]) / y_rot_l[val_l]).astype(int)

                in_view_l = (px_lid >= 0) & (px_lid < w) & (py_lid >= 0) & (py_lid < h)
                for px_i, py_i in zip(px_lid[in_view_l], py_lid[in_view_l]):
                    cv2.circle(canvas, (int(px_i), int(py_i)), 2, (255, 255, 0), -1, cv2.LINE_AA)

        # Draw Origin Marker
        cv2.circle(canvas, (cx, int(cy - (focal * (-1.0 * sin_p)) / (1.0 * cos_p))), 4, (0, 0, 255), -1, cv2.LINE_AA)

        # Status HUD
        cv2.putText(canvas, "3D Point Cloud View (ToF + LiDAR Fusion)", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Cyan: 3D LiDAR | Orange: 3D ToF Camera", (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow(config.VIEW_3D_WINDOW_NAME, canvas)

    @staticmethod
    def wait_key(delay_ms: int = 1) -> int:
        return cv2.waitKey(delay_ms) & 0xFF

    @staticmethod
    def close_all():
        cv2.destroyAllWindows()
