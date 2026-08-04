"""
lidar_3d_renderer.py
--------------------
Author: SUDHARSAN
Advanced 2D Radar and 3D Perspective Renderers for Standalone LiDAR.
Includes 3D camera auto-rotation, obstacle cluster bounding boxes, and safety HUD.
"""

from typing import List, Dict, Optional, Tuple
import numpy as np
import cv2

import lidar_config as config
from standalone_lidar import LidarPoint
from obstacle_analyzer import ObstacleCluster, ObstacleAnalyzer


class LidarRenderer2D:
    def __init__(self, width: int = config.DISPLAY_WIDTH, height: int = config.DISPLAY_HEIGHT):
        self.w = width
        self.h = height
        self.cx = width // 2
        self.cy = height // 2 + 20
        self.max_r_m = config.LIDAR_MAX_RANGE_M
        self.scale = (min(self.cx, self.cy) - 40) / self.max_r_m
        self.sweep_angle = 0.0

        # Pre-render radar background canvas
        self._bg = np.full((self.h, self.w, 3), (12, 14, 20), dtype=np.uint8)

        # Draw grid rings
        for r in [1.0, 2.0, 3.0, 5.0, 7.5, 10.0]:
            if r <= self.max_r_m:
                r_px = int(r * self.scale)
                cv2.circle(self._bg, (self.cx, self.cy), r_px, (35, 45, 55), 1, cv2.LINE_AA)
                cv2.putText(
                    self._bg, f"{r:.1f}m", (self.cx + 5, self.cy - r_px + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 115, 130), 1, cv2.LINE_AA
                )

        # Draw polar angle lines (every 30 degrees)
        for deg in range(0, 360, 30):
            rad = np.radians(deg)
            x_end = int(self.cx + (self.max_r_m * self.scale) * np.sin(rad))
            y_end = int(self.cy - (self.max_r_m * self.scale) * np.cos(rad))
            cv2.line(self._bg, (self.cx, self.cy), (x_end, y_end), (30, 38, 48), 1, cv2.LINE_AA)

        # Sensor center marker
        cv2.circle(self._bg, (self.cx, self.cy), 5, (0, 0, 255), -1, cv2.LINE_AA)

    def render(
        self,
        points: List[LidarPoint],
        clusters: List[ObstacleCluster],
        sectors: Dict[str, float],
        connected: bool,
        scan_rate_hz: float,
        show_clusters: bool = True,
        show_safety: bool = True
    ) -> np.ndarray:
        canvas = self._bg.copy()

        # 1. Draw rotating radar sweep line
        self.sweep_angle = (self.sweep_angle + 0.15) % (2 * np.pi)
        sw_x = int(self.cx + (self.max_r_m * self.scale) * np.sin(self.sweep_angle))
        sw_y = int(self.cy - (self.max_r_m * self.scale) * np.cos(self.sweep_angle))
        cv2.line(canvas, (self.cx, self.cy), (sw_x, sw_y), (0, 90, 0), 1)

        min_dist = 99.0

        if connected and points:
            x_arr = np.array([p.x for p in points])
            y_arr = np.array([p.y for p in points])
            r_arr = np.array([p.range_m for p in points])

            if len(r_arr) > 0:
                min_dist = float(np.min(r_arr))
                xs = (self.cx + x_arr * self.scale).astype(int)
                ys = (self.cy - y_arr * self.scale).astype(int)

                in_b = (xs >= 0) & (xs < self.w) & (ys >= 0) & (ys < self.h)
                xs_v = xs[in_b]
                ys_v = ys[in_b]

                # Fast vectorized 2D point array indexing
                canvas[ys_v, xs_v] = (0, 255, 255)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    canvas[np.clip(ys_v + dy, 0, self.h - 1), np.clip(xs_v + dx, 0, self.w - 1)] = (0, 200, 200)

        # 3. Draw Obstacle Clusters
        if show_clusters and clusters:
            for obs in clusters:
                pts_px = []
                for pt_x, pt_y in obs.points:
                    px = int(self.cx + pt_x * self.scale)
                    py = int(self.cy - pt_y * self.scale)
                    pts_px.append([px, py])

                pts_arr = np.array(pts_px, dtype=np.int32)
                x_b, y_b, w_b, h_b = cv2.boundingRect(pts_arr)
                cv2.rectangle(canvas, (x_b - 2, y_b - 2), (x_b + w_b + 2, y_b + h_b + 2), (0, 255, 255), 1)

                label = f"#{obs.cluster_id} {obs.distance_m:.2f}m ({obs.width_m:.2f}m)"
                cv2.putText(
                    canvas, label, (x_b, max(y_b - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1
                )

        # 4. Safety Proximity Banner
        if show_safety:
            status_text, color = ObstacleAnalyzer.get_safety_status(min_dist if min_dist < 90 else 99.0)
            cv2.rectangle(canvas, (0, 0), (self.w, 40), (20, 24, 30), -1)
            cv2.putText(canvas, status_text, (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 5. Sector Analytics Overlay
        y_sec = 60
        cv2.putText(canvas, "SECTOR READINGS", (self.w - 180, y_sec), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        for i, (name, val) in enumerate(sectors.items()):
            val_str = f"{val:.2f}m" if val < 90 else "CLEAR"
            s_color = (0, 0, 255) if val < config.SAFETY_CRITICAL_M else ((0, 255, 255) if val < config.SAFETY_WARNING_M else (200, 200, 200))
            cv2.putText(
                canvas, f"{name}: {val_str}", (self.w - 180, y_sec + 20 + i * 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, s_color, 1
            )

        footer = f"Rate: {scan_rate_hz:.1f} Hz | Points: {len(points)} | Obstacles: {len(clusters)}"
        cv2.putText(canvas, footer, (15, self.h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        return canvas


class LidarRenderer3D:
    def __init__(self, width: int = config.DISPLAY_WIDTH, height: int = config.DISPLAY_HEIGHT):
        self.w = width
        self.h = height
        self.pitch_deg = 35.0
        self.yaw_deg = 0.0
        self.auto_rotate = True
        self.focal = 400.0

    def render(
        self,
        points: List[LidarPoint],
        clusters: List[ObstacleCluster],
        connected: bool
    ) -> np.ndarray:
        canvas = np.full((self.h, self.w, 3), (10, 12, 16), dtype=np.uint8)

        if self.auto_rotate:
            self.yaw_deg = (self.yaw_deg + 1.0) % 360.0

        pitch_rad = np.radians(self.pitch_deg)
        yaw_rad = np.radians(self.yaw_deg)

        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)

        cx, cy = self.w // 2, self.h // 2 + 80

        # 1. 3D Perspective Ground Grid
        grid_color = (35, 40, 50)
        for y_m in np.arange(1.0, 11.0, 1.0):
            x1, y1 = -6.0, y_m
            x2, y2 = 6.0, y_m

            xr1 = x1 * cos_y - y1 * sin_y
            yr1 = x1 * sin_y + y1 * cos_y
            xr2 = x2 * cos_y - y2 * sin_y
            yr2 = x2 * sin_y + y2 * cos_y

            yp1 = yr1 * cos_p
            zp1 = -yr1 * sin_p
            yp2 = yr2 * cos_p
            zp2 = -yr2 * sin_p

            if yp1 > 0.1 and yp2 > 0.1:
                px1 = int(cx + (self.focal * xr1) / yp1)
                py1 = int(cy - (self.focal * zp1) / yp1)
                px2 = int(cx + (self.focal * xr2) / yp2)
                py2 = int(cy - (self.focal * zp2) / yp2)
                cv2.line(canvas, (px1, py1), (px2, py2), grid_color, 1)

        # 2. Vectorized 3D LiDAR Point Projection
        if connected and points:
            x_pts = np.array([p.x for p in points])
            y_pts = np.array([p.y for p in points])

            x_rot = x_pts * cos_y - y_pts * sin_y
            y_rot = x_pts * sin_y + y_pts * cos_y

            y_proj = y_rot * cos_p
            z_proj = -y_rot * sin_p

            valid = y_proj > 0.1
            if np.any(valid):
                px = (cx + (self.focal * x_rot[valid]) / y_proj[valid]).astype(int)
                py = (cy - (self.focal * z_proj[valid]) / y_proj[valid]).astype(int)

                in_b = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
                px_v = px[in_b]
                py_v = py[in_b]

                # Fast vectorized 3D point plot
                canvas[py_v, px_v] = (255, 255, 0)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    canvas[np.clip(py_v + dy, 0, self.h - 1), np.clip(px_v + dx, 0, self.w - 1)] = (220, 220, 0)

        # 3. Render 3D Extruded Obstacle Pillars
        if clusters:
            for obs in clusters:
                cx_obs, cy_obs = obs.center_x, obs.center_y
                xr = cx_obs * cos_y - cy_obs * sin_y
                yr = cx_obs * sin_y + cy_obs * cos_y

                if yr > 0.1:
                    # Bottom point (Z=0)
                    yp_bot = yr * cos_p
                    zp_bot = -yr * sin_p
                    px_bot = int(cx + (self.focal * xr) / yp_bot)
                    py_bot = int(cy - (self.focal * zp_bot) / yp_bot)

                    # Top point (Z=0.8m height extrusion)
                    z_top = 0.8
                    yp_top = yr * cos_p + z_top * sin_p
                    zp_top = -yr * sin_p + z_top * cos_p
                    px_top = int(cx + (self.focal * xr) / yp_top)
                    py_top = int(cy - (self.focal * zp_top) / yp_top)

                    if 0 <= px_bot < self.w and 0 <= py_bot < self.h:
                        cv2.line(canvas, (px_bot, py_bot), (px_top, py_top), (0, 0, 255), 2)
                        cv2.circle(canvas, (px_top, py_top), 4, (0, 255, 255), -1)

        # HUD Overlay
        cv2.putText(canvas, f"3D Point Cloud View (Yaw: {self.yaw_deg:.0f}° | Pitch: {self.pitch_deg:.0f}°)", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Press 'A' to toggle Auto-Rotation | 'Q' to Quit", (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA)

        return canvas
