"""
main_lidar_test.py
------------------
Author: SUDHARSAN
Main entry point for the Standalone LiDAR 2D/3D Test Suite.

Runs YDLIDAR X2 independently with real-time 2D radar, 3D point cloud,
obstacle clustering, proximity safety zone alerts, and snapshot saving.
"""

import os
import sys
import time
import signal
import json
import datetime
import cv2

# Set OpenCV GUI platform environment
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import lidar_config as config
from standalone_lidar import StandaloneLidar, LidarInitError
from obstacle_analyzer import ObstacleAnalyzer
from lidar_3d_renderer import LidarRenderer2D, LidarRenderer3D


class StandaloneLidarApp:
    def __init__(self):
        self.lidar = StandaloneLidar(port=config.LIDAR_PORT)
        self.analyzer = ObstacleAnalyzer()
        self.renderer2d = LidarRenderer2D()
        self.renderer3d = LidarRenderer3D()

        self._running = False
        self._show_2d = True
        self._show_3d = True
        self._show_clusters = True
        self._show_safety = True

        os.makedirs(config.SAVE_DIR, exist_ok=True)

    def start(self):
        signal.signal(signal.SIGINT, self._handle_sigint)

        print("=" * 60)
        print(" STANDALONE YDLIDAR X2 2D/3D TEST SUITE ")
        print("=" * 60)

        try:
            self.lidar.initialize()
            self.lidar.start_background_scan()
        except LidarInitError as exc:
            print(f"[MainLidarTest] ERROR: {exc}")
            sys.exit(1)

        cv2.namedWindow(config.WINDOW_2D_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.WINDOW_2D_NAME, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)

        cv2.namedWindow(config.WINDOW_3D_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.WINDOW_3D_NAME, config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT)

        self._running = True
        print("[MainLidarTest] Running at 60 FPS. Controls:")
        print("  Q - Quit | S - Save Snapshot | A - Toggle 3D Auto-Rotate")
        print("  C - Toggle Clusters | Z - Toggle Safety HUD | 2/3 - Toggle Views\n")

        self._run_loop()

    def _run_loop(self):
        frame_interval = 1.0 / config.TARGET_FPS

        try:
            while self._running:
                loop_start = time.time()

                scan = self.lidar.latest_scan
                connected = self.lidar.connected
                rate_hz = self.lidar.scan_rate_hz
                sectors = self.lidar.get_sector_ranges()

                # Extract obstacle clusters
                clusters = self.analyzer.extract_clusters(scan.points)

                # Render 2D View
                if self._show_2d:
                    frame2d = self.renderer2d.render(
                        scan.points, clusters, sectors, connected, rate_hz,
                        show_clusters=self._show_clusters, show_safety=self._show_safety
                    )
                    cv2.imshow(config.WINDOW_2D_NAME, frame2d)

                # Render 3D View
                if self._show_3d:
                    frame3d = self.renderer3d.render(scan.points, clusters, connected)
                    cv2.imshow(config.WINDOW_3D_NAME, frame3d)

                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    self._handle_key(key, scan, clusters)

                # FPS Pacing
                elapsed = time.time() - loop_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[MainLidarTest] Keyboard interrupt.")
        finally:
            self.shutdown()

    def _handle_key(self, key: int, scan, clusters):
        char = chr(key).lower() if 0 <= key < 256 else ''

        if char == 'q':
            self._running = False
        elif char == 's':
            self._save_snapshot(scan, clusters)
        elif char == 'a':
            self.renderer3d.auto_rotate = not self.renderer3d.auto_rotate
            state = "ON" if self.renderer3d.auto_rotate else "OFF"
            print(f"[MainLidarTest] 3D Auto-Rotate: {state}")
        elif char == 'c':
            self._show_clusters = not self._show_clusters
            print(f"[MainLidarTest] Obstacle Clusters Display: {self._show_clusters}")
        elif char == 'z':
            self._show_safety = not self._show_safety
            print(f"[MainLidarTest] Safety Proximity HUD: {self._show_safety}")
        elif char == '2':
            self._show_2d = not self._show_2d
            print(f"[MainLidarTest] 2D Window Display: {self._show_2d}")
        elif char == '3':
            self._show_3d = not self._show_3d
            print(f"[MainLidarTest] 3D Window Display: {self._show_3d}")

    def _save_snapshot(self, scan, clusters):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(config.SAVE_DIR, f"lidar_scan_{ts}.png")
        json_path = os.path.join(config.SAVE_DIR, f"lidar_scan_{ts}.json")

        if self._show_2d:
            frame = self.renderer2d.render(
                scan.points, clusters, self.lidar.get_sector_ranges(),
                self.lidar.connected, self.lidar.scan_rate_hz
            )
            cv2.imwrite(img_path, frame)

        data = {
            "timestamp": ts,
            "scan_time": scan.scan_time,
            "point_count": scan.point_count,
            "obstacle_count": len(clusters),
            "obstacles": [
                {
                    "id": c.cluster_id,
                    "distance_m": c.distance_m,
                    "angle_deg": c.angle_deg,
                    "width_m": c.width_m,
                    "point_count": c.point_count,
                    "center": [c.center_x, c.center_y]
                }
                for c in clusters
            ],
            "points": [[p.angle_rad, p.range_m, p.intensity] for p in scan.points]
        }

        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\n[MainLidarTest] Saved snapshot to {img_path} and {json_path}")

    def _handle_sigint(self, signum, frame):
        print("\n[MainLidarTest] SIGINT received, shutting down...")
        self._running = False

    def shutdown(self):
        print("[MainLidarTest] Shutting down...")
        try:
            self.lidar.stop()
        except Exception as exc:
            print(f"[MainLidarTest] Error stopping LiDAR: {exc}")

        cv2.destroyAllWindows()
        print("[MainLidarTest] Clean shutdown complete.")


def main():
    app = StandaloneLidarApp()
    app.start()


if __name__ == "__main__":
    main()
