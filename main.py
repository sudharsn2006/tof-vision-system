"""
main.py
--------
Author: SUDHARSAN
Entry point for the ToF Depth Object Detection + LiDAR Sensor Test App.

Pipeline per frame (ToF only):
    raw depth frame -> DepthProcessor.clean() -> DepthProcessor.threshold()
    -> ObjectDetector.detect() -> DisplayManager (draw + show)
    -> terminal status print (only on change)

The YDLIDAR X2 runs independently in the background (see lidar_manager.py)
and is only used here to print connection/status info -- it does NOT feed
into object detection in this version.

Keyboard controls (focus must be on an OpenCV window):
    Q - Quit
    S - Save current depth image (color-mapped) to ./captures/
    R - Reset detection (clears the "already printed" status so the next
        frame's state is printed fresh, and forces re-evaluation)
    L - Toggle LiDAR status overlay on/off in the terminal log
"""

import os
import sys
import time
import signal
import datetime

# Ensure OpenCV GUI backend runs cleanly on Raspberry Pi OS Desktop
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2

import config
from camera_manager import CameraManager, CameraInitError
from depth_processor import DepthProcessor
from object_detector import ObjectDetector
from display_manager import DisplayManager
from lidar_manager import LiDARManager, LidarInitError


class Application:
    def __init__(self):
        self.camera = CameraManager(max_range_mm=config.TOF_MAX_RANGE_MM)
        self.depth_processor = DepthProcessor()
        self.object_detector = ObjectDetector()
        self.display = None  # created after camera is confirmed working
        self.lidar = LiDARManager(port=config.LIDAR_PORT)

        self._running = False
        self._last_object_present = None  # None = unknown, True/False = last printed state
        self._lidar_display_enabled = True
        self._detection_distance_mm = config.DETECTION_DISTANCE_MM

        os.makedirs(config.SAVE_DIR, exist_ok=True)

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------
    def start(self):
        signal.signal(signal.SIGINT, self._handle_sigint)

        self._start_camera()
        self._start_lidar()  # non-fatal if it fails; camera can still run standalone

        self.display = DisplayManager()
        self._running = True
        print("[Main] Application started. Press Q in an OpenCV window to quit.")
        self._run_loop()

    def _start_camera(self):
        try:
            self.camera.open()
        except CameraInitError as exc:
            print(f"[Main] FATAL: {exc}")
            sys.exit(1)

    def _start_lidar(self):
        try:
            self.lidar.initialize()
            self.lidar.start_background_scan()
        except LidarInitError as exc:
            print(f"[Main] WARNING: LiDAR unavailable ({exc}). "
                  f"Continuing with ToF-only detection.")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _run_loop(self):
        frame_interval = 1.0 / config.TARGET_FPS
        last_lidar_print = 0.0

        try:
            while self._running:
                loop_start = time.time()

                depth_m, confidence = self.camera.get_depth_frame()
                if depth_m is None:
                    time.sleep(0.005)
                    continue

                depth_clean = self.depth_processor.clean(depth_m, confidence)
                mask = self.depth_processor.threshold(depth_clean, self._detection_distance_mm)
                obj = self.object_detector.detect(mask, depth_clean)

                depth_display = self.depth_processor.to_display_image(
                    depth_clean, config.TOF_MAX_RANGE_MM, grayscale=config.USE_GRAYSCALE_DEPTH
                )
                self.display.update_depth_view(depth_display, obj)
                self.display.update_object_window(obj)

                # Update LiDAR 2D Visual window
                lidar_snap = self.lidar.get_status_snapshot()
                self.display.update_lidar_window(
                    self.lidar.latest_scan.points,
                    lidar_snap["connected"],
                    lidar_snap["scan_rate_hz"]
                )

                # Update 3D Point Cloud View window
                if getattr(config, "ENABLE_3D_VIEW", True):
                    self.display.update_3d_window(
                        depth_clean,
                        self.lidar.latest_scan.points,
                        lidar_snap["connected"]
                    )

                self._print_status_on_change(obj)

                if self._lidar_display_enabled and (time.time() - last_lidar_print) >= 2.0:
                    self._print_lidar_status()
                    last_lidar_print = time.time()

                key = self.display.wait_key(1)
                self._handle_key(key, depth_display)

                if key in (ord('q'), ord('Q')):
                    self._running = False

                # Simple FPS pacing so we don't burn CPU faster than needed.
                elapsed = time.time() - loop_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("[Main] Keyboard interrupt received.")
        finally:
            self.shutdown()

    # ------------------------------------------------------------------
    # Terminal output
    # ------------------------------------------------------------------
    def _print_status_on_change(self, obj):
        is_present = obj is not None

        if is_present == self._last_object_present:
            if is_present:
                self._print_object_line(obj)
            return

        self._last_object_present = is_present

        if is_present:
            print("\n--- OBJECT DETECTED ---")
            self._print_object_line(obj, full=True)
        else:
            print("\nNO OBJECT DETECTED")

    def _print_object_line(self, obj, full: bool = False):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        if full:
            print(f"Distance:      {obj.avg_distance_m:.2f} meters "
                  f"(min {obj.min_distance_m:.2f} / max {obj.max_distance_m:.2f})")
            print(f"Object Center: ({obj.center[0]}, {obj.center[1]})")
            print(f"Object Area:   {obj.area_px} pixels")
            print(f"Timestamp:     {ts}")
        else:
            print(f"[{ts}] Distance: {obj.avg_distance_m:.2f} m | "
                  f"Center: ({obj.center[0]}, {obj.center[1]}) | "
                  f"Area: {obj.area_px} px", end="\r", flush=True)

    def _print_lidar_status(self):
        status = self.lidar.get_status_snapshot()
        if status["connected"]:
            print(f"\n[LiDAR] Connected | Scan Rate: {status['scan_rate_hz']} Hz | "
                  f"Points: {status['point_count']}")
        else:
            print("\n[LiDAR] Not connected.")

    # ------------------------------------------------------------------
    # Keyboard handling
    # ------------------------------------------------------------------
    def _handle_key(self, key: int, depth_display):
        char = chr(key).lower() if 0 <= key < 256 else ''
        if char == 's':
            self._save_frame(depth_display)
        elif char == 'r':
            self._reset_detection()
        elif char == 'l':
            self._lidar_display_enabled = not self._lidar_display_enabled
            state = "enabled" if self._lidar_display_enabled else "disabled"
            print(f"\n[Main] LiDAR status display {state}.")

    def _save_frame(self, depth_color):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(config.SAVE_DIR, f"depth_{ts}.png")
        cv2.imwrite(path, depth_color)
        print(f"\n[Main] Saved current depth image to {path}")

    def _reset_detection(self):
        self._last_object_present = None
        print("\n[Main] Detection state reset.")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _handle_sigint(self, signum, frame):
        print("\n[Main] SIGINT received, shutting down...")
        self._running = False

    def shutdown(self):
        print("[Main] Shutting down...")
        try:
            self.camera.close()
        except Exception as exc:
            print(f"[Main] Error closing camera: {exc}")

        try:
            self.lidar.stop()
        except Exception as exc:
            print(f"[Main] Error stopping LiDAR: {exc}")

        if self.display is not None:
            self.display.close_all()

        print("[Main] Shutdown complete.")


def main():
    app = Application()
    app.start()


if __name__ == "__main__":
    main()
