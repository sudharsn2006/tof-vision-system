"""
main_camera_test.py
-------------------
Author: SUDHARSAN
Main entry point for the Standalone Arducam B0410 ToF Camera Test Suite.

Renders 6 individual feature windows/tabs:
  Tab 1: Grayscale Amplitude Stream (940nm IR)
  Tab 2: Metric Depth Map (JET & Grayscale + Live Depth Probe)
  Tab 3: Contour & Distance Object Detector
  Tab 4: 3D Surface Point Cloud Visualizer
  Tab 5: RAW Phase & Confidence Diagnostics
  Tab 6: Camera Control & Telemetry Panel
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

import camera_config as config
from standalone_camera import StandaloneCamera, CameraInitError
from camera_features import CameraFeatureProcessor
from camera_ui_renderer import CameraUIRenderer


class StandaloneCameraApp:
    def __init__(self):
        self.camera = StandaloneCamera()
        self.processor = CameraFeatureProcessor()
        self.renderer = CameraUIRenderer()

        self._running = False
        self._use_colormap = True
        self._auto_rotate_3d = True
        self._probe_pt = None

        # Feature tab toggles
        self.active_tabs = {
            "tab1": True,  # Amplitude
            "tab2": True,  # Metric Depth
            "tab3": True,  # Object Detector
            "tab4": True,  # 3D Surface
            "tab5": True,  # Diagnostics
            "tab6": True   # Telemetry
        }

        os.makedirs(config.SAVE_DIR, exist_ok=True)

    def start(self):
        signal.signal(signal.SIGINT, self._handle_sigint)

        print("=" * 65)
        print(" STANDALONE ARDUCAM B0410 TOF CAMERA MULTI-TAB TEST SUITE ")
        print("=" * 65)

        self.camera.open()

        # Initialize individual feature windows
        cv2.namedWindow(config.WIN_AMPLITUDE, cv2.WINDOW_NORMAL)
        cv2.namedWindow(config.WIN_DEPTH, cv2.WINDOW_NORMAL)
        cv2.namedWindow(config.WIN_DETECTION, cv2.WINDOW_NORMAL)
        cv2.namedWindow(config.WIN_3D_SURFACE, cv2.WINDOW_NORMAL)
        cv2.namedWindow(config.WIN_DIAGNOSTICS, cv2.WINDOW_NORMAL)
        cv2.namedWindow(config.WIN_TELEMETRY, cv2.WINDOW_NORMAL)

        # Set mouse callback for depth probe on Tab 2
        cv2.setMouseCallback(config.WIN_DEPTH, self._on_depth_mouse_click)

        self._running = True
        print("[MainCameraTest] Running at 60 FPS. Controls:")
        print("  Q - Quit | S - Save Snapshot | M - Toggle Near/Far Mode")
        print("  C - Toggle JET/Grayscale Depth | A - Toggle 3D Auto-Rotate")
        print("  1-6 - Toggle individual feature tabs\n")

        self._run_loop()

    def _on_depth_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Map window display coords to 240x180 sensor pixel coords
            px = int(x * (config.SENSOR_WIDTH_PX / config.DISPLAY_WIDTH))
            py = int(y * (config.SENSOR_HEIGHT_PX / config.DISPLAY_HEIGHT))
            self._probe_pt = (px, py)

    def _run_loop(self):
        frame_interval = 1.0 / config.TARGET_FPS

        try:
            while self._running:
                loop_start = time.time()
                pkt = self.camera.get_latest_packet()

                if pkt is not None:
                    # Tab 1: Amplitude
                    if self.active_tabs["tab1"]:
                        amp_bgr = self.processor.process_amplitude_stream(pkt.amplitude)
                        frame_t1 = self.renderer.render_amplitude_tab(amp_bgr, pkt.fps)
                        cv2.imshow(config.WIN_AMPLITUDE, frame_t1)

                    # Tab 2: Depth Map
                    if self.active_tabs["tab2"]:
                        depth_bgr, probe_d = self.processor.process_depth_stream(
                            pkt.depth_m, pkt.confidence,
                            use_colormap=self._use_colormap,
                            probe_pt=self._probe_pt
                        )
                        frame_t2 = self.renderer.render_depth_tab(
                            depth_bgr, pkt.fps, self.camera.mode, probe_d, self._probe_pt
                        )
                        cv2.imshow(config.WIN_DEPTH, frame_t2)
                    else:
                        depth_bgr = None

                    # Tab 3: Object Detector
                    if self.active_tabs["tab3"]:
                        if depth_bgr is None:
                            depth_bgr, _ = self.processor.process_depth_stream(
                                pkt.depth_m, pkt.confidence, use_colormap=self._use_colormap
                            )
                        mask, obj = self.processor.detect_objects(pkt.depth_m)
                        frame_t3 = self.renderer.render_detection_tab(depth_bgr, obj, pkt.fps)
                        cv2.imshow(config.WIN_DETECTION, frame_t3)

                    # Tab 4: 3D Surface Point Cloud
                    if self.active_tabs["tab4"]:
                        x_r, y_r, z_r = self.processor.project_3d_point_cloud(
                            pkt.depth_m, yaw_deg=self.renderer.yaw_3d, pitch_deg=35.0
                        )
                        frame_t4 = self.renderer.render_3d_surface_tab(
                            x_r, y_r, z_r, pkt.fps, auto_rotate=self._auto_rotate_3d
                        )
                        cv2.imshow(config.WIN_3D_SURFACE, frame_t4)

                    # Tab 5: RAW Diagnostics
                    if self.active_tabs["tab5"]:
                        conf_hm, stats = self.processor.generate_confidence_diagnostics(pkt.confidence, pkt.depth_m)
                        frame_t5 = self.renderer.render_diagnostics_tab(conf_hm, stats, pkt.fps)
                        cv2.imshow(config.WIN_DIAGNOSTICS, frame_t5)

                    # Tab 6: Telemetry Panel
                    if self.active_tabs["tab6"]:
                        frame_t6 = self.renderer.render_telemetry_tab(pkt.fps, self.camera.mode)
                        cv2.imshow(config.WIN_TELEMETRY, frame_t6)

                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    self._handle_key(key, pkt)

                elapsed = time.time() - loop_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[MainCameraTest] Keyboard interrupt.")
        finally:
            self.shutdown()

    def _handle_key(self, key: int, pkt):
        char = chr(key).lower() if 0 <= key < 256 else ''

        if char == 'q':
            self._running = False
        elif char == 'm':
            new_mode = "NEAR" if self.camera.mode == "FAR" else "FAR"
            self.camera.set_range_mode(new_mode)
        elif char == 'c':
            self._use_colormap = not self._use_colormap
            mode_str = "JET Colormap" if self._use_colormap else "Grayscale Depth"
            print(f"[MainCameraTest] Depth display: {mode_str}")
        elif char == 'a':
            self._auto_rotate_3d = not self._auto_rotate_3d
            state = "ON" if self._auto_rotate_3d else "OFF"
            print(f"[MainCameraTest] 3D Auto-Rotate: {state}")
        elif char == 's':
            self._save_snapshot(pkt)
        elif char in ('1', '2', '3', '4', '5', '6'):
            t_name = f"tab{char}"
            self.active_tabs[t_name] = not self.active_tabs[t_name]
            state = "VISIBLE" if self.active_tabs[t_name] else "HIDDEN"
            print(f"[MainCameraTest] Feature Tab {char}: {state}")

    def _save_snapshot(self, pkt):
        if pkt is None:
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(config.SAVE_DIR, f"camera_depth_{ts}.png")
        json_path = os.path.join(config.SAVE_DIR, f"camera_telemetry_{ts}.json")

        depth_bgr, _ = self.processor.process_depth_stream(pkt.depth_m, pkt.confidence)
        cv2.imwrite(img_path, depth_bgr)

        _, obj = self.processor.detect_objects(pkt.depth_m)
        data = {
            "timestamp": ts,
            "sensor_model": config.SENSOR_MODEL,
            "resolution": [config.SENSOR_WIDTH_PX, config.SENSOR_HEIGHT_PX],
            "mode": self.camera.mode,
            "fps": pkt.fps,
            "detected_object": {
                "avg_depth_m": obj.avg_depth_m,
                "min_depth_m": obj.min_depth_m,
                "max_depth_m": obj.max_depth_m,
                "bbox": list(obj.bbox),
                "center": list(obj.center),
                "area_px": obj.area_px
            } if obj is not None else None
        }

        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\n[MainCameraTest] Saved snapshot to {img_path} and {json_path}")

    def _handle_sigint(self, signum, frame):
        print("\n[MainCameraTest] SIGINT received, shutting down...")
        self._running = False

    def shutdown(self):
        print("[MainCameraTest] Shutting down...")
        try:
            self.camera.close()
        except Exception as exc:
            print(f"[MainCameraTest] Error closing camera: {exc}")

        cv2.destroyAllWindows()
        print("[MainCameraTest] Clean shutdown complete.")


def main():
    app = StandaloneCameraApp()
    app.start()


if __name__ == "__main__":
    main()
