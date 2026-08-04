"""
standalone_camera.py
--------------------
Author: SUDHARSAN
Threaded Arducam B0410 SDK wrapper and frame polling engine.
Extracts Depth, Grayscale Amplitude, and Confidence maps into a non-blocking queue.
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

try:
    import ArducamDepthCamera as ac
    ARDUCAM_SDK_AVAILABLE = True
except ImportError:
    ARDUCAM_SDK_AVAILABLE = False

import camera_config as config


class CameraInitError(Exception):
    """Raised when camera initialization fails."""
    pass


@dataclass
class CameraFramePacket:
    depth_m: np.ndarray          # 240x180 float32 (meters)
    amplitude: np.ndarray        # 240x180 float32 (IR intensity)
    confidence: np.ndarray       # 240x180 float32 (0-255)
    timestamp: float = 0.0
    fps: float = 0.0


class StandaloneCamera:
    def __init__(self):
        self._camera = None
        self._is_open = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self.latest_packet: Optional[CameraFramePacket] = None
        self.actual_fps = 0.0
        self.mode = config.RANGE_MODE
        self._frame_count = 0
        self._last_fps_calc = time.time()

    def open(self):
        """Initializes CSI ToF camera and starts acquisition thread."""
        if not ARDUCAM_SDK_AVAILABLE:
            print("[StandaloneCamera] SDK not found. Running in Synthetic Demonstration Mode.")
            self._is_open = True
            self._running = True
            self._thread = threading.Thread(target=self._synthetic_loop, daemon=True)
            self._thread.start()
            return

        try:
            self._camera = ac.ArducamCamera()
            ret = self._camera.open(ac.Connection.CSI, 0)
            if ret != 0:
                raise CameraInitError(f"Failed to open Arducam B0410 camera (code {ret}).")

            ret = self._camera.start(ac.FrameType.DEPTH)
            if ret != 0:
                raise CameraInitError(f"Failed to start depth stream (code {ret}).")

            max_range = config.MAX_RANGE_FAR_MM if self.mode == "FAR" else config.MAX_RANGE_NEAR_MM
            try:
                self._camera.setControl(ac.Control.RANGE, max_range)
            except Exception:
                pass

            self._is_open = True
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
            print("[StandaloneCamera] Arducam B0410 ToF Camera initialized successfully.")

        except Exception as exc:
            print(f"[StandaloneCamera] Notice: Hardware init fallback: {exc}. Starting Synthetic Mode.")
            self._is_open = True
            self._running = True
            self._thread = threading.Thread(target=self._synthetic_loop, daemon=True)
            self._thread.start()

    def _capture_loop(self):
        """Real hardware frame polling loop."""
        while self._running and self._is_open:
            try:
                frame = self._camera.requestFrame(100)
                if frame is not None:
                    try:
                        depth_buf = frame.depth_data
                        conf_buf = getattr(frame, "confidence_data", None)
                        amp_buf = getattr(frame, "amplitude_data", None)

                        depth_m = depth_buf.astype(np.float32) / 1000.0
                        h, w = depth_m.shape[:2]

                        if conf_buf is not None:
                            conf = conf_buf.astype(np.float32)
                        else:
                            conf = np.full((h, w), 255.0, dtype=np.float32)

                        if amp_buf is not None:
                            amp = amp_buf.astype(np.float32)
                        else:
                            amp = np.clip(1.0 - (depth_m / 4.0), 0.0, 1.0) * 255.0

                        now = time.time()
                        self._frame_count += 1
                        if (now - self._last_fps_calc) >= 1.0:
                            self.actual_fps = self._frame_count / (now - self._last_fps_calc)
                            self._frame_count = 0
                            self._last_fps_calc = now

                        packet = CameraFramePacket(
                            depth_m=depth_m,
                            amplitude=amp,
                            confidence=conf,
                            timestamp=now,
                            fps=self.actual_fps
                        )

                        with self._lock:
                            self.latest_packet = packet
                    finally:
                        self._camera.releaseFrame(frame)
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.01)

    def _synthetic_loop(self):
        """Generates synthetic 240x180 frames for testing without camera attached."""
        h, w = config.SENSOR_HEIGHT_PX, config.SENSOR_WIDTH_PX
        t = 0.0
        while self._running and self._is_open:
            t += 0.05
            y, x = np.mgrid[0:h, 0:w]
            cx_t = w / 2 + np.sin(t) * 40
            cy_t = h / 2 + np.cos(t) * 30

            r = np.hypot(x - cx_t, y - cy_t)
            depth_m = 0.8 + 1.2 * (r / np.hypot(w, h))
            amplitude = np.clip(255.0 * (1.0 - r / np.hypot(w, h)), 20, 255)
            confidence = np.full((h, w), 240.0, dtype=np.float32)

            now = time.time()
            self._frame_count += 1
            if (now - self._last_fps_calc) >= 1.0:
                self.actual_fps = self._frame_count / (now - self._last_fps_calc)
                self._frame_count = 0
                self._last_fps_calc = now

            packet = CameraFramePacket(
                depth_m=depth_m.astype(np.float32),
                amplitude=amplitude.astype(np.float32),
                confidence=confidence,
                timestamp=now,
                fps=self.actual_fps
            )

            with self._lock:
                self.latest_packet = packet

            time.sleep(0.033)

    def get_latest_packet(self) -> Optional[CameraFramePacket]:
        with self._lock:
            return self.latest_packet

    def set_range_mode(self, mode: str):
        """Toggle Near (0.1-2.0m) vs Far (0.2-4.0m) mode."""
        self.mode = mode
        if self._camera is not None and self._is_open:
            max_range = config.MAX_RANGE_FAR_MM if mode == "FAR" else config.MAX_RANGE_NEAR_MM
            try:
                self._camera.setControl(ac.Control.RANGE, max_range)
                print(f"[StandaloneCamera] Mode set to {mode} ({max_range}mm max range).")
            except Exception as exc:
                print(f"[StandaloneCamera] Control notice: {exc}")

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if self._camera is not None and self._is_open:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception:
                pass
        self._is_open = False
        print("[StandaloneCamera] Camera streams stopped.")
