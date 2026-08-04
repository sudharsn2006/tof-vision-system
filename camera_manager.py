"""
camera_manager.py
------------------
Author: SUDHARSAN
Wraps the official ArducamDepthCamera SDK for the B0410 ToF AI Camera.

Responsibilities:
  * Open/close the CSI ToF sensor
  * Pull raw depth + confidence/amplitude frames
  * Convert raw depth into a metric (meters) numpy array
  * Provide a clean, minimal-copy interface to the rest of the app

NOTE ON SDK INSTALLATION:
  The Arducam ToF camera is driven by the "ArducamDepthCamera" Python
  package, distributed by Arducam (github.com/ArduCAM/Arducam_tof_camera).
  Install it per Arducam's instructions for Raspberry Pi OS 64-bit, e.g.:

      git clone https://github.com/ArduCAM/Arducam_tof_camera.git
      cd Arducam_tof_camera
      ./Install_dependencies.sh
      pip3 install ArducamDepthCamera

  The exact class/method names below follow the SDK's documented API
  (ArducamCamera, TOFConnect, TOFFrameFormat, requestFrame, etc). If Arducam
  changes the SDK signature in a future release, only this file should need
  updating -- the rest of the application only talks to CameraManager.
"""

import threading
import time
import numpy as np

try:
    import ArducamDepthCamera as ac
    ARDUCAM_SDK_AVAILABLE = True
except ImportError:
    ARDUCAM_SDK_AVAILABLE = False


class CameraInitError(Exception):
    """Raised when the ToF camera cannot be opened or configured."""
    pass


class CameraManager:
    """
    Manages the lifecycle of the Arducam B0410 ToF camera and exposes
    a non-blocking get_depth_frame() call backed by a background acquisition thread.
    """

    def __init__(self, max_range_mm: int):
        self.max_range_mm = max_range_mm
        self._camera = None
        self._is_open = False
        self._width = 0
        self._height = 0
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._latest_depth_m = None
        self._latest_confidence = None

    def open(self):
        """Initialize and start the ToF camera and background frame thread."""
        if not ARDUCAM_SDK_AVAILABLE:
            raise CameraInitError(
                "ArducamDepthCamera SDK not found. Install it per Arducam's "
                "official instructions (see camera_manager.py header)."
            )

        try:
            self._camera = ac.ArducamCamera()

            ret = self._camera.open(ac.Connection.CSI, 0)
            if ret != 0:
                raise CameraInitError(f"Failed to open ToF camera (error code {ret}).")

            ret = self._camera.start(ac.FrameType.DEPTH)
            if ret != 0:
                raise CameraInitError(f"Failed to start depth stream (error code {ret}).")

            try:
                self._camera.setControl(ac.Control.RANGE, self.max_range_mm)
            except Exception:
                pass

            self._is_open = True
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
            print("[CameraManager] Arducam B0410 ToF camera initialized successfully (threaded mode).")

        except CameraInitError:
            raise
        except Exception as exc:
            raise CameraInitError(f"Unexpected error initializing ToF camera: {exc}")

    def _capture_loop(self):
        """Background thread constantly polling frames from the SDK."""
        import config
        while self._running and self._is_open:
            try:
                frame = self._camera.requestFrame(100)
                if frame is not None:
                    try:
                        confidence_buf = getattr(frame, "confidence_data", None)
                        amplitude_buf = getattr(frame, "amplitude_data", None)

                        if getattr(config, "CAMERA_MODE", "DEPTH") == "NORMAL" and amplitude_buf is not None and amplitude_buf.size > 0:
                            # Normal camera IR/Amplitude video feed (ultra-fast, zero depth lag)
                            amp_float = amplitude_buf.astype(np.float32)
                            max_val = np.max(amp_float)
                            depth_m = (amp_float / max_val) if max_val > 0 else amp_float
                        else:
                            # Depth mode
                            depth_buf = frame.depth_data
                            depth_m = depth_buf.astype(np.float32) / 1000.0

                        if confidence_buf is not None:
                            confidence = confidence_buf.astype(np.float32)
                        else:
                            confidence = np.full(depth_m.shape, 255.0, dtype=np.float32)

                        with self._lock:
                            self._latest_depth_m = depth_m
                            self._latest_confidence = confidence
                            self._height, self._width = depth_m.shape[:2]
                    finally:
                        self._camera.releaseFrame(frame)
                else:
                    time.sleep(0.005)
            except Exception as exc:
                time.sleep(0.01)

    def get_depth_frame(self, timeout_ms: int = 0):
        """
        Instant non-blocking retrieval of the latest captured depth frame.
        """
        with self._lock:
            if self._latest_depth_m is None:
                return None, None
            return self._latest_depth_m.copy(), self._latest_confidence.copy()

    def close(self):
        """Stop background thread and release camera device."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if self._camera is not None and self._is_open:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception as exc:
                print(f"[CameraManager] Warning while closing camera: {exc}")
        self._is_open = False
        print("[CameraManager] Camera closed.")

    @property
    def is_open(self):
        return self._is_open

    @property
    def resolution(self):
        return self._width, self._height
