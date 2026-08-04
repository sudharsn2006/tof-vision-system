"""
human_detection.py
------------------
Author: SUDHARSAN
Focused human detection application for the Arducam B0410 ToF Camera.

Windows:
  - DEPTH    : JET colormap depth map at native 240x180 - clean, no HUD
  - CONFIDENCE: Arducam-style black/gray confidence map at native 240x180
  - DETECTION : Human silhouette bounding box on depth image at native 240x180

Design goals:
  - Native 240x180 output (zero resize cost, displayed at screen origin)
  - All heavy tabs (3D projection, telemetry panel) removed
  - Single frame-grab per loop cycle, shared across all three windows
  - Confidence visualised as pure black/gray (low=black, high=white) matching
    the Arducam_tof_camera SDK example style
  - Zero HUD text drawn onto any frame
"""

import os
import sys
import time
import signal
import threading
from typing import Optional, Tuple

import cv2
import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

try:
    import ArducamDepthCamera as ac
    _SDK = True
except ImportError:
    _SDK = False

# ─────────────────────────── camera constants ───────────────────────────────
W, H              = 240, 180          # native sensor resolution
MIN_DEPTH_M       = 0.1               # meters – minimum valid reading
MAX_DEPTH_M       = 2.0               # meters – NEAR mode (best for humans ≤2 m)
CONFIDENCE_THRESH = 30                # drop pixels below this confidence value
MIN_HUMAN_PX      = 400               # min contour area to count as a human blob
MAX_HUMAN_DIST_M  = 2.0               # detection range cap (meters)

WIN_DEPTH  = "Depth"
WIN_CONF   = "Confidence"
WIN_DETECT = "Detection"


# ─────────────────────────── camera thread ──────────────────────────────────
class FramePacket:
    __slots__ = ("depth_m", "amplitude", "confidence")

    def __init__(self, depth_m: np.ndarray, amplitude: np.ndarray, confidence: np.ndarray):
        self.depth_m    = depth_m
        self.amplitude  = amplitude
        self.confidence = confidence


class CameraThread(threading.Thread):
    """Background thread: polls Arducam SDK (or synthetic fallback) and
    keeps only the newest frame – the main loop always reads the latest."""

    def __init__(self):
        super().__init__(daemon=True)
        self._lock   = threading.Lock()
        self._packet: Optional[FramePacket] = None
        self._running = False
        self._camera  = None

    # ── public API ──────────────────────────────────────────────────────────
    def start_camera(self) -> bool:
        """Open the camera.  Returns True on success."""
        if _SDK:
            try:
                cam = ac.ArducamCamera()
                if cam.open(ac.Connection.CSI, 0) != 0:
                    raise RuntimeError("Camera open failed")
                if cam.start(ac.FrameType.DEPTH) != 0:
                    raise RuntimeError("Stream start failed")
                # Use NEAR mode: best SNR for human-sized targets ≤ 2 m
                try:
                    cam.setControl(ac.Control.RANGE, int(MAX_DEPTH_M * 1000))
                except Exception:
                    pass
                self._camera = cam
                print("[CameraThread] Arducam B0410 opened in NEAR mode (0.1–2.0 m).")
                return True
            except Exception as exc:
                print(f"[CameraThread] Hardware init failed ({exc}) – using synthetic data.")
        else:
            print("[CameraThread] ArducamDepthCamera SDK not found – using synthetic data.")
        return False

    def get_latest(self) -> Optional[FramePacket]:
        with self._lock:
            return self._packet

    def stop(self):
        self._running = False

    # ── thread body ─────────────────────────────────────────────────────────
    def run(self):
        self._running = True
        if self._camera is not None:
            self._run_hardware()
        else:
            self._run_synthetic()

    def _run_hardware(self):
        """Real camera polling loop – request / release as fast as possible."""
        while self._running:
            try:
                frame = self._camera.requestFrame(80)   # 80 ms timeout
                if frame is None:
                    continue
                try:
                    depth_mm = frame.depth_data          # uint16, mm
                    conf_raw = getattr(frame, "confidence_data", None)
                    amp_raw  = getattr(frame, "amplitude_data",  None)

                    depth_m = depth_mm.astype(np.float32) / 1000.0

                    conf = conf_raw.astype(np.float32) if conf_raw is not None \
                           else np.full((H, W), 255.0, np.float32)

                    amp  = amp_raw.astype(np.float32)  if amp_raw  is not None \
                           else np.clip(1.0 - depth_m / MAX_DEPTH_M, 0.0, 1.0) * 255.0

                    pkt = FramePacket(depth_m=depth_m, amplitude=amp, confidence=conf)
                    with self._lock:
                        self._packet = pkt
                finally:
                    self._camera.releaseFrame(frame)
            except Exception:
                time.sleep(0.01)

        # graceful teardown
        try:
            self._camera.stop()
            self._camera.close()
        except Exception:
            pass

    def _run_synthetic(self):
        """Synthetic moving-blob scene (no SDK required)."""
        t = 0.0
        while self._running:
            t += 0.07
            y_grid, x_grid = np.mgrid[0:H, 0:W].astype(np.float32)

            # Moving human-shaped blob
            bx = W / 2 + np.sin(t) * 60
            by = H / 2 + np.cos(t * 0.7) * 30
            r  = np.hypot(x_grid - bx, y_grid - by)

            # Depth: blob at 0.8 m, background at 2.0 m
            depth_m = np.where(r < 35, 0.8 + 0.05 * (r / 35), 2.0).astype(np.float32)

            # Confidence: high inside blob, lower outside
            conf = np.where(r < 35, 240.0, 80.0).astype(np.float32)
            amp  = np.clip(255.0 * (1.0 - r / np.hypot(W, H)), 10, 255).astype(np.float32)

            with self._lock:
                self._packet = FramePacket(depth_m=depth_m, amplitude=amp, confidence=conf)

            time.sleep(0.033)   # ~30 FPS synthetic


# ─────────────────────────── image processing ───────────────────────────────
class FrameProcessor:
    """All per-frame computations, designed for 240×180 input → 240×180 output."""

    def __init__(self):
        k = 3
        self._morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

    # ── depth map ───────────────────────────────────────────────────────────
    def depth_to_color(self, depth_m: np.ndarray, confidence: np.ndarray) -> np.ndarray:
        """JET colormap depth image.  Invalid/low-confidence pixels → black.
        Output: uint8 BGR 240×180, ready for imshow without any resize."""
        d = depth_m.copy()
        invalid = (d < MIN_DEPTH_M) | (d > MAX_DEPTH_M) | np.isnan(d)
        if confidence is not None:
            invalid |= (confidence < CONFIDENCE_THRESH)
        d[invalid] = 0.0

        norm = np.clip(d / MAX_DEPTH_M, 0.0, 1.0)
        u8   = (norm * 255).astype(np.uint8)
        col  = cv2.applyColorMap(u8, cv2.COLORMAP_JET)
        col[d == 0] = 0          # pure black for invalid
        return col

    # ── confidence map ──────────────────────────────────────────────────────
    def confidence_to_gray(self, confidence: np.ndarray) -> np.ndarray:
        """Arducam-style confidence map: black=low confidence, white=high confidence.
        Matches the official Arducam_tof_camera SDK example output.
        Output: uint8 BGR 240×180 (single-channel stretched to 0-255)."""
        norm = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        return cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)

    # ── human detection ─────────────────────────────────────────────────────
    def detect_humans(
        self,
        depth_m: np.ndarray,
        confidence: np.ndarray,
        depth_bgr: np.ndarray
    ) -> np.ndarray:
        """Threshold depth for human-range pixels, find largest contour,
        draw bounding box and distance label.  No HUD bars – only the
        detection annotation is overlaid on a clean copy of depth_bgr.
        Output: uint8 BGR 240×180."""
        d = depth_m.copy()

        # Mask: pixels where a human could be (valid depth + good confidence)
        invalid = (d < MIN_DEPTH_M) | (d > MAX_HUMAN_DIST_M)
        if confidence is not None:
            invalid |= (confidence < CONFIDENCE_THRESH)
        d[invalid] = 0.0

        mask = (d > 0).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph)

        canvas = depth_bgr.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Keep only blobs large enough to be a human torso in frame
        humans = [c for c in contours if cv2.contourArea(c) >= MIN_HUMAN_PX]

        for c in humans:
            x, y, bw, bh = cv2.boundingRect(c)

            # Mean depth of the blob
            obj_mask = np.zeros((H, W), np.uint8)
            cv2.drawContours(obj_mask, [c], -1, 255, cv2.FILLED)
            depths = d[(obj_mask == 255) & (d > 0)]
            if depths.size == 0:
                continue
            mean_d = float(np.mean(depths))

            # Bounding box (green)
            cv2.rectangle(canvas, (x, y), (x + bw, y + bh), (0, 255, 0), 1)

            # Distance label – small font, positioned inside the box
            label = f"{mean_d:.2f}m"
            lx = x + 2
            ly = max(y + 12, 12)
            cv2.putText(canvas, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)

        return canvas


# ─────────────────────────── main application ───────────────────────────────
class HumanDetectionApp:
    """Minimal, low-latency human detection viewer.

    Three clean windows displayed at native 240×180:
      • Depth       – JET depth map
      • Confidence  – Arducam-style black/gray confidence map
      • Detection   – JET depth with human bounding boxes
    """

    # Window positions (column, row) so they sit side-by-side at screen origin
    _WIN_POSITIONS = {
        WIN_DEPTH:  (0,   0),
        WIN_CONF:   (250, 0),
        WIN_DETECT: (500, 0),
    }

    def __init__(self):
        self._cam_thread = CameraThread()
        self._processor  = FrameProcessor()
        self._running    = False

    def start(self):
        signal.signal(signal.SIGINT, self._sigint)

        ok = self._cam_thread.start_camera()
        self._cam_thread.start()

        # Create windows at exact pixel positions, non-resizable (WINDOW_AUTOSIZE)
        for name, (px, py) in self._WIN_POSITIONS.items():
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(name, px, py)

        self._running = True
        print("[HumanDetection] Running.  Q = quit.")
        self._loop()

    def _loop(self):
        """Main render loop – grab once, derive three images, push to windows."""
        try:
            while self._running:
                pkt = self._cam_thread.get_latest()
                if pkt is not None:
                    depth_bgr = self._processor.depth_to_color(pkt.depth_m, pkt.confidence)
                    conf_bgr  = self._processor.confidence_to_gray(pkt.confidence)
                    det_bgr   = self._processor.detect_humans(pkt.depth_m, pkt.confidence, depth_bgr)

                    cv2.imshow(WIN_DEPTH,  depth_bgr)
                    cv2.imshow(WIN_CONF,   conf_bgr)
                    cv2.imshow(WIN_DETECT, det_bgr)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:   # Q or Esc
                    self._running = False
        finally:
            self._shutdown()

    def _shutdown(self):
        self._cam_thread.stop()
        self._cam_thread.join(timeout=1.5)
        cv2.destroyAllWindows()
        print("[HumanDetection] Stopped.")

    def _sigint(self, *_):
        self._running = False


def main():
    HumanDetectionApp().start()


if __name__ == "__main__":
    main()
