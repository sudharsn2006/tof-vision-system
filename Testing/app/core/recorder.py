"""
app/core/recorder.py
--------------------
Author: SUDHARSAN
Handles asynchronous CSV recording of 3D LiDAR scan frames and PNG screenshot persistence.
"""

import csv
import os
import time
from typing import Optional, List
import cv2
import numpy as np

from app.core.data_types import LidarScan


class DataRecorder:
    """CSV data recording manager for scan points."""

    def __init__(self, records_dir: str = "records"):
        self.records_dir = records_dir
        self.is_recording: bool = False
        self._csv_file = None
        self._csv_writer = None
        self.active_filepath: Optional[str] = None
        self.recorded_frames_count: int = 0

        os.makedirs(self.records_dir, exist_ok=True)

    def start_recording(self) -> str:
        """Initialize a new CSV file for recording."""
        if self.is_recording:
            return self.active_filepath or ""

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"lidar_scan_{timestamp_str}.csv"
        self.active_filepath = os.path.join(self.records_dir, filename)

        self._csv_file = open(self.active_filepath, mode="w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        # Write CSV header
        self._csv_writer.writerow(["Timestamp", "Angle_deg", "Distance_mm", "X_mm", "Y_mm", "Z_mm", "Intensity"])

        self.is_recording = True
        self.recorded_frames_count = 0
        print(f"[Recorder] Started CSV recording to {self.active_filepath}")
        return self.active_filepath

    def record_scan(self, scan: LidarScan) -> None:
        """Write a complete scan frame to the active CSV file."""
        if not self.is_recording or not self._csv_writer or not self._csv_file:
            return

        try:
            for pt in scan.points:
                self._csv_writer.writerow([
                    f"{scan.timestamp:.4f}",
                    f"{pt.angle_deg:.2f}",
                    f"{pt.distance_mm:.1f}",
                    f"{pt.x_mm:.1f}",
                    f"{pt.y_mm:.1f}",
                    f"{pt.z_mm:.1f}",
                    f"{pt.intensity:.0f}"
                ])
            self._csv_file.flush()
            self.recorded_frames_count += 1
        except Exception as e:
            print(f"[Recorder] Error writing CSV frame: {e}")

    def stop_recording(self) -> Optional[str]:
        """Close active CSV file and stop recording."""
        if not self.is_recording:
            return None

        filepath = self.active_filepath
        self.is_recording = False

        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception as e:
                print(f"[Recorder] Error closing CSV file: {e}")
            self._csv_file = None
            self._csv_writer = None

        print(f"[Recorder] Stopped recording. Saved {self.recorded_frames_count} frames to {filepath}")
        return filepath


class ScreenshotManager:
    """Saves OpenGL framebuffer / QImage content to PNG image file."""

    def __init__(self, captures_dir: str = "captures"):
        self.captures_dir = captures_dir
        os.makedirs(self.captures_dir, exist_ok=True)

    def save_image(self, image_np_rgb: np.ndarray) -> str:
        """
        Save an RGB numpy array frame to disk as PNG using OpenCV.
        """
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = f"lidar_viewport_{timestamp_str}.png"
        filepath = os.path.join(self.captures_dir, filename)

        # Convert RGB to BGR for OpenCV
        bgr = cv2.cvtColor(image_np_rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(filepath, bgr)
        print(f"[ScreenshotManager] Saved screenshot to {filepath}")
        return filepath
