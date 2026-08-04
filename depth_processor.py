"""
depth_processor.py
-------------------
Author: SUDHARSAN
Cleans a raw depth-in-meters frame and produces a binary foreground mask
suitable for contour-based object detection.

All operations here are classic image processing (no AI models):
  1. Drop invalid / low-confidence pixels
  2. Median blur to remove salt-and-pepper depth noise
  3. Distance threshold to isolate "near" objects
  4. Morphological open + close to remove speckle and fill small holes
"""

import numpy as np
import cv2

import config


class DepthProcessor:
    def __init__(self):
        k = config.MORPH_KERNEL_SIZE
        self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

    def clean(self, depth_m: np.ndarray, confidence: np.ndarray) -> np.ndarray:
        if getattr(config, "CAMERA_MODE", "DEPTH") == "NORMAL":
            return cv2.GaussianBlur(depth_m, (3, 3), 0)

        depth = depth_m.copy()
        min_valid_m = config.TOF_MIN_VALID_MM / 1000.0
        max_valid_m = config.TOF_MAX_RANGE_MM / 1000.0
        invalid_mask = (depth < min_valid_m) | (depth > max_valid_m) | np.isnan(depth)

        if confidence is not None:
            low_conf_mask = confidence < config.TOF_INVALID_CONFIDENCE_THRESH
            invalid_mask = invalid_mask | low_conf_mask

        depth[invalid_mask] = 0.0
        return cv2.medianBlur(depth, config.MEDIAN_BLUR_KERNEL)

    def threshold(self, depth_clean: np.ndarray, detection_distance_mm: int) -> np.ndarray:
        if getattr(config, "CAMERA_MODE", "DEPTH") == "NORMAL":
            mask = (depth_clean > 0.15).astype(np.uint8) * 255
            return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_kernel)

        min_valid_m = config.TOF_MIN_VALID_MM / 1000.0
        max_detect_m = detection_distance_mm / 1000.0

        mask = np.zeros(depth_clean.shape, dtype=np.uint8)
        candidate = (depth_clean > min_valid_m) & (depth_clean <= max_detect_m)
        mask[candidate] = 255

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self._morph_kernel,
            iterations=config.MORPH_OPEN_ITERATIONS
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self._morph_kernel,
            iterations=config.MORPH_CLOSE_ITERATIONS
        )

        return mask

    @staticmethod
    def to_display_image(depth_clean: np.ndarray, max_range_mm: int, grayscale: bool = config.USE_GRAYSCALE_DEPTH) -> np.ndarray:
        if getattr(config, "CAMERA_MODE", "DEPTH") == "NORMAL":
            # Direct normal camera video feed (0 - 255 grayscale)
            depth_8u = (np.clip(depth_clean, 0.0, 1.0) * 255.0).astype(np.uint8)
            return cv2.cvtColor(depth_8u, cv2.COLOR_GRAY2BGR)

        max_range_m = max_range_mm / 1000.0
        normalized = np.clip(depth_clean / max_range_m, 0.0, 1.0)

        if grayscale:
            depth_8u = ((1.0 - normalized) * 255).astype(np.uint8)
            img = cv2.cvtColor(depth_8u, cv2.COLOR_GRAY2BGR)
            img[depth_clean == 0] = (0, 0, 0)
            return img

        depth_8u = (normalized * 255).astype(np.uint8)
        color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
        color[depth_clean == 0] = (0, 0, 0)
        return color

    @staticmethod
    def to_colormap(depth_clean: np.ndarray, max_range_mm: int) -> np.ndarray:
        return DepthProcessor.to_display_image(depth_clean, max_range_mm, grayscale=False)
