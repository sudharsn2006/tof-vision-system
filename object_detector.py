"""
object_detector.py
--------------------
Author: SUDHARSAN
Pure geometric / statistical object detection from a binary foreground mask
plus the underlying cleaned depth map. No AI models are used.

For each frame, finds the largest valid contour (the closest / most
prominent object) and reports:
    - bounding rectangle
    - center point
    - average / min / max depth (meters)
    - pixel area
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import cv2

import config


@dataclass
class DetectedObject:
    bbox: Tuple[int, int, int, int]   # x, y, w, h
    center: Tuple[int, int]           # cx, cy
    area_px: int
    avg_distance_m: float
    min_distance_m: float
    max_distance_m: float
    contour: np.ndarray


class ObjectDetector:
    def __init__(self, min_contour_area_px: int = config.MIN_CONTOUR_AREA_PX):
        self.min_contour_area_px = min_contour_area_px

    def detect(self, mask: np.ndarray, depth_clean: np.ndarray) -> Optional[DetectedObject]:
        """
        Runs connected-component / contour analysis on the binary mask and
        returns the largest qualifying object, or None if no object is
        present.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Ignore tiny contours (sensor speckle, dust, reflections).
        valid_contours = [c for c in contours if cv2.contourArea(c) >= self.min_contour_area_px]
        if not valid_contours:
            return None

        # Choose the largest contour by pixel area as "the" detected object.
        largest = max(valid_contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(largest)
        area_px = int(cv2.contourArea(largest))

        # Build a mask restricted to this contour to gather depth statistics
        # only from pixels that belong to the object (not the whole bbox,
        # which may include background).
        obj_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(obj_mask, [largest], -1, 255, thickness=cv2.FILLED)

        object_depths = depth_clean[(obj_mask == 255) & (depth_clean > 0)]
        if object_depths.size == 0:
            return None

        avg_dist = float(np.mean(object_depths))
        min_dist = float(np.min(object_depths))
        max_dist = float(np.max(object_depths))

        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = x + w // 2, y + h // 2

        return DetectedObject(
            bbox=(x, y, w, h),
            center=(cx, cy),
            area_px=area_px,
            avg_distance_m=avg_dist,
            min_distance_m=min_dist,
            max_distance_m=max_dist,
            contour=largest,
        )
