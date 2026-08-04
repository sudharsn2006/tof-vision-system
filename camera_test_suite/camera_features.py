"""
camera_features.py
------------------
Author: SUDHARSAN
Core image processing, 3D pinhole spatial projection, contour object detection,
and diagnostic algorithms for the Arducam B0410 ToF Camera.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
import numpy as np
import cv2

import camera_config as config


@dataclass
class DetectedCameraObject:
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    area_px: int
    avg_depth_m: float
    min_depth_m: float
    max_depth_m: float
    contour: np.ndarray


class CameraFeatureProcessor:
    def __init__(self):
        k = config.MORPH_KERNEL_SIZE
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

    def process_amplitude_stream(self, amplitude: np.ndarray) -> np.ndarray:
        """Converts 940nm VCSEL IR intensity to an AGC contrast-enhanced grayscale video frame."""
        amp_norm = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        amp_enhanced = clahe.apply(amp_norm)
        return cv2.cvtColor(amp_enhanced, cv2.COLOR_GRAY2BGR)

    def process_depth_stream(
        self,
        depth_m: np.ndarray,
        confidence: np.ndarray,
        use_colormap: bool = True,
        probe_pt: Optional[Tuple[int, int]] = None
    ) -> Tuple[np.ndarray, Optional[float]]:
        """Processes raw depth map, applies noise filtering, colormap, and optional depth probe."""
        depth = depth_m.copy()

        min_m = config.MIN_VALID_DEPTH_MM / 1000.0
        max_m = (config.MAX_RANGE_FAR_MM if config.RANGE_MODE == "FAR" else config.MAX_RANGE_NEAR_MM) / 1000.0

        invalid = (depth < min_m) | (depth > max_m) | np.isnan(depth)
        if confidence is not None:
            invalid = invalid | (confidence < config.CONFIDENCE_THRESHOLD)

        depth[invalid] = 0.0
        depth_clean = cv2.medianBlur(depth, config.MEDIAN_BLUR_KERNEL)

        probe_dist = None
        if probe_pt is not None:
            px, py = probe_pt
            h, w = depth_clean.shape[:2]
            if 0 <= px < w and 0 <= py < h:
                probe_dist = float(depth_clean[py, px])

        norm = np.clip(depth_clean / max_m, 0.0, 1.0)
        if use_colormap:
            depth_8u = (norm * 255.0).astype(np.uint8)
            img = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
        else:
            depth_8u = ((1.0 - norm) * 255.0).astype(np.uint8)
            img = cv2.cvtColor(depth_8u, cv2.COLOR_GRAY2BGR)

        img[depth_clean == 0] = (0, 0, 0)
        return img, probe_dist

    def detect_objects(
        self,
        depth_m: np.ndarray,
        max_dist_mm: int = config.DETECTION_DISTANCE_MM
    ) -> Tuple[np.ndarray, Optional[DetectedCameraObject]]:
        """Contour-based object detection using depth thresholding."""
        min_m = config.MIN_VALID_DEPTH_MM / 1000.0
        max_m = max_dist_mm / 1000.0

        mask = ((depth_m >= min_m) & (depth_m <= max_m)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_c = [c for c in contours if cv2.contourArea(c) >= config.MIN_CONTOUR_AREA_PX]

        if not valid_c:
            return mask, None

        largest = max(valid_c, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        area = int(cv2.contourArea(largest))

        obj_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(obj_mask, [largest], -1, 255, thickness=cv2.FILLED)

        obj_depths = depth_m[(obj_mask == 255) & (depth_m > 0)]
        if obj_depths.size == 0:
            return mask, None

        avg_d = float(np.mean(obj_depths))
        min_d = float(np.min(obj_depths))
        max_d = float(np.max(obj_depths))

        M = cv2.moments(largest)
        cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
        cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2

        detected = DetectedCameraObject(
            bbox=(x, y, w, h),
            center=(cx, cy),
            area_px=area,
            avg_depth_m=avg_d,
            min_depth_m=min_d,
            max_depth_m=max_d,
            contour=largest
        )
        return mask, detected

    @staticmethod
    def project_3d_point_cloud(
        depth_m: np.ndarray,
        yaw_deg: float = 0.0,
        pitch_deg: float = 35.0,
        step: int = 4
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Projects 240x180 depth map into 3D spatial coordinates (X, Y, Z)
        using 70 deg FOV pinhole camera geometry.
        """
        dh, dw = depth_m.shape[:2]

        # 70 deg diagonal FOV camera intrinsics
        fov_rad = np.radians(config.FOV_DIAGONAL_DEG)
        focal_px = (np.hypot(dw, dh) / 2.0) / np.tan(fov_rad / 2.0)
        cx, cy = dw / 2.0, dh / 2.0

        v_coords, u_coords = np.mgrid[0:dh:step, 0:dw:step]
        depth_samples = depth_m[::step, ::step]

        valid = (depth_samples > 0.1) & (depth_samples <= 4.0)
        z_spatial = depth_samples[valid]

        if z_spatial.size == 0:
            return np.array([]), np.array([]), np.array([])

        u_samp = u_coords[valid]
        v_samp = v_coords[valid]

        # Pinhole 3D back-projection: X = (u - cx)*Z / f, Y = (v - cy)*Z / f
        x_spatial = (u_samp - cx) * z_spatial / focal_px
        y_spatial = -(v_samp - cy) * z_spatial / focal_px

        # Perspective 3D rotation
        pitch_rad = np.radians(pitch_deg)
        yaw_rad = np.radians(yaw_deg)

        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)

        # Yaw rotation (around Z)
        x_rot = x_spatial * cos_y - z_spatial * sin_y
        z_rot_temp = x_spatial * sin_y + z_spatial * cos_y

        # Pitch rotation (around X)
        y_rot = y_spatial * cos_p + z_rot_temp * sin_p
        z_rot = -y_spatial * sin_p + z_rot_temp * cos_p

        return x_rot, y_rot, z_rot

    def generate_confidence_diagnostics(
        self,
        confidence: np.ndarray,
        depth_m: np.ndarray
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """Generates diagnostics confidence heatmap and SNR metric analytics."""
        conf_norm = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        conf_heatmap = cv2.applyColorMap(conf_norm, cv2.COLORMAP_VIRIDIS)

        valid_mask = confidence >= config.CONFIDENCE_THRESHOLD
        valid_ratio = float(np.mean(valid_mask)) * 100.0
        avg_conf = float(np.mean(confidence))
        noise_ratio = float(100.0 - valid_ratio)

        stats = {
            "valid_pixel_pct": round(valid_ratio, 1),
            "avg_confidence": round(avg_conf, 1),
            "noise_pct": round(noise_ratio, 1)
        }
        return conf_heatmap, stats
