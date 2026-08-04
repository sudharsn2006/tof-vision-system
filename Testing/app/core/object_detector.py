"""
app/core/object_detector.py
----------------------------
Author: SUDHARSAN
Real-time 2D/3D Euclidean spatial clustering and collision detection for LiDAR point clouds.
Converts polar scan samples to Cartesian space, clusters adjacent obstacles, and computes metrics.
"""

import math
from typing import List, Tuple, Optional
import numpy as np

from app.core.data_types import ScanPoint, LidarScan, DetectedObject


class ObjectDetector:
    """Euclidean spatial clustering and hazard detection engine."""

    def __init__(self, cluster_eps_mm: float = 200.0, min_cluster_size: int = 3, collision_radius_mm: float = 500.0):
        self.cluster_eps_mm = cluster_eps_mm
        self.min_cluster_size = min_cluster_size
        self.collision_radius_mm = collision_radius_mm

    def process_scan(self, scan: LidarScan) -> Tuple[List[DetectedObject], Optional[DetectedObject], bool]:
        """
        Process a complete scan, classify points into collision zones, and extract cluster objects.

        Returns:
            - List of DetectedObject instances
            - Nearest DetectedObject (or None)
            - Global collision warning flag (True if any object/point enters collision radius)
        """
        if not scan.points:
            return [], None, False

        has_collision = False

        # Classify collision flags on individual points
        for pt in scan.points:
            if 0.0 < pt.distance_mm <= self.collision_radius_mm:
                pt.is_collision = True
                has_collision = True
            elif self.collision_radius_mm < pt.distance_mm <= self.collision_radius_mm * 2.5:
                pt.is_warning = True

        # Extract non-zero Cartesian points for clustering
        valid_pts = [p for p in scan.points if p.distance_mm > 0.0]
        if not valid_pts:
            return [], None, False

        coords = np.array([[p.x_mm, p.y_mm] for p in valid_pts], dtype=np.float32)

        # Simple & fast Euclidean distance neighbor-group clustering
        num_points = len(valid_pts)
        visited = [False] * num_points
        clusters: List[List[int]] = []

        for i in range(num_points):
            if visited[i]:
                continue
            visited[i] = True

            # Find neighbors within eps
            diffs = coords - coords[i]
            dists_sq = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
            neighbors = np.where(dists_sq <= self.cluster_eps_mm ** 2)[0].tolist()

            if len(neighbors) < self.min_cluster_size:
                continue

            current_cluster = []
            queue = neighbors
            for idx in queue:
                if not visited[idx]:
                    visited[idx] = True
                    # Find sub-neighbors
                    sub_diffs = coords - coords[idx]
                    sub_dists_sq = sub_diffs[:, 0] ** 2 + sub_diffs[:, 1] ** 2
                    sub_neighbors = np.where(sub_dists_sq <= self.cluster_eps_mm ** 2)[0].tolist()
                    if len(sub_neighbors) >= self.min_cluster_size:
                        queue.extend([n for n in sub_neighbors if n not in queue])
                current_cluster.append(idx)

            if len(current_cluster) >= self.min_cluster_size:
                clusters.append(current_cluster)

        # Build DetectedObject models
        detected_objects: List[DetectedObject] = []
        for obj_id, cluster_indices in enumerate(clusters, start=1):
            cluster_pts = [valid_pts[idx] for idx in cluster_indices]
            cx = float(np.mean([p.x_mm for p in cluster_pts]))
            cy = float(np.mean([p.y_mm for p in cluster_pts]))

            dist = math.hypot(cx, cy)
            angle_rad = math.atan2(cy, cx)
            angle_deg = (math.degrees(angle_rad) + 360.0) % 360.0

            # Bounding radius around centroid
            max_r = max(math.hypot(p.x_mm - cx, p.y_mm - cy) for p in cluster_pts)
            bounding_radius = max(max_r, 50.0)

            is_obj_collision = any(p.is_collision for p in cluster_pts) or (dist <= self.collision_radius_mm)
            if is_obj_collision:
                has_collision = True

            obj = DetectedObject(
                object_id=obj_id,
                centroid_x_mm=cx,
                centroid_y_mm=cy,
                distance_mm=dist,
                angle_deg=angle_deg,
                bounding_radius_mm=bounding_radius,
                point_count=len(cluster_pts),
                is_collision=is_obj_collision,
                points=cluster_pts
            )
            detected_objects.append(obj)

        # Identify nearest object
        nearest_obj = min(detected_objects, key=lambda o: o.distance_mm) if detected_objects else None

        return detected_objects, nearest_obj, has_collision
