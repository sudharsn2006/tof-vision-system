"""
obstacle_analyzer.py
--------------------
Author: SUDHARSAN
Clustering, obstacle extraction, and safety zone proximity analytics.
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

import lidar_config as config
from standalone_lidar import LidarPoint


@dataclass
class ObstacleCluster:
    cluster_id: int
    points: List[Tuple[float, float]]  # (x, y) in meters
    center_x: float
    center_y: float
    distance_m: float
    angle_deg: float
    width_m: float
    point_count: int


class ObstacleAnalyzer:
    def __init__(
        self,
        eps_m: float = config.CLUSTER_EPSILON_M,
        min_points: int = config.MIN_CLUSTER_POINTS
    ):
        self.eps_m = eps_m
        self.min_points = min_points

    def extract_clusters(self, points: List[LidarPoint]) -> List[ObstacleCluster]:
        """Ultra-fast O(N) spatial grid clustering for 60+ FPS performance."""
        if not points:
            return []

        coords = np.array([[p.x, p.y] for p in points])
        if len(coords) < self.min_points:
            return []

        # Quantize points into spatial grid cells of size eps_m
        grid_keys = np.floor(coords / self.eps_m).astype(int)

        cell_map = {}
        for idx, key in enumerate(grid_keys):
            cell_tuple = (key[0], key[1])
            if cell_tuple not in cell_map:
                cell_map[cell_tuple] = []
            cell_map[cell_tuple].append(idx)

        visited_cells = set()
        clusters: List[ObstacleCluster] = []
        cluster_id = 1

        for cell_key, indices in cell_map.items():
            if cell_key in visited_cells:
                continue

            # Gather adjacent grid cells (3x3 neighborhood)
            group_indices = []
            queue = [cell_key]
            visited_cells.add(cell_key)

            while queue:
                curr_cell = queue.pop(0)
                if curr_cell in cell_map:
                    group_indices.extend(cell_map[curr_cell])
                    # Check 8 adjacent cells
                    cx, cy = curr_cell
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            adj = (cx + dx, cy + dy)
                            if adj not in visited_cells and adj in cell_map:
                                visited_cells.add(adj)
                                queue.append(adj)

            if len(group_indices) >= self.min_points:
                c_coords = coords[group_indices]
                cx = float(np.mean(c_coords[:, 0]))
                cy = float(np.mean(c_coords[:, 1]))
                dist = float(np.hypot(cx, cy))
                angle = float(np.degrees(np.arctan2(cx, cy)))

                # Approximate width using min/max bounding range
                min_pt = np.min(c_coords, axis=0)
                max_pt = np.max(c_coords, axis=0)
                width = float(np.hypot(max_pt[0] - min_pt[0], max_pt[1] - min_pt[1]))

                clusters.append(ObstacleCluster(
                    cluster_id=cluster_id,
                    points=[(pt[0], pt[1]) for pt in c_coords],
                    center_x=cx,
                    center_y=cy,
                    distance_m=dist,
                    angle_deg=angle,
                    width_m=width,
                    point_count=len(c_coords)
                ))
                cluster_id += 1

        clusters.sort(key=lambda c: c.distance_m)
        return clusters

    @staticmethod
    def get_safety_status(min_dist_m: float) -> Tuple[str, Tuple[int, int, int]]:
        """Determines proximity warning state and UI color code."""
        if min_dist_m < config.SAFETY_CRITICAL_M:
            return "EMERGENCY: STOP!", (0, 0, 255)  # Red
        elif min_dist_m < config.SAFETY_WARNING_M:
            return "WARNING: CAUTION", (0, 255, 255)  # Yellow
        else:
            return "STATUS: CLEAR", (0, 255, 0)  # Green
