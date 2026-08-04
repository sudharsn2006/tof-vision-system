"""
app/hardware/mock_lidar.py
--------------------------
Author: SUDHARSAN
Realistic fallback mock LiDAR generator for testing without physical hardware.
Simulates a 360-degree rotational scan stream (720 points at 8 Hz) with room boundaries,
static obstacles, and dynamic moving targets entering the 0.5m collision zone.
"""

import math
import time
import numpy as np
from PySide6.QtCore import QThread, Signal

from app.core.data_types import ScanPoint, LidarScan


class MockLidarThread(QThread):
    """Background worker thread simulating a continuous 360-degree LiDAR scan."""

    scan_ready = Signal(object)  # Emits LidarScan
    status_changed = Signal(str)

    def __init__(self, target_freq_hz: float = 8.0, num_samples: int = 720):
        super().__init__()
        self.target_freq_hz = target_freq_hz
        self.num_samples = num_samples
        self.running: bool = False
        self.start_time: float = time.time()

    def run(self) -> None:
        """Main simulation render loop."""
        self.running = True
        self.start_time = time.time()
        self.status_changed.emit("Connected (Mock LiDAR)")

        last_time = time.time()

        while self.running:
            now = time.time()
            dt = max(now - last_time, 0.001)
            last_time = now

            t = now - self.start_time
            scan_points: list[ScanPoint] = []

            # Dynamic moving obstacle (oscillates along Y axis into 0.5m collision zone)
            moving_ox = 600.0 * math.cos(t * 0.8)
            moving_oy = 400.0 + 800.0 * math.sin(t * 1.2)  # Moves between -400mm and +1200mm

            # Static obstacles
            static_obs = [
                (-1200.0, 1500.0, 300.0), # (x, y, radius)
                (1800.0, -800.0, 250.0),
                (-1500.0, -1500.0, 400.0),
            ]

            # Room dimensions (5m x 4m rectangle bounds)
            room_x_min, room_x_max = -2500.0, 2500.0
            room_y_min, room_y_max = -2000.0, 2000.0

            for i in range(self.num_samples):
                angle_deg = (360.0 / self.num_samples) * i
                angle_rad = math.radians(angle_deg)
                cos_a = math.cos(angle_rad)
                sin_a = math.sin(angle_rad)

                # Raycast against room boundary walls
                d_wall = 10000.0
                if cos_a > 1e-4:
                    d_wall = min(d_wall, (room_x_max) / cos_a)
                elif cos_a < -1e-4:
                    d_wall = min(d_wall, (room_x_min) / cos_a)

                if sin_a > 1e-4:
                    d_wall = min(d_wall, (room_y_max) / sin_a)
                elif sin_a < -1e-4:
                    d_wall = min(d_wall, (room_y_min) / sin_a)

                d_final = d_wall

                # Raycast against static obstacles
                for ox, oy, r in static_obs:
                    # Distance from origin ray to sphere center
                    proj = ox * cos_a + oy * sin_a
                    if proj > 0:
                        perp_sq = (ox**2 + oy**2) - proj**2
                        if perp_sq < r**2:
                            d_hit = proj - math.sqrt(r**2 - perp_sq)
                            if 0 < d_hit < d_final:
                                d_final = d_hit

                # Raycast against dynamic moving obstacle
                proj_m = moving_ox * cos_a + moving_oy * sin_a
                if proj_m > 0:
                    perp_sq_m = (moving_ox**2 + moving_oy**2) - proj_m**2
                    r_m = 220.0
                    if perp_sq_m < r_m**2:
                        d_hit_m = proj_m - math.sqrt(r_m**2 - perp_sq_m)
                        if 0 < d_hit_m < d_final:
                            d_final = d_hit_m

                # Add small Gaussian noise to distance
                noise = np.random.normal(0, 8.0)
                distance_mm = max(d_final + noise, 50.0)

                x_mm = distance_mm * cos_a
                y_mm = distance_mm * sin_a

                scan_points.append(ScanPoint(
                    angle_deg=angle_deg,
                    angle_rad=angle_rad,
                    distance_mm=distance_mm,
                    x_mm=x_mm,
                    y_mm=y_mm,
                    z_mm=0.0,
                    intensity=200.0 + np.random.uniform(-30, 55)
                ))

            scan = LidarScan(
                timestamp=now,
                points=scan_points,
                scan_frequency_hz=self.target_freq_hz,
                sample_rate_sps=int(self.num_samples * self.target_freq_hz)
            )
            self.scan_ready.emit(scan)

            # Sleep to match target frame frequency
            time.sleep(1.0 / self.target_freq_hz)

        self.status_changed.emit("Disconnected")

    def stop(self) -> None:
        """Stop background worker execution."""
        self.running = False
        self.wait(1000)
