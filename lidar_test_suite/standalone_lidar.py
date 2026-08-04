"""
standalone_lidar.py
-------------------
Author: SUDHARSAN
Standalone YDLIDAR X2 SDK wrapper and background thread processor.
Exposes thread-safe scan data, sector analytics, and historical point trail buffers.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import numpy as np

try:
    import ydlidar
    YDLIDAR_SDK_AVAILABLE = True
except ImportError:
    YDLIDAR_SDK_AVAILABLE = False

import lidar_config as config


class LidarInitError(Exception):
    """Raised when LiDAR connection or configuration fails."""
    pass


@dataclass
class LidarPoint:
    angle_rad: float
    range_m: float
    intensity: float
    x: float
    y: float


@dataclass
class ScanPacket:
    points: List[LidarPoint] = field(default_factory=list)
    scan_time: float = 0.0
    point_count: int = 0
    timestamp: float = 0.0


class StandaloneLidar:
    def __init__(self, port: str = config.LIDAR_PORT):
        self.port = port
        self._laser = None
        self._is_running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self.latest_scan = ScanPacket()
        self.history_scans: List[ScanPacket] = []
        self.max_history_len = 5
        self.connected = False
        self.scan_rate_hz = 0.0

        self._scan_count = 0
        self._last_rate_calc_time = time.time()

    def initialize(self):
        """Sets up SDK parameters and connects to the USB LiDAR device."""
        if not YDLIDAR_SDK_AVAILABLE:
            raise LidarInitError(
                "ydlidar module not found. Build and install YDLidar-SDK per README."
            )

        ydlidar.os_init()
        self._laser = ydlidar.CYdLidar()

        self._laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, config.LIDAR_BAUDRATE)
        self._laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self._laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self._laser.setlidaropt(ydlidar.LidarPropScanFrequency, config.LIDAR_FREQUENCY)
        self._laser.setlidaropt(ydlidar.LidarPropSampleRate, config.LIDAR_SAMPLE_RATE)
        self._laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self._laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMaxRange, config.LIDAR_MAX_RANGE_M)
        self._laser.setlidaropt(ydlidar.LidarPropMinRange, config.LIDAR_MIN_RANGE_M)
        self._laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        ok = self._laser.initialize()
        if not ok:
            raise LidarInitError(f"Failed to initialize YDLIDAR on {self.port}.")

        ok = self._laser.turnOn()
        if not ok:
            raise LidarInitError("Failed to turn on YDLIDAR motor scan.")

        self.connected = True
        print(f"[StandaloneLidar] Connected to YDLIDAR X2 on {self.port}")

    def start_background_scan(self):
        """Starts background scan daemon thread."""
        if not self.connected:
            raise LidarInitError("Call initialize() before starting background thread.")

        self._is_running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        scan_msg = ydlidar.LaserScan()
        while self._is_running and ydlidar.os_isOk():
            try:
                result = self._laser.doProcessSimple(scan_msg)
                if result:
                    raw_pts = scan_msg.points
                    parsed_points: List[LidarPoint] = []

                    for p in raw_pts:
                        angle = p.angle
                        # Auto-detect degrees vs radians
                        if abs(angle) > 7.0:
                            angle = np.radians(angle)

                        r = p.range
                        if config.LIDAR_MIN_RANGE_M <= r <= config.LIDAR_MAX_RANGE_M:
                            x = r * np.sin(angle)
                            y = r * np.cos(angle)
                            parsed_points.append(LidarPoint(
                                angle_rad=angle,
                                range_m=r,
                                intensity=p.intensity,
                                x=x,
                                y=y
                            ))

                    packet = ScanPacket(
                        points=parsed_points,
                        scan_time=scan_msg.config.scan_time,
                        point_count=len(parsed_points),
                        timestamp=time.time()
                    )

                    with self._lock:
                        self.latest_scan = packet
                        self.history_scans.append(packet)
                        if len(self.history_scans) > self.max_history_len:
                            self.history_scans.pop(0)

                    self._scan_count += 1
                    now = time.time()
                    elapsed = now - self._last_rate_calc_time
                    if elapsed >= 1.0:
                        self.scan_rate_hz = self._scan_count / elapsed
                        self._scan_count = 0
                        self._last_rate_calc_time = now
                else:
                    time.sleep(0.01)
            except Exception as exc:
                time.sleep(0.02)

    def get_sector_ranges(self) -> Dict[str, float]:
        """Calculates minimum obstacle distance for 6 directional sectors."""
        sectors = {
            "Front": 99.0,
            "Front-Left": 99.0,
            "Front-Right": 99.0,
            "Left": 99.0,
            "Right": 99.0,
            "Rear": 99.0,
        }

        with self._lock:
            pts = self.latest_scan.points

        for p in pts:
            deg = np.degrees(p.angle_rad)
            r = p.range_m
            if -30 <= deg <= 30:
                sectors["Front"] = min(sectors["Front"], r)
            elif 30 < deg <= 75:
                sectors["Front-Right"] = min(sectors["Front-Right"], r)
            elif -75 <= deg < -30:
                sectors["Front-Left"] = min(sectors["Front-Left"], r)
            elif 75 < deg <= 135:
                sectors["Right"] = min(sectors["Right"], r)
            elif -135 <= deg < -75:
                sectors["Left"] = min(sectors["Left"], r)
            else:
                sectors["Rear"] = min(sectors["Rear"], r)

        return sectors

    def stop(self):
        """Clean shutdown of motor and scanning thread."""
        self._is_running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        if self._laser is not None:
            try:
                self._laser.turnOff()
                self._laser.disconnecting()
            except Exception as exc:
                print(f"[StandaloneLidar] Shutdown notice: {exc}")

        self.connected = False
        print("[StandaloneLidar] LiDAR motor powered down.")
