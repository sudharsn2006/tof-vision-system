"""
lidar_manager.py
-----------------
Author: SUDHARSAN
Wraps the official YDLIDAR SDK Python bindings for the YDLIDAR X2 (USB).

This module only INITIALIZES the LiDAR and keeps it scanning in a
background thread. Per the project spec, LiDAR data is NOT used for
object detection in this version -- it's kept modular so a future
sensor-fusion module can consume self.latest_scan directly.

INSTALLATION:
  Build/install the YDLIDAR SDK + Python bindings per YDLIDAR's official
  instructions (github.com/YDLIDAR/YDLidar-SDK):

      git clone https://github.com/YDLIDAR/YDLidar-SDK.git
      cd YDLidar-SDK/build
      cmake ..
      make
      sudo make install
      cd ../python
      python3 setup.py install

  This exposes an `ydlidar` module with a `CYdLidar` class.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

try:
    import ydlidar
    YDLIDAR_SDK_AVAILABLE = True
except ImportError:
    YDLIDAR_SDK_AVAILABLE = False

import config


class LidarInitError(Exception):
    """Raised when the YDLIDAR X2 cannot be opened or configured."""
    pass


@dataclass
class LidarScan:
    points: List[Tuple[float, float, float]] = field(default_factory=list)  # (angle_rad, range_m, intensity)
    scan_time: float = 0.0
    point_count: int = 0


class LiDARManager:
    """
    Manages the YDLIDAR X2 lifecycle and runs scanning on a background
    thread so it never blocks the main ToF processing loop.
    """

    def __init__(self, port: str = config.LIDAR_PORT):
        self.port = port
        self._laser = None
        self._is_running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self.latest_scan = LidarScan()
        self.connected = False
        self.scan_rate_hz = 0.0

        self._scan_count = 0
        self._last_rate_calc_time = time.time()

    def initialize(self):
        """Sets up SDK parameters and opens the serial connection to the LiDAR."""
        if not YDLIDAR_SDK_AVAILABLE:
            raise LidarInitError(
                "ydlidar SDK/python bindings not found. Install the official "
                "YDLIDAR-SDK per the instructions in lidar_manager.py."
            )

        ydlidar.os_init()
        self._laser = ydlidar.CYdLidar()

        self._laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, config.LIDAR_BAUDRATE)
        self._laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self._laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self._laser.setlidaropt(ydlidar.LidarPropScanFrequency, config.LIDAR_FREQUENCY)
        self._laser.setlidaropt(ydlidar.LidarPropSampleRate, config.LIDAR_SAMPLE_RATE)
        self._laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)  # X2 is single-channel
        self._laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self._laser.setlidaropt(ydlidar.LidarPropMaxRange, config.LIDAR_MAX_RANGE_M)
        self._laser.setlidaropt(ydlidar.LidarPropMinRange, config.LIDAR_MIN_RANGE_M)
        self._laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        ok = self._laser.initialize()
        if not ok:
            raise LidarInitError("YDLIDAR X2 failed to initialize (check USB port/permissions).")

        ok = self._laser.turnOn()
        if not ok:
            raise LidarInitError("YDLIDAR X2 failed to start scanning.")

        self.connected = True
        print("[LiDARManager] LiDAR Connected")
        print("[LiDARManager] Scanning Started")

    def start_background_scan(self):
        """Starts the scan-collection loop on a daemon thread."""
        if not self.connected:
            raise LidarInitError("Cannot start scanning before initialize() succeeds.")

        self._is_running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        scan_msg = ydlidar.LaserScan()
        while self._is_running and ydlidar.os_isOk():
            try:
                result = self._laser.doProcessSimple(scan_msg)
                if result:
                    points = [(p.angle, p.range, p.intensity) for p in scan_msg.points]

                    with self._lock:
                        self.latest_scan = LidarScan(
                            points=points,
                            scan_time=scan_msg.config.scan_time,
                            point_count=len(points),
                        )

                    self._scan_count += 1
                    now = time.time()
                    elapsed = now - self._last_rate_calc_time
                    if elapsed >= 1.0:
                        self.scan_rate_hz = self._scan_count / elapsed
                        self._scan_count = 0
                        self._last_rate_calc_time = now
                else:
                    # A single failed scan isn't fatal; keep trying.
                    time.sleep(0.05)

            except Exception as exc:
                print(f"[LiDARManager] Scan error: {exc}")
                time.sleep(0.1)

    def get_status_snapshot(self):
        """Thread-safe read of the most recent scan stats for display/logging."""
        with self._lock:
            return {
                "connected": self.connected,
                "scan_rate_hz": round(self.scan_rate_hz, 2),
                "point_count": self.latest_scan.point_count,
            }

    def stop(self):
        """Stops the background thread and powers down the LiDAR cleanly."""
        self._is_running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        if self._laser is not None:
            try:
                self._laser.turnOff()
                self._laser.disconnecting()
            except Exception as exc:
                print(f"[LiDARManager] Warning while stopping LiDAR: {exc}")

        self.connected = False
        print("[LiDARManager] LiDAR stopped.")
