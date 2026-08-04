"""
app/hardware/serial_lidar.py
----------------------------
Author: SUDHARSAN
High-performance PySerial driver for YDLIDAR X2 USB LiDAR.
Provides hardware auto-detection, packet parsing, checksum validation,
polar-to-Cartesian coordinate transformation, and Qt QThread stream worker.
"""

import math
import struct
import time
from typing import List, Optional, Tuple
import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal

from app.core.data_types import ScanPoint, LidarScan


def detect_ydlidar_port() -> str:
    """Auto-detect USB serial port for YDLIDAR (Silicon Labs CP210x, FTDI, or /dev/ttyUSB*)."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()
        if "cp210" in desc or "cp210" in hwid or "usb" in port.device.lower() or "ttyusb" in port.device.lower():
            return port.device
    if ports:
        return ports[0].device
    return "/dev/ttyUSB0"


class YdLidarX2Driver:
    """YDLIDAR X2 direct serial protocol parser."""

    PH_BYTE1 = 0xAA
    PH_BYTE2 = 0x55

    def __init__(self, port: str = "AUTO", baud_rate: int = 115200, timeout: float = 1.0):
        self.requested_port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.active_port: str = ""
        self.is_connected: bool = False

        # Accumulator for 360-degree sweep
        self.current_scan_points: List[ScanPoint] = []
        self.last_scan_time: float = time.time()
        self.scan_count: int = 0
        self.total_samples: int = 0

    def connect(self) -> bool:
        """Open USB serial connection."""
        if self.is_connected and self.ser and self.ser.is_open:
            return True

        target_port = detect_ydlidar_port() if self.requested_port == "AUTO" else self.requested_port
        try:
            self.ser = serial.Serial(
                port=target_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.active_port = target_port
            self.is_connected = True
            print(f"[YdLidarDriver] Successfully connected to {self.active_port} at {self.baud_rate} baud.")
            return True
        except Exception as e:
            print(f"[YdLidarDriver] Failed to connect to {target_port}: {e}")
            self.is_connected = False
            self.ser = None
            return False

    def disconnect(self) -> None:
        """Close USB serial connection gracefully."""
        self.is_connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        print("[YdLidarDriver] Disconnected from LiDAR.")

    def read_packet(self) -> Optional[LidarScan]:
        """
        Read and parse one YDLIDAR X2 serial data frame packet.
        Returns LidarScan when a complete 360-degree rotation is accumulated.
        """
        if not self.ser or not self.ser.is_open:
            return None

        try:
            # Sync to 0xAA 0x55 header
            header = self.ser.read(2)
            if len(header) < 2:
                return None

            if header[0] != self.PH_BYTE1 or header[1] != self.PH_BYTE2:
                # Flush single byte to re-align sync
                self.ser.read(1)
                return None

            # Packet Body Header: CT (1 byte), LS (1 byte), FSA (2 bytes), LSA (2 bytes), CS (2 bytes)
            pkt_head = self.ser.read(8)
            if len(pkt_head) < 8:
                return None

            ct, ls, fsa_raw, lsa_raw, cs_raw = struct.unpack("<BBHHH", pkt_head)
            sample_count = ls

            # Read distance samples (2 bytes per sample)
            sample_bytes = self.ser.read(sample_count * 2)
            if len(sample_bytes) < sample_count * 2:
                return None

            # Checksum Verification
            computed_cs = self.PH_BYTE1 ^ (self.PH_BYTE2 << 8)
            computed_cs ^= (ct | (ls << 8))
            computed_cs ^= fsa_raw
            computed_cs ^= lsa_raw
            for i in range(0, len(sample_bytes), 2):
                val = sample_bytes[i] | (sample_bytes[i + 1] << 8)
                computed_cs ^= val

            if computed_cs != cs_raw:
                # Checksum error - drop corrupt packet
                return None

            # Process angles and distance values
            fsa_deg = (fsa_raw >> 1) / 64.0
            lsa_deg = (lsa_raw >> 1) / 64.0

            # Angle difference
            diff_deg = (lsa_deg - fsa_deg + 360.0) % 360.0
            step_deg = diff_deg / (sample_count - 1) if sample_count > 1 else 0.0

            packet_completed_scan: Optional[LidarScan] = None

            for i in range(sample_count):
                raw_dist = sample_bytes[i * 2] | (sample_bytes[i * 2 + 1] << 8)
                distance_mm = raw_dist / 4.0

                raw_angle = (fsa_deg + step_deg * i) % 360.0

                # Angle correction offset for distance optics
                if distance_mm > 0:
                    try:
                        corr_rad = math.atan(21.8 * (155.3 - distance_mm) / (155.3 * distance_mm))
                        corrected_angle = (raw_angle + math.degrees(corr_rad) + 360.0) % 360.0
                    except Exception:
                        corrected_angle = raw_angle
                else:
                    corrected_angle = raw_angle

                angle_rad = math.radians(corrected_angle)
                x_mm = distance_mm * math.cos(angle_rad)
                y_mm = distance_mm * math.sin(angle_rad)

                point = ScanPoint(
                    angle_deg=corrected_angle,
                    angle_rad=angle_rad,
                    distance_mm=distance_mm,
                    x_mm=x_mm,
                    y_mm=y_mm,
                    z_mm=0.0,
                    intensity=255.0 if distance_mm > 0 else 0.0
                )

                # Check if a 360-degree rotation wrapped around
                if self.current_scan_points and (corrected_angle < self.current_scan_points[-1].angle_deg - 180.0):
                    now = time.time()
                    dt = max(now - self.last_scan_time, 0.001)
                    scan_freq = 1.0 / dt
                    self.last_scan_time = now

                    packet_completed_scan = LidarScan(
                        timestamp=now,
                        points=list(self.current_scan_points),
                        scan_frequency_hz=scan_freq,
                        sample_rate_sps=len(self.current_scan_points) * int(scan_freq)
                    )
                    self.current_scan_points.clear()

                self.current_scan_points.append(point)

            return packet_completed_scan

        except serial.SerialException as se:
            print(f"[YdLidarDriver] USB Serial Error: {se}")
            self.disconnect()
            return None
        except Exception as e:
            print(f"[YdLidarDriver] Unexpected parsing exception: {e}")
            return None


class LidarThread(QThread):
    """Background worker thread for continuous LiDAR hardware streaming."""

    scan_ready = Signal(object)      # Emits LidarScan
    status_changed = Signal(str)     # Connection state message
    connection_failed = Signal(str) # Error notification

    def __init__(self, port: str = "AUTO", baud_rate: int = 115200):
        super().__init__()
        self.driver = YdLidarX2Driver(port=port, baud_rate=baud_rate)
        self.running: bool = False

    def run(self) -> None:
        """Main thread loop for serial polling."""
        self.running = True
        self.status_changed.emit("Connecting...")

        if not self.driver.connect():
            self.connection_failed.emit(f"Could not open serial port {self.driver.requested_port}")
            self.status_changed.emit("Disconnected")
            self.running = False
            return

        self.status_changed.emit(f"Connected ({self.driver.active_port})")

        while self.running and self.driver.is_connected:
            scan = self.driver.read_packet()
            if scan and scan.points:
                self.scan_ready.emit(scan)
            else:
                # Sleep briefly to reduce CPU spin if buffer empty
                time.sleep(0.002)

        self.driver.disconnect()
        self.status_changed.emit("Disconnected")

    def stop(self) -> None:
        """Stop background worker execution."""
        self.running = False
        self.wait(1000)
