"""
app/core/data_types.py
----------------------
Data structures for 3D LiDAR measurements, object clusters, and hardware telemetry.
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class ScanPoint:
    """Individual 2D/3D LiDAR measurement point."""
    angle_deg: float
    angle_rad: float
    distance_mm: float
    x_mm: float
    y_mm: float
    z_mm: float = 0.0
    intensity: float = 255.0
    is_collision: bool = False
    is_warning: bool = False


@dataclass
class LidarScan:
    """Frame container for a complete 360-degree LiDAR scan."""
    timestamp: float
    points: List[ScanPoint] = field(default_factory=list)
    scan_frequency_hz: float = 0.0
    sample_rate_sps: int = 0


@dataclass
class DetectedObject:
    """Clustered object isolated from scan point cloud."""
    object_id: int
    centroid_x_mm: float
    centroid_y_mm: float
    distance_mm: float
    angle_deg: float
    bounding_radius_mm: float
    point_count: int
    is_collision: bool = False
    points: List[ScanPoint] = field(default_factory=list)


@dataclass
class TelemetryData:
    """System hardware performance and stream status telemetry."""
    connected: bool = False
    port: str = "N/A"
    baud_rate: int = 115200
    status: str = "Disconnected"
    fps: float = 0.0
    scan_frequency_hz: float = 0.0
    samples_per_sec: int = 0
    cpu_usage_pct: float = 0.0
    ram_usage_pct: float = 0.0
    temperature_c: float = 0.0
    scan_time_sec: float = 0.0
