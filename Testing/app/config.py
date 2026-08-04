"""
app/config.py
-------------
Author: SUDHARSAN
Global configuration manager for LiDAR 3D Viewer Pro. Handles runtime
settings and persistent JSON storage.
"""

import json
import os
from dataclasses import dataclass, field, asdict
from typing import List


@dataclass
class AppConfig:
    """Application configuration schema."""
    # Serial Communication
    port: str = "AUTO"
    baud_rate: int = 115200
    timeout: float = 1.0
    auto_reconnect: bool = True

    # Viewport & Rendering
    point_size: float = 6.0
    fps_limit: int = 60
    camera_speed: float = 1.0
    bg_color: List[float] = field(default_factory=lambda: [0.08, 0.09, 0.12, 1.0])
    grid_extent_mm: float = 10000.0  # 10m x 10m
    minor_grid_mm: float = 100.0     # 100mm grid spacing
    major_grid_mm: float = 1000.0    # 1000mm major spacing

    # Safety & Collision
    collision_radius_mm: float = 500.0  # 0.5m collision threshold
    warning_radius_mm: float = 1500.0   # 1.5m warning threshold
    cluster_eps_mm: float = 200.0       # Euclidean clustering distance threshold
    min_cluster_size: int = 3           # Minimum points per object

    # HUD & Minimap
    show_hud: bool = True
    show_minimap: bool = True
    show_laser_sweep: bool = True
    show_distance_rings: bool = True
    show_robot_model: bool = True

    # Export paths
    captures_dir: str = "captures"
    records_dir: str = "records"


class ConfigManager:
    """Manages reading and writing application settings to JSON file."""

    CONFIG_FILE = "config.json"

    def __init__(self, filepath: str = CONFIG_FILE):
        self.filepath = filepath
        self.config = AppConfig()
        self.load()

    def load(self) -> AppConfig:
        """Load configuration from disk if exists, otherwise write defaults."""
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.config = AppConfig(**{k: v for k, v in data.items() if k in AppConfig.__dataclass_fields__})
            except Exception as e:
                print(f"[ConfigManager] Warning loading config ({e}), using defaults.")
                self.config = AppConfig()
        else:
            self.save()
        return self.config

    def save(self) -> None:
        """Save current configuration to JSON file."""
        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(asdict(self.config), f, indent=4)
        except Exception as e:
            print(f"[ConfigManager] Error saving config: {e}")
