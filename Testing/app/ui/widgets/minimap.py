"""
app/ui/widgets/minimap.py
--------------------------
Author: SUDHARSAN
2D Top-Down Radar Minimap Overlay Widget.
Displays real-time polar point cloud projection, range rings, and collision zones in a compact view.
"""

import math
from typing import List, Optional
from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PySide6.QtWidgets import QWidget

from app.core.data_types import ScanPoint, LidarScan


class MinimapWidget(QWidget):
    """2D Radar minimap widget rendered with PySide6 QPainter."""

    def __init__(self, max_range_mm: float = 4000.0, parent=None):
        super().__init__(parent)
        self.max_range_mm = max_range_mm
        self.setFixedSize(180, 180)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        self.setStyleSheet("background-color: rgba(15, 18, 26, 210); border: 1px solid #2d3448; border-radius: 8px;")

        self.current_scan: Optional[LidarScan] = None
        self.sweep_angle_deg: float = 0.0

    def update_scan(self, scan: LidarScan, sweep_angle_deg: float = 0.0) -> None:
        """Receive latest scan data and trigger repaint."""
        self.current_scan = scan
        self.sweep_angle_deg = sweep_angle_deg
        self.update()

    def paintEvent(self, event) -> None:
        """Render radar minimap background, range rings, points, and robot orientation."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0
        radius = min(w, h) / 2.0 - 10.0

        # Scale factor: pixels per millimeter
        scale = radius / self.max_range_mm

        # Draw Radar Outer Border & Center Crosshair
        painter.setPen(QPen(QColor(45, 52, 72), 1))
        painter.setBrush(QBrush(QColor(12, 15, 23, 220)))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # Crosshairs
        painter.setPen(QPen(QColor(60, 70, 95, 150), 1, Qt.DashLine))
        painter.drawLine(QPointF(cx, cy - radius), QPointF(cx, cy + radius))
        painter.drawLine(QPointF(cx - radius, cy), QPointF(cx + radius, cy))

        # Concentric Distance Rings (0.5m, 1.0m, 2.0m, 3.0m, 4.0m)
        ring_ranges_m = [0.5, 1.0, 2.0, 3.0, 4.0]
        painter.setFont(QFont("Arial", 7))
        for r_m in ring_ranges_m:
            r_px = (r_m * 1000.0) * scale
            if r_px < radius:
                painter.setPen(QPen(QColor(0, 170, 255, 60), 1, Qt.DashLine))
                painter.drawEllipse(QPointF(cx, cy), r_px, r_px)
                # Label
                painter.setPen(QPen(QColor(120, 150, 180, 180)))
                painter.drawText(int(cx + 2), int(cy - r_px - 1), f"{r_m}m")

        # Laser Radar Sweep Line Animation
        sweep_rad = math.radians(self.sweep_angle_deg)
        sx = cx + radius * math.cos(sweep_rad)
        sy = cy - radius * math.sin(sweep_rad)
        painter.setPen(QPen(QColor(0, 255, 200, 120), 1.5))
        painter.drawLine(QPointF(cx, cy), QPointF(sx, sy))

        # Render Scan Point Cloud
        if self.current_scan and self.current_scan.points:
            for pt in self.current_scan.points:
                if pt.distance_mm <= 0:
                    continue
                px = cx + pt.x_mm * scale
                py = cy - pt.y_mm * scale  # Invert Y for screen space

                # Color classification
                if pt.is_collision:
                    color = QColor(255, 40, 40, 240)
                elif pt.is_warning:
                    color = QColor(255, 220, 0, 220)
                else:
                    color = QColor(0, 255, 120, 200)

                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(color))
                painter.drawEllipse(QPointF(px, py), 1.5, 1.5)

        # Center Robot Symbol (Small Blue Triangle + Red Forward Line)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 170, 255, 255)))
        painter.drawEllipse(QPointF(cx, cy), 4.0, 4.0)

        # Forward Direction Arrow
        painter.setPen(QPen(QColor(255, 50, 50, 255), 2))
        painter.drawLine(QPointF(cx, cy), QPointF(cx + 8.0, cy))
