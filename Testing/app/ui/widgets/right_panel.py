"""
app/ui/widgets/right_panel.py
------------------------------
Author: SUDHARSAN
Right object detection and collision warning side-panel for LiDAR 3D Viewer Pro.
Displays nearest object distance/bearing, detected obstacle cluster counts, and flashing safety alarms.
"""

from typing import Optional
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLabel, QFrame
)

from app.core.data_types import DetectedObject


class RightPanel(QWidget):
    """Right side panel displaying real-time object tracking metrics and collision warnings."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(280)
        self.setStyleSheet("""
            QWidget {
                background-color: #12151e;
                color: #d1d7e0;
                font-family: 'Segoe UI', Helvetica, Arial, sans-serif;
            }
            QGroupBox {
                background: #1a1e2b;
                border: 1px solid #2d3448;
                border-radius: 6px;
                margin-top: 12px;
                font-weight: bold;
                font-size: 12px;
                color: #4da6ff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QLabel {
                font-size: 11px;
            }
            QLabel#valLabel {
                font-weight: bold;
                color: #00ffcc;
                font-size: 13px;
            }
            QFrame#alertBox {
                background-color: #1a221a;
                border: 2px solid #00aa44;
                border-radius: 6px;
                padding: 8px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # 1. Nearest Object Tracking Group
        grp_obj = QGroupBox("NEAREST OBJECT")
        form_obj = QFormLayout(grp_obj)
        form_obj.setContentsMargins(10, 15, 10, 10)
        form_obj.setSpacing(8)

        self.lbl_distance = self._create_val_label("--- mm (--- m)")
        self.lbl_angle = self._create_val_label("---°")
        self.lbl_obj_count = self._create_val_label("0 objects")
        self.lbl_scan_time = self._create_val_label("0.00 s")

        form_obj.addRow("Distance:", self.lbl_distance)
        form_obj.addRow("Bearing Angle:", self.lbl_angle)
        form_obj.addRow("Detected Objects:", self.lbl_obj_count)
        form_obj.addRow("Current Scan Time:", self.lbl_scan_time)
        layout.addWidget(grp_obj)

        # 2. Collision Warning Status Box
        grp_warn = QGroupBox("HAZARD & COLLISION STATUS")
        vbox_warn = QVBoxLayout(grp_warn)
        vbox_warn.setContentsMargins(10, 15, 10, 10)

        self.alert_box = QFrame()
        self.alert_box.setObjectName("alertBox")
        box_layout = QVBoxLayout(self.alert_box)
        box_layout.setContentsMargins(4, 4, 4, 4)

        self.lbl_alert_text = QLabel("CLEAR / SAFE")
        self.lbl_alert_text.setAlignment(Qt.AlignCenter)
        self.lbl_alert_text.setStyleSheet("color: #00ff66; font-size: 14px; font-weight: bold;")
        box_layout.addWidget(self.lbl_alert_text)

        vbox_warn.addWidget(self.alert_box)
        layout.addWidget(grp_warn)

        # Flashing timer for collision warning
        self.flash_timer = QTimer(self)
        self.flash_timer.setInterval(300)
        self.flash_timer.timeout.connect(self._toggle_flash_state)
        self.flash_state: bool = False
        self.is_flashing: bool = False

        layout.addStretch()

    def _create_val_label(self, default_text: str = "-", color: str = "#00ffcc") -> QLabel:
        lbl = QLabel(default_text)
        lbl.setObjectName("valLabel")
        lbl.setStyleSheet(f"color: {color}; font-weight: bold;")
        return lbl

    def update_objects_info(
        self,
        nearest_object: Optional[DetectedObject],
        total_objects: int,
        has_collision: bool,
        scan_time_sec: float
    ) -> None:
        """Update right side-panel telemetry and collision status."""
        self.lbl_obj_count.setText(f"{total_objects} detected")
        self.lbl_scan_time.setText(f"{scan_time_sec:.2f} s")

        if nearest_object:
            dist_m = nearest_object.distance_mm / 1000.0
            self.lbl_distance.setText(f"{nearest_object.distance_mm:.1f} mm ({dist_m:.2f} m)")
            self.lbl_angle.setText(f"{nearest_object.angle_deg:.1f}°")
        else:
            self.lbl_distance.setText("--- mm (--- m)")
            self.lbl_angle.setText("---°")

        # Warning status handling
        if has_collision:
            if not self.is_flashing:
                self.is_flashing = True
                self.flash_timer.start()
        else:
            if self.is_flashing:
                self.is_flashing = False
                self.flash_timer.stop()
                self._set_safe_status()
            else:
                self._set_safe_status()

    def _set_safe_status(self) -> None:
        self.alert_box.setStyleSheet("background-color: #1a221a; border: 2px solid #00aa44; border-radius: 6px;")
        self.lbl_alert_text.setText("CLEAR / SAFE")
        self.lbl_alert_text.setStyleSheet("color: #00ff66; font-size: 14px; font-weight: bold;")

    def _toggle_flash_state(self) -> None:
        self.flash_state = not self.flash_state
        if self.flash_state:
            self.alert_box.setStyleSheet("background-color: #4a0000; border: 2px solid #ff0000; border-radius: 6px;")
            self.lbl_alert_text.setText("🚨 COLLISION WARNING 🚨")
            self.lbl_alert_text.setStyleSheet("color: #ffffff; font-size: 14px; font-weight: bold;")
        else:
            self.alert_box.setStyleSheet("background-color: #1a0000; border: 2px solid #aa0000; border-radius: 6px;")
            self.lbl_alert_text.setText("⚠️ ZONE INTRUSION ⚠️")
            self.lbl_alert_text.setStyleSheet("color: #ff3333; font-size: 14px; font-weight: bold;")
