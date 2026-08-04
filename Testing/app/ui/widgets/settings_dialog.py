"""
app/ui/widgets/settings_dialog.py
---------------------------------
Author: SUDHARSAN
Settings and configuration modal dialog for LiDAR 3D Viewer Pro.
Allows real-time tuning of OpenGL rendering properties, collision safety limits, and serial device parameters.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QGroupBox, QDoubleSpinBox,
    QSpinBox, QComboBox, QCheckBox, QPushButton, QHBoxLayout, QLabel
)

from app.config import AppConfig


class SettingsDialog(QDialog):
    """Configuration dialog for application and rendering settings."""

    def __init__(self, config: AppConfig, parent=None):
        super().__init__(parent)
        self.config = config
        self.setWindowTitle("LiDAR 3D Viewer Pro - Settings")
        self.setFixedSize(420, 520)
        self.setModal(True)
        self.setStyleSheet("""
            QDialog {
                background-color: #161922;
                color: #e1e6f0;
                font-family: 'Segoe UI', Helvetica, Arial, sans-serif;
            }
            QGroupBox {
                background: #1e2230;
                border: 1px solid #2d3448;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                color: #00aaff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
            QLabel {
                color: #c0c7d5;
            }
            QSpinBox, QDoubleSpinBox, QComboBox {
                background-color: #12141c;
                color: #00ffcc;
                border: 1px solid #32394d;
                border-radius: 4px;
                padding: 4px;
            }
            QPushButton {
                background: #252b3d;
                color: #ffffff;
                border: 1px solid #3e4863;
                border-radius: 4px;
                padding: 6px 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #323a52;
            }
            QPushButton#btnSave {
                background: #0077cc;
                border-color: #0099ff;
            }
            QPushButton#btnSave:hover {
                background: #0088ee;
            }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # 1. Rendering & Graphics Group
        grp_gfx = QGroupBox("VIEWPORT & GRAPHICS")
        form_gfx = QFormLayout(grp_gfx)
        form_gfx.setContentsMargins(10, 15, 10, 10)

        self.spin_point_size = QDoubleSpinBox()
        self.spin_point_size.setRange(1.0, 12.0)
        self.spin_point_size.setSingleStep(0.5)
        self.spin_point_size.setValue(self.config.point_size)

        self.spin_fps_limit = QSpinBox()
        self.spin_fps_limit.setRange(15, 144)
        self.spin_fps_limit.setValue(self.config.fps_limit)

        self.spin_cam_speed = QDoubleSpinBox()
        self.spin_cam_speed.setRange(0.2, 5.0)
        self.spin_cam_speed.setSingleStep(0.1)
        self.spin_cam_speed.setValue(self.config.camera_speed)

        form_gfx.addRow("Point Size (px):", self.spin_point_size)
        form_gfx.addRow("Target FPS Limit:", self.spin_fps_limit)
        form_gfx.addRow("Camera Orbit Speed:", self.spin_cam_speed)
        main_layout.addWidget(grp_gfx)

        # 2. Collision Safety & Grid Group
        grp_safety = QGroupBox("SAFETY & ENVIRONMENT")
        form_safety = QFormLayout(grp_safety)
        form_safety.setContentsMargins(10, 15, 10, 10)

        self.spin_collision_r = QDoubleSpinBox()
        self.spin_collision_r.setRange(100.0, 3000.0)
        self.spin_collision_r.setSingleStep(50.0)
        self.spin_collision_r.setSuffix(" mm")
        self.spin_collision_r.setValue(self.config.collision_radius_mm)

        self.spin_grid_extent = QDoubleSpinBox()
        self.spin_grid_extent.setRange(2000.0, 50000.0)
        self.spin_grid_extent.setSingleStep(1000.0)
        self.spin_grid_extent.setSuffix(" mm")
        self.spin_grid_extent.setValue(self.config.grid_extent_mm)

        form_safety.addRow("Collision Radius:", self.spin_collision_r)
        form_safety.addRow("Grid Extent:", self.spin_grid_extent)
        main_layout.addWidget(grp_safety)

        # 3. Hardware & Serial Group
        grp_hw = QGroupBox("SERIAL HARDWARE")
        form_hw = QFormLayout(grp_hw)
        form_hw.setContentsMargins(10, 15, 10, 10)

        self.cmb_baud = QComboBox()
        self.cmb_baud.addItems(["115200", "230400", "460800", "921600"])
        self.cmb_baud.setCurrentText(str(self.config.baud_rate))

        self.chk_auto_reconnect = QCheckBox("Auto Reconnect on USB Disconnect")
        self.chk_auto_reconnect.setChecked(self.config.auto_reconnect)

        form_hw.addRow("Baud Rate:", self.cmb_baud)
        form_hw.addRow("", self.chk_auto_reconnect)
        main_layout.addWidget(grp_hw)

        main_layout.addStretch()

        # Dialog Buttons
        btn_box = QHBoxLayout()
        btn_cancel = QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)

        btn_save = QPushButton("Save & Apply")
        btn_save.setObjectName("btnSave")
        btn_save.clicked.connect(self._save_settings)

        btn_box.addWidget(btn_cancel)
        btn_box.addWidget(btn_save)
        main_layout.addLayout(btn_box)

    def _save_settings(self) -> None:
        """Apply form values back into AppConfig dataclass."""
        self.config.point_size = self.spin_point_size.value()
        self.config.fps_limit = self.spin_fps_limit.value()
        self.config.camera_speed = self.spin_cam_speed.value()
        self.config.collision_radius_mm = self.spin_collision_r.value()
        self.config.grid_extent_mm = self.spin_grid_extent.value()
        self.config.baud_rate = int(self.cmb_baud.currentText())
        self.config.auto_reconnect = self.chk_auto_reconnect.isChecked()
        self.accept()
