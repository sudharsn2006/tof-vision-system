"""
app/ui/widgets/left_panel.py
-----------------------------
Author: SUDHARSAN
Left telemetry side-panel for LiDAR 3D Viewer Pro.
Displays hardware COM parameters, connection status, frame metrics, and Pi hardware system resource stats.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLabel, QProgressBar
)

from app.core.data_types import TelemetryData


class LeftPanel(QWidget):
    """Left side panel displaying real-time system and LiDAR device metrics."""

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
            }
            QProgressBar {
                background-color: #0e1118;
                border: 1px solid #2d3448;
                border-radius: 3px;
                text-align: center;
                height: 14px;
                font-size: 9px;
                color: #ffffff;
            }
            QProgressBar::chunk {
                background-color: #00aaee;
                border-radius: 2px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # 1. Device Information Group
        grp_device = QGroupBox("DEVICE INFORMATION")
        form_dev = QFormLayout(grp_device)
        form_dev.setContentsMargins(10, 15, 10, 10)
        form_dev.setSpacing(6)

        self.lbl_port = self._create_val_label("AUTO")
        self.lbl_baud = self._create_val_label("115200")
        self.lbl_status = self._create_val_label("Disconnected", color="#ff4444")

        form_dev.addRow("COM Port:", self.lbl_port)
        form_dev.addRow("Baud Rate:", self.lbl_baud)
        form_dev.addRow("Status:", self.lbl_status)
        layout.addWidget(grp_device)

        # 2. Performance Metrics Group
        grp_perf = QGroupBox("PERFORMANCE METRICS")
        form_perf = QFormLayout(grp_perf)
        form_perf.setContentsMargins(10, 15, 10, 10)
        form_perf.setSpacing(6)

        self.lbl_fps = self._create_val_label("0.0 FPS")
        self.lbl_scan_freq = self._create_val_label("0.0 Hz")
        self.lbl_samples = self._create_val_label("0 sps")

        form_perf.addRow("Render FPS:", self.lbl_fps)
        form_perf.addRow("Scan Frequency:", self.lbl_scan_freq)
        form_perf.addRow("Samples/sec:", self.lbl_samples)
        layout.addWidget(grp_perf)

        # 3. System Hardware Resources Group
        grp_sys = QGroupBox("HARDWARE RESOURCES")
        form_sys = QFormLayout(grp_sys)
        form_sys.setContentsMargins(10, 15, 10, 10)
        form_sys.setSpacing(8)

        self.bar_cpu = QProgressBar()
        self.bar_cpu.setRange(0, 100)
        self.bar_cpu.setValue(0)

        self.bar_ram = QProgressBar()
        self.bar_ram.setRange(0, 100)
        self.bar_ram.setValue(0)

        self.lbl_temp = self._create_val_label("N/A °C", color="#ffaa00")

        form_sys.addRow("CPU Usage:", self.bar_cpu)
        form_sys.addRow("RAM Usage:", self.bar_ram)
        form_sys.addRow("CPU Temp:", self.lbl_temp)
        layout.addWidget(grp_sys)

        layout.addStretch()

    def _create_val_label(self, default_text: str = "-", color: str = "#00ffcc") -> QLabel:
        lbl = QLabel(default_text)
        lbl.setObjectName("valLabel")
        lbl.setStyleSheet(f"color: {color};")
        return lbl

    def update_telemetry(self, data: TelemetryData) -> None:
        """Update telemetry GUI fields."""
        self.lbl_port.setText(data.port)
        self.lbl_baud.setText(str(data.baud_rate))

        if data.connected:
            self.lbl_status.setText("CONNECTED")
            self.lbl_status.setStyleSheet("color: #00ff66; font-weight: bold;")
        else:
            self.lbl_status.setText(data.status.upper())
            self.lbl_status.setStyleSheet("color: #ff4444; font-weight: bold;")

        self.lbl_fps.setText(f"{data.fps:.1f} FPS")
        self.lbl_scan_freq.setText(f"{data.scan_frequency_hz:.1f} Hz")
        self.lbl_samples.setText(f"{data.samples_per_sec} sps")

        self.bar_cpu.setValue(int(data.cpu_usage_pct))
        self.bar_ram.setValue(int(data.ram_usage_pct))

        if data.temperature_c > 0:
            self.lbl_temp.setText(f"{data.temperature_c:.1f} °C")
        else:
            self.lbl_temp.setText("N/A")
