"""
app/ui/widgets/toolbar.py
--------------------------
Author: SUDHARSAN
Top application action toolbar for LiDAR 3D Viewer Pro.
Contains hardware connection controls, stream triggers, view reset, screenshot, recording, and settings.
"""

from PySide6.QtCore import Signal, QSize
from PySide6.QtGui import QAction, QIcon
from PySide6.QtWidgets import QToolBar, QWidget, QSizePolicy, QLabel


class MainToolBar(QToolBar):
    """Top Action Toolbar with styled buttons and status indicator."""

    connect_requested = Signal()
    disconnect_requested = Signal()
    start_scan_requested = Signal()
    stop_scan_requested = Signal()
    reset_view_requested = Signal()
    screenshot_requested = Signal()
    record_toggled = Signal(bool)
    settings_requested = Signal()
    exit_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("Main Toolbar", parent)
        self.setMovable(False)
        self.setIconSize(QSize(20, 20))
        self.setStyleSheet("""
            QToolBar {
                background: #161922;
                border-bottom: 1px solid #2a2f3d;
                spacing: 8px;
                padding: 4px 8px;
            }
            QToolButton {
                background: #212635;
                color: #e1e6f0;
                border: 1px solid #32394d;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 12px;
            }
            QToolButton:hover {
                background: #2d3448;
                border-color: #4f5b7c;
            }
            QToolButton:pressed {
                background: #1b1f2b;
            }
            QToolButton:checked {
                background: #8b0000;
                color: #ffffff;
                border-color: #ff3333;
            }
        """)

        # Add Actions
        self.btn_connect = self.addAction("Connect")
        self.btn_connect.triggered.connect(self.connect_requested.emit)

        self.btn_disconnect = self.addAction("Disconnect")
        self.btn_disconnect.triggered.connect(self.disconnect_requested.emit)

        self.addSeparator()

        self.btn_start_scan = self.addAction("Start Scan")
        self.btn_start_scan.triggered.connect(self.start_scan_requested.emit)

        self.btn_stop_scan = self.addAction("Stop Scan")
        self.btn_stop_scan.triggered.connect(self.stop_scan_requested.emit)

        self.addSeparator()

        self.btn_reset_view = self.addAction("Reset View")
        self.btn_reset_view.triggered.connect(self.reset_view_requested.emit)

        self.btn_screenshot = self.addAction("Screenshot")
        self.btn_screenshot.triggered.connect(self.screenshot_requested.emit)

        self.btn_record = self.addAction("Record CSV")
        self.btn_record.setCheckable(True)
        self.btn_record.toggled.connect(self.record_toggled.emit)

        self.addSeparator()

        self.btn_settings = self.addAction("Settings")
        self.btn_settings.triggered.connect(self.settings_requested.emit)

        # Spacer
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.addWidget(spacer)

        self.btn_exit = self.addAction("Exit")
        self.btn_exit.triggered.connect(self.exit_requested.emit)

    def update_connection_state(self, connected: bool) -> None:
        """Update toolbar button enabled states based on hardware connection."""
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.btn_start_scan.setEnabled(connected)
        self.btn_stop_scan.setEnabled(connected)
