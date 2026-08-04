"""
app/ui/main_window.py
---------------------
Author: SUDHARSAN
Primary QMainWindow application shell for LiDAR 3D Viewer Pro.
Integrates top toolbar, side telemetry & hazard panels, 3D OpenGL viewport, 2D radar minimap,
multithreaded hardware streaming (real serial & fallback mock generator), CSV recorder, and settings dialog.
"""

import os
import sys
import time
from typing import Optional

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QIcon, QKeySequence, QShortcut, QCloseEvent
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QMessageBox, QApplication
)

from app.config import ConfigManager, AppConfig
from app.core.data_types import LidarScan, TelemetryData
from app.core.object_detector import ObjectDetector
from app.core.recorder import DataRecorder, ScreenshotManager
from app.core.system_stats import SystemStatsMonitor
from app.hardware.serial_lidar import LidarThread
from app.hardware.mock_lidar import MockLidarThread
from app.ui.widgets.toolbar import MainToolBar
from app.ui.widgets.left_panel import LeftPanel
from app.ui.widgets.right_panel import RightPanel
from app.ui.widgets.gl_viewport import OpenGLViewport
from app.ui.widgets.minimap import MinimapWidget
from app.ui.widgets.settings_dialog import SettingsDialog


class MainWindow(QMainWindow):
    """Main RViz-like Desktop Window for LiDAR 3D Viewer Pro."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LiDAR 3D Viewer Pro - [Raspberry Pi 4 / USB LiDAR]")
        self.setMinimumSize(1280, 720)
        self.resize(1600, 900)

        # 1. Load Configuration
        self.config_manager = ConfigManager()
        self.config: AppConfig = self.config_manager.config

        # 2. Core Processing Engines
        self.detector = ObjectDetector(
            cluster_eps_mm=self.config.cluster_eps_mm,
            min_cluster_size=self.config.min_cluster_size,
            collision_radius_mm=self.config.collision_radius_mm
        )
        self.recorder = DataRecorder(records_dir=self.config.records_dir)
        self.screenshot_mgr = ScreenshotManager(captures_dir=self.config.captures_dir)
        self.sys_stats = SystemStatsMonitor()

        # Telemetry State
        self.telemetry = TelemetryData(
            port=self.config.port,
            baud_rate=self.config.baud_rate
        )
        self.start_scan_time: Optional[float] = None
        self.lidar_thread: Optional[object] = None

        # Audio Beep Throttle
        self.last_beep_time: float = 0.0

        # 3. Setup UI Layout
        self._init_ui()

        # 4. Setup Timers
        self.stats_timer = QTimer(self)
        self.stats_timer.setInterval(1000)  # Update CPU/RAM/Temp stats every 1 sec
        self.stats_timer.timeout.connect(self._update_system_stats)
        self.stats_timer.start()

        # Keyboard Short-cuts (F11 Fullscreen)
        self.shortcut_fs = QShortcut(QKeySequence(Qt.Key_F11), self)
        self.shortcut_fs.activated.connect(self._toggle_fullscreen)

        # Auto-connect on startup
        QTimer.singleShot(500, self._auto_connect_hardware)

    def _init_ui(self) -> None:
        """Construct application dark layout hierarchy."""
        self.setStyleSheet("background-color: #0b0d13; color: #ffffff;")

        # Central Widget & Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_vbox = QVBoxLayout(central_widget)
        main_vbox.setContentsMargins(0, 0, 0, 0)
        main_vbox.setSpacing(0)

        # Top Toolbar
        self.toolbar = MainToolBar(self)
        self.toolbar.connect_requested.connect(self.connect_lidar)
        self.toolbar.disconnect_requested.connect(self.disconnect_lidar)
        self.toolbar.start_scan_requested.connect(self.start_scan)
        self.toolbar.stop_scan_requested.connect(self.stop_scan)
        self.toolbar.reset_view_requested.connect(self.reset_viewport_camera)
        self.toolbar.screenshot_requested.connect(self.take_screenshot)
        self.toolbar.record_toggled.connect(self.toggle_recording)
        self.toolbar.settings_requested.connect(self.open_settings_dialog)
        self.toolbar.exit_requested.connect(self.close)
        main_vbox.addWidget(self.toolbar)

        # Workspace Body Layout (Left Panel | 3D Viewport + Overlay Minimap | Right Panel)
        body_hbox = QHBoxLayout()
        body_hbox.setContentsMargins(0, 0, 0, 0)
        body_hbox.setSpacing(0)

        # Left Telemetry Panel
        self.left_panel = LeftPanel(self)
        body_hbox.addWidget(self.left_panel)

        # Center Container for OpenGL Viewport & Overlay Minimap
        center_container = QWidget()
        center_layout = QVBoxLayout(center_container)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(0)

        self.gl_viewport = OpenGLViewport(self.config, self)
        self.gl_viewport.fps_updated.connect(self._on_fps_updated)
        center_layout.addWidget(self.gl_viewport)

        # Overlay 2D Radar Minimap (Top-Right corner of viewport)
        self.minimap = MinimapWidget(max_range_mm=self.config.grid_extent_mm / 2.0, parent=self.gl_viewport)
        self.minimap.show()

        body_hbox.addWidget(center_container, stretch=1)

        # Right Hazard Panel
        self.right_panel = RightPanel(self)
        body_hbox.addWidget(self.right_panel)

        main_vbox.addLayout(body_hbox, stretch=1)

        self.toolbar.update_connection_state(False)

    def _position_minimap(self) -> None:
        if hasattr(self, 'minimap') and hasattr(self, 'gl_viewport'):
            mw = self.minimap.width()
            vw = self.gl_viewport.width()
            if vw > mw + 30:
                self.minimap.move(vw - mw - 15, 15)

    def showEvent(self, event) -> None:
        """Position minimap in top-right corner on initial window display."""
        super().showEvent(event)
        self._position_minimap()

    def resizeEvent(self, event) -> None:
        """Keep minimap widget anchored in top-right corner of viewport."""
        super().resizeEvent(event)
        self._position_minimap()

    def _auto_connect_hardware(self) -> None:
        """Attempt hardware USB auto-connect or fall back to mock stream."""
        self.connect_lidar()

    @Slot()
    def connect_lidar(self) -> None:
        """Establish connection with physical YDLIDAR X2 USB device or launch mock generator."""
        if self.lidar_thread and self.lidar_thread.isRunning():
            return

        print("[MainWindow] Attempting to connect to LiDAR hardware...")
        # First attempt physical serial connection
        real_thread = LidarThread(port=self.config.port, baud_rate=self.config.baud_rate)
        real_thread.status_changed.connect(self._on_lidar_status_changed)
        real_thread.connection_failed.connect(self._on_serial_failed_fallback_mock)
        real_thread.scan_ready.connect(self._on_scan_ready)

        self.lidar_thread = real_thread
        self.lidar_thread.start()

    @Slot(str)
    def _on_serial_failed_fallback_mock(self, err_msg: str) -> None:
        """Fallback to mock stream generator if physical serial port is not present."""
        print(f"[MainWindow] Serial connection note ({err_msg}). Launching fallback Mock LiDAR stream.")
        if self.lidar_thread:
            self.lidar_thread.disconnect()

        mock_thread = MockLidarThread(target_freq_hz=8.0, num_samples=720)
        mock_thread.status_changed.connect(self._on_lidar_status_changed)
        mock_thread.scan_ready.connect(self._on_scan_ready)

        self.lidar_thread = mock_thread
        self.lidar_thread.start()

    @Slot()
    def disconnect_lidar(self) -> None:
        """Stop LiDAR hardware stream."""
        if self.lidar_thread:
            if hasattr(self.lidar_thread, 'stop'):
                self.lidar_thread.stop()
            self.lidar_thread = None

        self.telemetry.connected = False
        self.telemetry.status = "Disconnected"
        self.left_panel.update_telemetry(self.telemetry)
        self.toolbar.update_connection_state(False)
        self.start_scan_time = None

    @Slot()
    def start_scan(self) -> None:
        """Start scan processing."""
        if not self.telemetry.connected:
            self.connect_lidar()

    @Slot()
    def stop_scan(self) -> None:
        """Pause scan stream."""
        self.disconnect_lidar()

    @Slot()
    def reset_viewport_camera(self) -> None:
        """Reset 3D OpenGL Viewport camera."""
        self.gl_viewport.camera.reset()

    @Slot()
    def take_screenshot(self) -> None:
        """Capture PNG screenshot of 3D OpenGL Viewport."""
        rgb_arr = self.gl_viewport.capture_frame_np()
        path = self.screenshot_mgr.save_image(rgb_arr)
        QMessageBox.information(self, "Screenshot Saved", f"Saved 3D Viewport image to:\n{path}")

    @Slot(bool)
    def toggle_recording(self, checked: bool) -> None:
        """Toggle CSV scan data recording."""
        if checked:
            path = self.recorder.start_recording()
            self.statusBar().showMessage(f"Recording CSV scan data to {path}...", 4000)
        else:
            path = self.recorder.stop_recording()
            if path:
                QMessageBox.information(self, "Recording Saved", f"Saved CSV scan log to:\n{path}")

    @Slot()
    def open_settings_dialog(self) -> None:
        """Open settings dialog modal."""
        dialog = SettingsDialog(self.config, self)
        if dialog.exec() == SettingsDialog.Accepted:
            self.config_manager.save()
            # Update detector threshold parameters
            self.detector.collision_radius_mm = self.config.collision_radius_mm
            self.gl_viewport.update()

    def _toggle_fullscreen(self) -> None:
        """Toggle window full screen state."""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    @Slot(str)
    def _on_lidar_status_changed(self, status: str) -> None:
        """Handle status signal from hardware worker thread."""
        is_conn = "Connected" in status
        self.telemetry.connected = is_conn
        self.telemetry.status = status
        if is_conn:
            self.telemetry.port = getattr(self.lidar_thread, 'driver', None) and self.lidar_thread.driver.active_port or "Mock Device"
            if self.start_scan_time is None:
                self.start_scan_time = time.time()
        self.left_panel.update_telemetry(self.telemetry)
        self.toolbar.update_connection_state(is_conn)

    @Slot(float)
    def _on_fps_updated(self, fps: float) -> None:
        """Update FPS in telemetry model."""
        self.telemetry.fps = fps

    @Slot(object)
    def _on_scan_ready(self, scan: LidarScan) -> None:
        """Process incoming 360-degree scan frame."""
        if not scan:
            return

        # 1. Update Telemetry Metrics
        self.telemetry.scan_frequency_hz = scan.scan_frequency_hz
        self.telemetry.samples_per_sec = scan.sample_rate_sps
        scan_time_sec = (time.time() - self.start_scan_time) if self.start_scan_time else 0.0
        self.telemetry.scan_time_sec = scan_time_sec
        self.left_panel.update_telemetry(self.telemetry)

        # 2. Perform Spatial Object Clustering & Collision Detection
        detected_objects, nearest_obj, has_collision = self.detector.process_scan(scan)

        # 3. Handle Audio Beep Alert on Collision
        if has_collision:
            now = time.time()
            if now - self.last_beep_time > 0.4:
                QApplication.beep()
                self.last_beep_time = now

        # 4. Log scan to CSV if recording active
        if self.recorder.is_recording:
            self.recorder.record_scan(scan)

        # 5. Push data to 3D Viewport & 2D Minimap
        self.gl_viewport.update_scan_data(scan, detected_objects, nearest_obj, has_collision)
        self.minimap.update_scan(scan, sweep_angle_deg=scan.points[-1].angle_deg if scan.points else 0.0)

        # 6. Update Right Side-Panel
        self.right_panel.update_objects_info(nearest_obj, len(detected_objects), has_collision, scan_time_sec)

    def _update_system_stats(self) -> None:
        """Periodically refresh CPU, RAM, and Temperature metrics."""
        self.telemetry.cpu_usage_pct = self.sys_stats.get_cpu_usage()
        self.telemetry.ram_usage_pct = self.sys_stats.get_ram_usage()
        self.telemetry.temperature_c = self.sys_stats.get_cpu_temperature()
        self.left_panel.update_telemetry(self.telemetry)

    def closeEvent(self, event: QCloseEvent) -> None:
        """Gracefully terminate threads and save state on application exit."""
        self.disconnect_lidar()
        if self.recorder.is_recording:
            self.recorder.stop_recording()
        self.config_manager.save()
        event.accept()
