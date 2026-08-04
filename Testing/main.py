"""
main.py
-------
Author: SUDHARSAN
Application entry point for LiDAR 3D Viewer Pro desktop application.
Initializes QApplication, configures High-DPI support, and displays MainWindow.
"""

import sys
import os

# Ensure project root directory is in Python module search path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from app.ui.main_window import MainWindow


def main() -> None:
    """Application main entry point."""
    # High-DPI Scaling configuration
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("LiDAR 3D Viewer Pro")
    app.setOrganizationName("Robotics Software Engineering")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
