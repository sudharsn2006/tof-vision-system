"""
lidar_config.py
----------------
Author: SUDHARSAN
Configuration file for the Standalone LiDAR 2D/3D Test Suite.
"""

# Serial / Port settings
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200

# Range and Frequency parameters
LIDAR_MAX_RANGE_M = 10.0
LIDAR_MIN_RANGE_M = 0.05
LIDAR_SAMPLE_RATE = 5         # kHz
LIDAR_FREQUENCY = 12.0        # Hz motor scan rate

# Display & Window Settings
WINDOW_2D_NAME = "LiDAR 2D Radar View"
WINDOW_3D_NAME = "LiDAR 3D Point Cloud View"
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600
TARGET_FPS = 60

# Safety Proximity Warning Thresholds (meters)
SAFETY_CRITICAL_M = 0.5       # Red alert distance
SAFETY_WARNING_M = 1.2        # Yellow warning distance

# Clustering & Obstacle Detection
CLUSTER_EPSILON_M = 0.25      # Max distance between points in same cluster
MIN_CLUSTER_POINTS = 4        # Minimum points to form a valid obstacle cluster

# Capture Output
SAVE_DIR = "lidar_test_suite/captures"
