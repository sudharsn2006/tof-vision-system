"""
config.py
---------
Author: SUDHARSAN
Central configuration for the ToF Depth Object Detector + LiDAR monitor.

Keeping every tunable value in one place makes it easy to adjust behavior
for a specific test bench without hunting through the codebase.
"""

# ----------------------------------------------------------------------
# Arducam B0410 ToF Camera settings
# ----------------------------------------------------------------------
# The Arducam ToF SDK reports depth in millimeters. We convert to meters
# for all user-facing output.
TOF_MAX_RANGE_MM = 4000          # Sensor's usable max range (mm). B0410 ~ 2-4m depending on mode.
TOF_MIN_VALID_MM = 100           # Anything closer than this is treated as sensor noise/glare.
TOF_INVALID_CONFIDENCE_THRESH = 30  # Confidence/amplitude threshold below which a pixel is discarded.

# Depth threshold used to separate "foreground objects" from empty background/floor.
# Any pixel with depth between TOF_MIN_VALID_MM and DETECTION_DISTANCE_MM is considered
# a candidate object pixel. Adjust this per test scenario.
DETECTION_DISTANCE_MM = 2000      # 2.0 meters default detection range

# ----------------------------------------------------------------------
# Noise filtering / morphology (optimized for Pi 4 performance)
# ----------------------------------------------------------------------
MEDIAN_BLUR_KERNEL = 3            # Kernel size 3 is much faster on Pi 4 CPU than 5.
MORPH_KERNEL_SIZE = 3             # Reduced kernel size for speed.
MORPH_OPEN_ITERATIONS = 1
MORPH_CLOSE_ITERATIONS = 1

# ----------------------------------------------------------------------
# Contour / object filtering
# ----------------------------------------------------------------------
MIN_CONTOUR_AREA_PX = 400         # Ignore contours smaller than this (noise blobs).

# ----------------------------------------------------------------------
# Camera Stream Mode
# ----------------------------------------------------------------------
# Modes: "NORMAL" (Fast normal IR video feed) or "DEPTH" (ToF metric depth)
CAMERA_MODE = "NORMAL"

# ----------------------------------------------------------------------
# Display & Visualization
# ----------------------------------------------------------------------
DEPTH_WINDOW_NAME = "Camera View (Normal)"
OBJECT_WINDOW_NAME = "Object Distance"
LIDAR_WINDOW_NAME = "LiDAR 2D View"
VIEW_3D_WINDOW_NAME = "3D Point Cloud View"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
USE_GRAYSCALE_DEPTH = True        # Grayscale mode reduces CPU lag compared to jet colormap.
ENABLE_3D_VIEW = True             # Real-time 3D Point Cloud Visualization

# ----------------------------------------------------------------------
# Performance
# ----------------------------------------------------------------------
TARGET_FPS = 60                   # Target 60 FPS loop execution

# ----------------------------------------------------------------------
# YDLIDAR X2 settings (Maximized for area coverage & detail)
# ----------------------------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"       # Adjust to match your system (check `ls /dev/ttyUSB*`)
LIDAR_BAUDRATE = 115200
LIDAR_MAX_RANGE_M = 10.0          # Max sensing range extended to 10.0m
LIDAR_MIN_RANGE_M = 0.05
LIDAR_SAMPLE_RATE = 5            # Max 5 kHz sample rate for maximum point density
LIDAR_FREQUENCY = 12.0           # Max 12 Hz scan frequency for maximum area refresh

# ----------------------------------------------------------------------
# Output / saving
# ----------------------------------------------------------------------
SAVE_DIR = "captures"
