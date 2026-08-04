"""
camera_config.py
----------------
Author: SUDHARSAN
Configuration file for the Standalone Arducam B0410 Camera Test Suite.
"""

# Hardware Specifications (Arducam B0410 ToF Camera)
SENSOR_MODEL = "Arducam B0410 ToF"
SENSOR_WIDTH_PX = 240
SENSOR_HEIGHT_PX = 180
FOV_DIAGONAL_DEG = 70.0

# Range Modes (Near Mode: 0.1 - 2.0m, Far Mode: 0.2 - 4.0m)
RANGE_MODE = "FAR"  # "NEAR" or "FAR"
MAX_RANGE_NEAR_MM = 2000
MAX_RANGE_FAR_MM = 4000
MIN_VALID_DEPTH_MM = 100
CONFIDENCE_THRESHOLD = 30

# Window Titles (Individual Feature Tabs)
WIN_AMPLITUDE = "Tab 1: Grayscale Amplitude Stream (940nm IR)"
WIN_DEPTH = "Tab 2: Metric Depth Map (JET & Grayscale)"
WIN_DETECTION = "Tab 3: Contour & Distance Object Detector"
WIN_3D_SURFACE = "Tab 4: 3D Surface Point Cloud Visualizer"
WIN_DIAGNOSTICS = "Tab 5: RAW Phase & Confidence Diagnostics"
WIN_TELEMETRY = "Tab 6: Camera Control & Telemetry Panel"

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
TARGET_FPS = 60

# Filter Parameters
MEDIAN_BLUR_KERNEL = 3
MORPH_KERNEL_SIZE = 3
DETECTION_DISTANCE_MM = 2000
MIN_CONTOUR_AREA_PX = 300

# Capture directory
SAVE_DIR = "camera_test_suite/captures"
