# Centralized configuration parameters for the Virtual Scanner system.

import logging

# General Logging Configuration
LOGGING_LEVEL = logging.INFO
LOGGING_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

# System Processing Parameters
CHUNK_SIZE = 10_000_000      # Number of points to process per chunk (memory handling)
NUM_WORKERS = 4              # Default number of parallel workers

# Voxel & Occupancy Grid
VOXEL_SIZE = 0.1             # Voxel size in meters (5-10cm recommended)

# Scan Station Generation
TARGET_SCAN_SPACING_MIN = 2.0  # Minimum distance between scans indoors (meters)
TARGET_SCAN_SPACING_MAX = 4.0  # Maximum distance between scans indoors (meters)

# Raycasting & Visibility Optimization
MIN_OVERLAP_PERCENTAGE = 0.40  # Minimum 40% overlap needed between neighboring scans
TARGET_COVERAGE_PERCENTAGE = 0.99 # Stop when 99% of visible points are covered
RAY_DENSITY_MULTIPLIER = 1.0   # Multiplier for adaptive ray density
MAX_RAY_DISTANCE = 50.0        # Max distance a synthetic scanner ray can travel before stopping

# Bounding Box Padding (for local scene chunking)
BBOX_PADDING = 0.5           # Meters to pad chunks for seamless processing

# Phase 1 Usability & Abstractions
DEFAULT_MAX_SCAN_RANGE = 50.0 # Default max range for hard point exclusion
VISIBILITY_ENGINE = 'hpr'     # Active visibility engine ('nearest', 'hpr', 'future')
STATION_PROVIDER = 'PoseCSVProvider' # Active station placement provider

