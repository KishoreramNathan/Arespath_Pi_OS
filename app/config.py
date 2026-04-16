"""Central configuration for Arespath Rover.

All tuneable constants live here — change values here, not scattered
through the codebase.
"""
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent.parent
MAPS_DIR = BASE_DIR / "maps"
DATA_DIR = BASE_DIR / "data"
SNAPSHOT_DIR = BASE_DIR / "snapshots"
PHOTOS_DIR   = BASE_DIR / "photos"

MAPS_DIR.mkdir(exist_ok=True, parents=True)
DATA_DIR.mkdir(exist_ok=True, parents=True)
SNAPSHOT_DIR.mkdir(exist_ok=True, parents=True)
PHOTOS_DIR.mkdir(exist_ok=True, parents=True)

# ── Robot geometry ────────────────────────────────────────────────────────────
WHEEL_RADIUS_M = 0.060        # metres — half of 120 mm wheel diameter
WHEEL_BASE_M   = 0.400        # centre-to-centre wheel track (matches WHEEL_TRACK_M)

# ── OE-37 Encoder ─────────────────────────────────────────────────────────────
# The OE-37 is a magnetic quadrature encoder fitted on 37 mm gearmotors.
# It produces 11 pulses per revolution (PPR) on the motor shaft.
# Quadrature decoding (4× mode) gives 44 counts per motor revolution.
# Multiply by the gearbox ratio to get counts per wheel revolution.
#
#   Common gear ratios: 10:1, 20:1, 25:1, 30:1, 34:1, 50:1, 75:1
#   ► Change MOTOR_GEAR_RATIO to match your specific motor variant.
ENCODER_MODEL       = "OE-37"
ENCODER_PPR         = 11       # pulses per motor revolution (OE-37 datasheet)
ENCODER_QUADRATURE  = 4        # 4× quadrature decoding
MOTOR_GEAR_RATIO    = 30.0     # ← set this to your gearbox ratio
TICKS_PER_WHEEL_REV = float(ENCODER_PPR * ENCODER_QUADRATURE * MOTOR_GEAR_RATIO)
# = 1320 counts/rev at 30:1  |  1496 at 34:1  |  3300 at 75:1

# ── Drive limits ──────────────────────────────────────────────────────────────
MAX_PWM = 180                  # maximum PWM value sent to Arduino
MAX_LINEAR_MPS = 0.40          # max forward speed in m/s (normalised 1.0 → this)
MAX_ANGULAR_RADPS = 1.5        # max yaw rate in rad/s
OBSTACLE_STOP_DISTANCE_M       = 0.35   # lidar forward-cone stop threshold
OBSTACLE_WAIT_BEFORE_REPLAN_S  = 5.0   # wait this long for obstacle to move before replanning
OBSTACLE_REPLAN_REVERSE_S      = 0.8   # brief reverse after patience expires before replanning
OBSTACLE_REVERSE_SPEED         = -0.35 # linear speed fraction for reverse (-1..0)

# ── Rover physical geometry (used for radar rendering) ────────────────────────
ROVER_LENGTH_M  = 0.520   # front-to-back (metres)
ROVER_WIDTH_M   = 0.500   # left-to-right (metres)
ROVER_HEIGHT_M  = 0.300   # floor-to-top  (metres)
WHEEL_DIAMETER_M = 0.120  # wheel diameter
WHEEL_TRACK_M    = 0.400  # centre-to-centre distance between left and right wheels
# LiDAR position in robot frame (x=forward, y=left, origin at rover centre):
#   From front-right-bottom corner (user coords x=500mm, y=250mm, z=200mm):
#   rover is 520mm long → 260mm from centre to front → front offset = 260 - (520-500) = 240mm
#   rover is 500mm wide → 250mm from centre to right → centred: 250 - 250 = 0mm
LIDAR_OFFSET_X_M = 0.240   # forward of rover centre (+x = forward)
LIDAR_OFFSET_Y_M = 0.000   # laterally centred
DEFAULT_SPEED_PCT = 55         # default speed slider value (%)

# ── Control loop ──────────────────────────────────────────────────────────────
CONTROL_HZ = 20.0              # main robot control loop frequency
CMD_WORKER_HZ = 50.0           # serial command-dispatch worker frequency
MANUAL_COMMAND_TIMEOUT_S = 0.4  # manual command expires after this many seconds
TELEMETRY_TIMEOUT_S = 1.5      # mark disconnected after this gap in telemetry
HEARTBEAT_TIMEOUT_S = 3.0      # frontend heartbeat watchdog timeout
WATCHDOG_IDLE_S = 0.5          # idle watchdog — send STOP if no command issued
NAV_REPLAN_SECONDS = 1.5       # re-run A* planner interval
WAYPOINT_REACHED_M = 0.15      # waypoint is consumed when within this radius
GOAL_TOLERANCE_M = 0.20        # navigation goal reached threshold
HEADING_TOLERANCE_RAD = 0.25
PATH_LOOKAHEAD_M = 0.30        # pure-pursuit lookahead distance
NAV_LINEAR_GAIN = 0.90
NAV_ANGULAR_GAIN = 1.80
ROTATE_IN_PLACE_THRESHOLD_RAD = 1.0  # threshold to slow linear for large heading error

# ── Arduino serial ────────────────────────────────────────────────────────────
SERIAL_BAUD = 115200
ARDUINO_SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_CANDIDATES = [
    "/dev/ttyUSB0",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB1",
]

# ── Camera ───────────────────────────────────────────────────────────────────
CAMERA_DEVICES: dict = {
    "front": "/dev/video0",
    "rear":  "/dev/video2",
}
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
JPEG_QUALITY = 80
SNAPSHOT_SECONDS = 10
SNAPSHOT_RETENTION_SECONDS = 300

# ── Lidar (YDLIDAR X2) ───────────────────────────────────────────────────────
LIDAR_SERIAL_PORT = "/dev/ttyUSB1"
LIDAR_BAUDRATE = 115200
LIDAR_SCAN_FREQUENCY = 6.0
LIDAR_SAMPLE_RATE = 3
LIDAR_SINGLE_CHANNEL = True
LIDAR_MIN_RANGE_M = 0.12
LIDAR_MAX_RANGE_M = 8.0
LIDAR_RENDER_MAX_POINTS = 720   # Full scan density for radar visualization

# ── Occupancy map ─────────────────────────────────────────────────────────────
# 10,000 sq.ft ≈ 929 m².  At 0.02 m/cell (2cm) → high detail mapping.
# We use 2400 × 2400 cells → 48 m × 48 m = 2304 m² ≈ 24,800 sq.ft.
MAP_RESOLUTION_M = 0.02     # metres per cell (2cm for high detail)
MAP_SIZE_CELLS = 2400       # grid is MAP_SIZE_CELLS × MAP_SIZE_CELLS (48 m × 48 m)
MAP_ORIGIN_X_M = -24.0      # world X coordinate of cell (0,0)
MAP_ORIGIN_Y_M = -24.0      # world Y coordinate of cell (0,0)
FREE_HIT = -0.5             # log-odds decrement for free cells (moderate stability)
OCCUPIED_HIT = 2.0          # log-odds increment for occupied cells
LOG_ODDS_MIN = -5.0
LOG_ODDS_MAX = 5.0
MAP_OCCUPIED_THRESHOLD = 0.68  # Occupied threshold (higher = fewer false walls)
MAP_FREE_THRESHOLD = 0.32      # Free threshold (lower = more stable free space)
PLANNER_INFLATION_CELLS = 5    # obstacle inflation radius for A* (cells)

# ── LiDAR-based localization (scan-matching ICP) ──────────────────────────────
LIDAR_LOCALIZATION_ENABLED = True   # set False to fall back to pure odometry
LIDAR_ICP_MAX_ITERATIONS   = 50     # More iterations for better convergence
LIDAR_ICP_TOLERANCE_M      = 0.001  # Very strict tolerance for accuracy
LIDAR_ICP_MAX_CORRESP_M    = 0.40   # Correspondence search radius (meters)
LIDAR_ICP_WEIGHT           = 0.80   # Trust ICP heavily for drift correction
LIDAR_ICP_MIN_POINTS       = 15     # Skip ICP if scan has fewer points
LIDAR_ICP_INTERVAL_S       = 0.05   # Run ICP at 20Hz for smooth tracking while moving

# ── POI / Waypoint features ───────────────────────────────────────────────────
POIS_FILE             = DATA_DIR / "pois.json"              # persistent store for points of interest
RUNTIME_SETTINGS_FILE = DATA_DIR / "runtime_settings.json"  # live-tunable params (UI → disk)

# ── Web server ────────────────────────────────────────────────────────────────
WEB_PORT = 8080
STATUS_POLL_MS = 450
MAP_POLL_MS = 900
LIDAR_POLL_MS = 260
