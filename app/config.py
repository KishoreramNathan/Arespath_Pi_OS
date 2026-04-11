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
WHEEL_RADIUS_M = 0.050        # metres — measure your real wheel
WHEEL_BASE_M = 0.260          # centre-to-centre distance between wheels
TICKS_PER_WHEEL_REV = 420.0   # encoder counts per full wheel revolution

# ── Drive limits ──────────────────────────────────────────────────────────────
MAX_PWM = 180                  # maximum PWM value sent to Arduino
MAX_LINEAR_MPS = 0.40          # max forward speed in m/s (normalised 1.0 → this)
MAX_ANGULAR_RADPS = 1.5        # max yaw rate in rad/s
OBSTACLE_STOP_DISTANCE_M = 0.35  # lidar forward-cone stop threshold
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
LIDAR_RENDER_MAX_POINTS = 360

# ── Occupancy map ─────────────────────────────────────────────────────────────
MAP_RESOLUTION_M = 0.05     # metres per cell
MAP_SIZE_CELLS = 400        # grid is MAP_SIZE_CELLS × MAP_SIZE_CELLS
MAP_ORIGIN_X_M = -10.0      # world X coordinate of cell (0,0)
MAP_ORIGIN_Y_M = -10.0      # world Y coordinate of cell (0,0)
FREE_HIT = -1.0             # log-odds decrement for free cells
OCCUPIED_HIT = 3.0          # log-odds increment for occupied cells
LOG_ODDS_MIN = -5.0
LOG_ODDS_MAX = 5.0
MAP_OCCUPIED_THRESHOLD = 0.65
MAP_FREE_THRESHOLD = 0.35
PLANNER_INFLATION_CELLS = 2  # obstacle inflation radius for A*

# ── Web server ────────────────────────────────────────────────────────────────
WEB_PORT = 8080
STATUS_POLL_MS = 450
MAP_POLL_MS = 900
LIDAR_POLL_MS = 260
