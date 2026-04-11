# Arespath Rover

A full-stack Raspberry Pi robotics control system combining the proven motor control logic from **motor_web_console** with the SLAM architecture of **arespath_slam_enhanced**, rebuilt cleanly from scratch.

## Features

- **Hold-to-drive controls** — D-pad buttons and WASD keys run motors continuously while held; releasing immediately sends STOP
- **Three fail-safe layers** — Frontend (release/blur), backend queue watchdog, Arduino firmware watchdog (350 ms)
- **Socket.IO real-time transport** — Manual commands travel over WebSocket; REST is an automatic fallback
- **20 Hz control loop** — Odometry, mapping, obstacle detection, and navigation at 20 Hz
- **50 Hz command worker** — DriveCommandQueue serialises commands with priority (manual > nav) and idle watchdog
- **A\* autonomous navigation** — Click a goal on the map to start a mission; auto-replanning every 1.5 s
- **Occupancy grid SLAM** — Bayesian log-odds map built from lidar scans
- **YDLIDAR X2 support** — Real hardware with automatic mock fallback for development
- **Dual MJPEG camera streams** — OpenCV capture with placeholder frames when hardware is absent
- **Arduino Uno + BTS7960** — Full encoder odometry, PWM motor control, TEL telemetry frames

## Project Structure

```
arespath_rover/
├── app/
│   ├── main.py              ← Flask + Socket.IO server entry point
│   ├── config.py            ← All tuneable constants
│   ├── robot/
│   │   ├── control.py       ← RobotRuntime (20 Hz loop + monitor)
│   │   ├── command_bus.py   ← DriveCommandQueue (50 Hz worker)
│   │   ├── serial_bridge.py ← ArduinoBridge (non-blocking serial)
│   │   ├── state.py         ← Dataclasses: RobotState, Pose, NavigationState
│   │   ├── mapping.py       ← OccupancyGridMap (log-odds + Bresenham)
│   │   ├── planner.py       ← A* path planner + inflate_occupancy
│   │   ├── lidar.py         ← LidarManager (YDLIDAR X2 + mock)
│   │   └── camera.py        ← CameraManager (MJPEG + snapshots)
│   └── static/
│       ├── index.html       ← Single-page dashboard
│       ├── app.js           ← Socket.IO client, canvas renders, keyboard
│       └── styles.css       ← Dark mission-themed UI
├── arduino/
│   └── arespath_final/
│       └── arespath_final.ino  ← Arduino Uno firmware
├── docs/
│   └── real_time_control.md   ← Architecture, timing table, code snippets
├── maps/                    ← Saved occupancy maps (auto-created)
├── data/                    ← Runtime data (auto-created)
├── snapshots/               ← Camera snapshots (auto-created)
├── services/
│   └── arespath.service     ← systemd unit for Pi autostart
├── scripts/
│   └── install_pi_os.sh     ← Pi dependency installer
└── requirements.txt
```

## Quick Start

### 1. Install Python dependencies

```bash
pip install -r requirements.txt
```

For camera support:
```bash
pip install opencv-python-headless
```

For real YDLIDAR X2:
```bash
# Install ydlidar-sdk from https://github.com/YDLIDAR/YDLidar-SDK
```

### 2. Flash Arduino

Open `arduino/arespath_final/arespath_final.ino` in Arduino IDE and upload to your Uno.

Verify the serial pinout in the sketch header matches your wiring:
- Left BTS7960: RPWM=5, LPWM=6, R_EN=7, L_EN=8
- Right BTS7960: RPWM=9, LPWM=10, R_EN=11, L_EN=12
- Left encoder: A=2, B=A0
- Right encoder: A=3, B=A1

### 3. Configure

Edit `app/config.py` for your robot geometry and serial ports:

```python
WHEEL_RADIUS_M = 0.050        # your wheel radius
WHEEL_BASE_M   = 0.260        # your wheelbase
ARDUINO_SERIAL_PORT = "/dev/ttyUSB0"
LIDAR_SERIAL_PORT   = "/dev/ttyUSB1"
```

### 4. Run

```bash
python -m app.main
```

Open `http://<pi-ip>:8080` in your browser.

### 5. Autostart on Pi

```bash
sudo cp services/arespath.service /etc/systemd/system/
sudo systemctl enable --now arespath
```

## Driving

- **Arm first** — click "Arm Motors" before driving
- **D-pad** — hold a button to drive; release to stop
- **Keyboard** — hold W/A/S/D to drive; Space = stop; release any key = stop
- **Speed slider** — sets drive speed percentage (5–100%)
- **STOP button** — emergency stop, always works without arming

## Mapping & Navigation

1. Click **Start Map** to begin building the occupancy grid
2. Drive the robot around the space
3. Click **Stop Map** and **Save** when done
4. Switch to **Mission** mode
5. Click a point on the map to set a navigation goal
6. The robot plans an A\* path and drives to the goal

## REST API Reference

| Endpoint | Method | Body | Description |
|----------|--------|------|-------------|
| `/api/status` | GET | — | Full robot status |
| `/api/arm` | POST | `{armed}` | Arm/disarm motors |
| `/api/stop` | POST | — | Emergency stop |
| `/api/manual` | POST | `{linear, angular}` | Manual drive (-1…1) |
| `/api/mode` | POST | `{mode}` | pilot / mission |
| `/api/map/start` | POST | `{clear?}` | Start mapping |
| `/api/map/stop` | POST | — | Stop mapping |
| `/api/map/reset` | POST | — | Clear map |
| `/api/map/save` | POST | `{name}` | Save map |
| `/api/map/load` | POST | `{name}` | Load map |
| `/api/map/list` | GET | — | Saved map names |
| `/api/map/data` | GET | — | Map PNG + overlays |
| `/api/navigate/goal` | POST | `{x, y}` | Set nav goal |
| `/api/navigate/cancel` | POST | — | Cancel navigation |
| `/api/pose` | POST | `{x, y, theta}` | Set robot pose |
| `/api/lidar` | GET | — | Lidar point cloud |
| `/video/<cam>` | GET | — | MJPEG stream |
| `/healthz` | GET | — | Liveness probe |

## Socket.IO Events

| Event (client→server) | Payload | Description |
|----------------------|---------|-------------|
| `manual_command` | `{cmd, speed}` | Drive command (hold-to-drive) |
| `heartbeat` | `{}` | Watchdog reset |

| Event (server→client) | Payload | Description |
|----------------------|---------|-------------|
| `status` | full status dict | On connect |
| `ack` | `{cmd, ok}` | Command acknowledgement |
| `heartbeat_ack` | `{ts}` | Heartbeat response |

## Architecture

See [docs/real_time_control.md](docs/real_time_control.md) for the full architecture diagram, timing table, and annotated code snippets.
