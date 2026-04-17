# ROS-Like Python Navigation Stack

This project now uses a clearer node-style split while keeping the existing Flask and Socket.IO UI unchanged.

## Structure

```text
app/
  main.py                     # web API and Socket.IO only
  robot/
    control.py               # runtime orchestrator
    messages.py              # typed message contracts between modules
    ipc.py                   # latest-only mailboxes, ROS-topic-like delivery
    sensor_fusion.py         # LiDAR/radar fusion interface and obstacle frame
    lidar.py                 # LiDAR driver + timestamped scan buffer
    mapping.py               # occupancy grid / SLAM-lite map store
    planner_adv.py           # global planner (JPS with A* fallback)
    trajectory.py            # local planner smoothing + pure pursuit tracker
    command_bus.py           # motor command serializer
    serial_bridge.py         # hardware transport to Arduino
```

## Node Model

The runtime behaves like a compact ROS graph implemented with Python threads:

1. `lidar.py`
Publishes timestamped `RangeScanFrame` samples.

2. `sensor_fusion.py`
Consumes LiDAR and optional radar detections, outputs `FusedSensorFrame` with obstacle state.

3. `mapping.py`
Consumes pose + scan updates on a latest-only mailbox and updates the occupancy grid without blocking control.

4. `planner_adv.py`
Consumes `PlannerRequest` snapshots so planning runs on a stable map/pose/scan view instead of mutable shared state.

5. `trajectory.py`
Acts like the local planner/controller pair: smooths the global path, generates velocity targets, and tracks them with pure pursuit + PID heading control.

6. `command_bus.py` + `serial_bridge.py`
Act like the motor driver node and hardware interface node.

7. `main.py`
Acts like a UI gateway node. It no longer needs direct control logic.

## IPC Design

`LatestQueue` in `app/robot/ipc.py` intentionally drops stale work. That mirrors ROS systems that prefer fresh sensor and plan data over queued latency.

- Mapping queue: latest pose/scan pair only
- Planner queue: latest planning request only
- Web UI: Socket.IO push remains decoupled from control loop timing

## Real-Time Behavior

- Control loop stays at 50 Hz
- Planner runs asynchronously in its own worker thread
- Map updates run asynchronously in their own worker thread
- Sensor frames carry timestamps and sequence counters
- Path tracking uses denser trajectory sampling and monotonic lookahead progression to reduce oscillation and jitter

## Why This Feels More Like ROS

- Stable message contracts instead of ad-hoc shared reads
- Separate worker ownership for sensing, mapping, planning, control, and UI
- Latest-message semantics to avoid backlog lag on Raspberry Pi
- Decoupled UI transport so mission mode keeps running even if the browser is slow
