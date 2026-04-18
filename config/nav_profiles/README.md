# Navigation Profiles (YAML — Nav2 compatible)

Each `.yaml` file in this directory is a **navigation profile** — a named
collection of runtime-tunable parameters that configure every layer of the
Arespath navigation stack simultaneously, analogous to ROS2 Nav2's
`nav2_params.yaml`.

## File format

```yaml
profile:
  name: my_profile          # must match the filename stem
  description: "One line."  # shown in the Web UI profile selector

localization_server:
  ros__parameters:
    lidar_icp_weight: 0.80

planner_server:
  ros__parameters:
    obstacle_stop_distance_m: 0.35
    nav_replan_seconds: 1.5
    ...

controller_server:
  ros__parameters:
    max_linear_mps: 0.40
    kp_angular: 2.5
    ...

map_server:
  ros__parameters:
    web_map_scan_points: 120
    map_image_push_interval_s: 0.75
```

## Sections → Nodes

| YAML section          | Runtime node                 | What it tunes                             |
|-----------------------|------------------------------|-------------------------------------------|
| `localization_server` | `LocalizationLifecycleNode`  | ICP weight, map anchoring                 |
| `planner_server`      | `GlobalPlannerNode`          | JPS, obstacle avoidance, replan rate      |
| `controller_server`   | `LocalControllerNode`        | speed, PID gains, lookahead               |
| `map_server`          | `_map_push_loop` / renderer  | WebSocket push rate, scan point count     |

## Applying a profile

### Web UI
Select from the **Profile** dropdown in the Settings panel → click **Apply**.

### Python
```python
from app.robot.runtime_cfg import rtcfg
applied = rtcfg.apply_profile("smooth_indoor")
```

### REST API
```
POST /api/settings/profile
{"name": "explore_fast"}
```

## Bundled profiles

| Profile              | Use case                                  | Max speed |
|----------------------|-------------------------------------------|-----------|
| `balanced`           | General indoor/outdoor (default)          | 0.40 m/s  |
| `smooth_indoor`      | Tight corridors, gentle turns             | 0.28 m/s  |
| `explore_fast`       | Open areas, lighter UI                    | 0.55 m/s  |
| `aggressive_outdoor` | Fast outdoor runs, reduced UI overhead    | 0.70 m/s  |

## Adding a new profile

1. Copy `balanced.yaml` and rename it (filename = profile name).
2. Edit parameter values — unknown keys are silently ignored.
3. The profile appears in the Web UI immediately (no restart needed).

## Parameter reference

All valid keys are defined in `app/robot/runtime_cfg.py` in the `TUNABLES`
dict.  Each key has `min`, `max`, `default`, `label`, `unit`, and
`description` fields.  Values outside `[min, max]` are clamped on load.
