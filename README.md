# Arespath Rover — V1 Smooth ROS Merge

This package upgrades the original V1 rover project with the stronger V2 navigation and mapping stack while keeping the same overall web-console workflow.

## What changed

- Correct rover footprint using the V2 chassis values
  - Rover length: **0.520 m**
  - Rover width: **0.500 m**
  - Wheel base / track: **0.400 m**
- ROS-like navigation flow from V2
  - ICP-assisted localization
  - Global planner worker
  - Local controller node
  - Smoothed path tracking
  - Runtime nav profiles
- Smoother web map behavior from V2
  - Cached map image rendering
  - Scaled map fetches for pan/zoom
  - Better mission overlays and multi-waypoint handling
- Large-area default mapping setup
  - **10,000 sq ft** coverage target
  - **250 × 250** occupancy grid
  - **0.12192 m/cell** resolution

## Project layout

```
arespath_v1_smooth_ros/
├── app/
├── arduino/
├── config/nav_profiles/
├── docs/
├── services/
├── scripts/
├── tests/
├── MERGE_NOTES.md
└── requirements.txt
```

## Start

```bash
pip install -r requirements.txt
python -m app.main
```

Then open:

```
http://<pi-ip>:8080
```

## Mapping defaults

The default map is centered around the robot start area and sized for a roughly square **100 ft × 100 ft** space:

- `MAP_SIZE_CELLS = 250`
- `MAP_RESOLUTION_M = 0.12192`
- `MAP_ORIGIN_X_M = -15.24`
- `MAP_ORIGIN_Y_M = -15.24`

## Recommended profile

The default runtime navigation profile is:

- `large_area_250`

That profile is tuned for smoother motion and lighter web-map updates on a large indoor map.

## Run tests

```bash
python -m unittest tests/test_regressions.py
```
