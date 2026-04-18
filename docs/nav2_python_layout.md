# Python Nav2 Layout

The rover now includes a clearer Nav2-like package split:

```text
app/robot/
  tf.py                    # shared frame transforms (base, lidar, world)
  nav2/
    lifecycle.py           # managed-node lifecycle states
    navigator.py           # bt_navigator-style mission coordinator
    waypoint_follower.py   # mission waypoint sequencing
    costmap.py             # planner costmap snapshots
    behavior_tree.py       # wait / reverse / replan recovery decisions
```

## Why This Is Closer To ROS/Nav2

- Shared transform math prevents map, planner, and localization disagreement
- Explicit navigator and waypoint follower mirror Nav2 responsibilities
- Costmap snapshots separate live obstacle overlays from the persistent map
- Lifecycle state makes the navigation server state visible to the UI
