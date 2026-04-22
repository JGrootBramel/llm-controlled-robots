# Mission: fetch red cubes

End-to-end test task for the LIMO Cobot:

> "Explore the room, find red cubes, pick them up, and deliver them to a
> coordinate."

The orchestration lives on the remote PC as ROSA tools composing the
per-node skill services exposed by the refactored autonomy stack. See
[`node-contracts.md`](../architecture/node-contracts.md) for the full
capability API.

## Tools involved

| Tool | Service / Topic | Purpose |
| - | - | - |
| `enable_red_cube_detector(bool)` | `/red_cube_detector/enable` (SetBool) | Gate perception |
| `start_exploration` / `stop_exploration` | `/exploration_enabled` (SetBool) | Drive frontier loop |
| `reset_exploration` | `/exploration_reset` (Trigger) | Drop current frontier goal |
| `is_red_cube_found` / `get_latest_red_cube` | `/red_cubes/found`, `/red_cubes/latest_pose` | Read detector state |
| `snapshot_red_cube` | `/red_cube_detector/snapshot` (Trigger) | Force a fresh latched pose |
| `approach_object` / `cancel_approach` | `/approach_object/{approach,cancel}` (Trigger) | Drive to standoff |
| `pick_object` / `place_object` / `arm_go_home` | `/arm_control/{pick,place,go_home}` (Trigger) | MyCobot arm |
| `go_to_map_pose(x, y, yaw)` | publishes `/move_base_simple/goal` | Delivery nav |
| `fetch_red_cubes(...)` | convenience: composes everything above | One-call mission |
| `halt_robot` | cancels `/move_base`, zeroes `/cmd_vel`, disables exploration | Safety |

## Example prompts

```
Drive around the room and find red cubes. Bring the first one to
x=0.5, y=0.0, facing along +x, then come back to home.
```

```
Can you pick up the red cube I just put on the floor? Deliver it to
map coordinate (1.2, -0.4) with heading 0 degrees, then return the arm
home.
```

```
Stop! Cancel everything you're doing right now.
```

## Call-trace for `fetch_red_cubes(delivery_x=1.0, delivery_y=0.0)`

```
enable_red_cube_detector(True)
start_exploration()
# …wait for /red_cubes/found == True (up to detection_timeout_s)
stop_exploration()
snapshot_red_cube()
approach_object()
pick_object()
go_to_map_pose(x_m=1.0, y_m=0.0, yaw_deg=0.0)
place_object()
```

## Troubleshooting

- **"Service unavailable" on every call.** The robot's autonomy stack
  isn't running. On the robot: `roslaunch limo_rosa_bridge
  rosa_bridge.launch`.
- **No detection after 90 s of exploration.** Check the detector is
  enabled (`enable_red_cube_detector(True)`), that RGB + depth topics
  are flowing, and that the red cube is inside the HSV thresholds
  (`scripts/_helpers/hsv.py`).
- **`approach_object` always fails with "no reachable standoff".** The
  global costmap has inflated too much around the cube — re-run
  `reset_cam_coverage` + `start_exploration` to expand known space.
- **Arm won't move.** `arm_control_node` may have failed to open the
  serial port; check its log. `arm_go_home` returns a descriptive error.
