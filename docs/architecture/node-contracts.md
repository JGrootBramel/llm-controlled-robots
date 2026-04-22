# Node contracts — limo_rosa_bridge

One section per refactored node. This is the single source of truth for
the capability API exposed by the robot to ROSA and any other client.

All nodes live in
[`catkin_ws/src/limo_rosa_bridge/scripts/`](../../catkin_ws/src/limo_rosa_bridge/scripts)
and are launched from the three autonomy launch files in
[`catkin_ws/src/limo_rosa_bridge/launch/`](../../catkin_ws/src/limo_rosa_bridge/launch).

Guiding rules (also in the refactor plan):

- **One node = one concern.** No detection + motion + arm in the same
  process.
- **No new `.msg` / `.srv` / `.action` files.** We use
  `geometry_msgs/PoseStamped`, `std_msgs/Bool`, `std_srvs/Trigger`,
  `std_srvs/SetBool`, and the existing `move_base` action.
- Shared pure-Python helpers live in
  [`scripts/_helpers/`](../../catkin_ws/src/limo_rosa_bridge/scripts/_helpers)
  and are importable without ROS installed.

---

## `cam_coverage_node.py`

**Purpose.** Integrate camera viewpoint footprints into a coverage map so
the frontier explorer can tell "have I actually looked there yet?".

| | Topic / Service | Type | Notes |
| - | - | - | - |
| Sub | `<map_topic>` (default `/map`) | `nav_msgs/OccupancyGrid` | Latched map |
| Sub | TF | — | needs `<base_frame>` in `<map_frame>` |
| Pub | `/cam_coverage` | `nav_msgs/OccupancyGrid` | Latched, same grid as map |
| Srv | `/cam_coverage/reset` | `std_srvs/Empty` | Clear the coverage cells |

**Params:** `range_m`, `map_topic`, `map_frame`, `base_frame`, camera HFOV.

**Failure modes:** no `/map` yet → publishes nothing; TF lookup timeout →
log-throttled warning.

---

## `frontier_explorer_node.py`

**Purpose.** Pick the best reachable frontier goal and delegate
navigation to `move_base`. No object detection, no arm, no custom
standoff — those live in `approach_object_node` / `arm_control_node`.

| | Topic / Service | Type | Notes |
| - | - | - | - |
| Sub | `/cam_coverage` | `nav_msgs/OccupancyGrid` | |
| Sub | `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | |
| Pub | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | |
| Srv | `/exploration_enabled` | `std_srvs/SetBool` | Pause / resume |
| Srv | `/exploration_reset` | `std_srvs/Trigger` | Drop current goal, rescore |
| Action client | `/move_base` | `move_base_msgs/MoveBase` | |

**Params:** `enabled_at_start`, scoring weights, goal-spacing metres,
TF frames.

**Failure modes:** no reachable frontier → idles; `move_base` aborts →
node re-scores on next tick.

---

## `red_cube_detector_node.py`

**Purpose.** Perception only. Synchronise RGB, depth and `CameraInfo`;
apply an HSV red mask; compute one `PoseStamped` per blob in the map
frame.

| | Topic / Service | Type | Notes |
| - | - | - | - |
| Sub | `<rgb_topic>` (default `/camera/color/image_raw`) | `sensor_msgs/Image` | |
| Sub | `<depth_topic>` (default `/camera/depth/image_raw`) | `sensor_msgs/Image` | 32FC1 or 16UC1 |
| Sub | `<info_topic>` | `sensor_msgs/CameraInfo` | fx/fy/cx/cy |
| Pub | `/red_cubes/latest_pose` | `geometry_msgs/PoseStamped` | Latched |
| Pub | `/red_cubes/found` | `std_msgs/Bool` | Latched |
| Pub | `/red_cubes/markers` | `visualization_msgs/Marker` | RViz |
| Srv | `~enable` | `std_srvs/SetBool` | Default ON |
| Srv | `~snapshot` | `std_srvs/Trigger` | Republish best-known pose |

**Params:** HSV band overrides, blob area bounds, depth patch size,
`target_frame` (default `map`), `base_frame`.

**Failure modes:** TF missing → pose stays stale, `found` goes False;
no synced frames → nothing published.

---

## `approach_object_node.py`

**Purpose.** Drive the base up to a target pose (e.g. the detected red
cube) and finish with a yaw alignment. Takes over once exploration is
disabled.

| | Topic / Service | Type | Notes |
| - | - | - | - |
| Sub | `<target_topic>` (default `/red_cubes/latest_pose`) | `geometry_msgs/PoseStamped` | Latest is used |
| Pub | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Standoff goal |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Close-in + yaw |
| Pub | `/move_base/cancel` | `actionlib_msgs/GoalID` | on cancel |
| Srv | `~approach` | `std_srvs/Trigger` | Blocks until done |
| Srv | `~cancel` | `std_srvs/Trigger` | Abort in-flight move |

**Params:** `base_frame`, `map_frame`, `standoff`, `min_standoff`,
`approach_speed`, `yaw_tolerance`.

**Failure modes:** no target received → returns "no target"; all
standoff candidates in costmap → returns failure; caller should retry
after exploration discovers more space.

---

## `arm_control_node.py`

**Purpose.** Talk to the MyCobot 280 via `pymycobot`. Hosts the
`base_link` → arm-mm mapping and the workspace clamp. Subscribes to a
target pose but never *moves the base*.

| | Topic / Service | Type | Notes |
| - | - | - | - |
| Sub | `~target_pose` (default remapped from `/red_cubes/latest_pose`) | `geometry_msgs/PoseStamped` | Expected in `<base_frame>` |
| Srv | `~pick` | `std_srvs/Trigger` | Pre-grasp → grasp → lift |
| Srv | `~place` | `std_srvs/Trigger` | Place at tray + go home |
| Srv | `~go_home` | `std_srvs/Trigger` | Tuck arm |

**Params:** `port`, `baud`, `base_frame`, pre-grasp offsets, grasp RX/RY/RZ.

**Failure modes:** arm not connected → services return success=False with
a message; target stale → `~pick` returns failure without moving.

---

## Mission composition (remote side)

No robot-side orchestrator. The `fetch_red_cubes` ROSA tool
(`src/limo_llm_control/tools/mission.py`) sequences:

1. `/red_cube_detector/enable` → True
2. `/exploration_enabled` → True
3. Wait for `/red_cubes/found == True`
4. `/exploration_enabled` → False
5. `/red_cube_detector/snapshot` (re-latches pose)
6. `/approach_object/approach`
7. `/arm_control/pick`
8. `/move_base_simple/goal` (delivery pose)
9. `/arm_control/place`

Any step can be invoked individually — the mission tool is a convenience,
not a gate.
