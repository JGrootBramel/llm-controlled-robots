# Mission Testing

This guide validates the red-cube mission flow end-to-end:

1. Explore
2. Detect red cube
3. Approach + pick
4. Return to start/home
5. Place
6. Repeat (up to `max_cubes`)

---

## 1) Prerequisites

- Robot hardware connected (base, lidar, camera, arm)
- ROS network reachable from your test machine
- Repo checked out at:
  - `/home/cps-orin/llm-controlled-robots`

---

## 2) Run Unit Tests First (fast sanity check)

From repo root:

```bash
cd /home/cps-orin/llm-controlled-robots
source /opt/ros/noetic/setup.bash
source venv/bin/activate
export PYTHONPATH="$PWD/src:$PYTHONPATH"
pytest tests/unit/test_rosa_tools_clients.py -q
```

Expected result:

```text
17 passed
```

---

## 3) Start Robot Autonomy Stack

Run on the robot (or wherever robot ROS nodes are launched):

```bash
source /opt/ros/noetic/setup.bash
cd /home/cps-orin/llm-controlled-robots/catkin_ws
source devel/setup.bash
roslaunch limo_rosa_bridge robot_rosa_full.launch
```

Wait until move_base, detector, explorer, and arm services are up.

---

## 4) Start a Mission Test Console

Run on your control machine:

```bash
source /opt/ros/noetic/setup.bash
cd /home/cps-orin/llm-controlled-robots
source venv/bin/activate
python3
```

In Python:

```python
import rospy
rospy.init_node("mission_testing", anonymous=True)

from limo_llm_control.tools.navigation import get_current_map_pose
from limo_llm_control.tools.mission import fetch_red_cubes, fetch_red_cubes_to_start
```

---

## 5) Validate Home Pose Capture

In Python:

```python
fn_pose = get_current_map_pose.func if hasattr(get_current_map_pose, "func") else get_current_map_pose
print(fn_pose())
```

Expected:

- Returns `x=... y=... yaw_deg=...`
- No TF failure messages

If TF fails:

- Ensure localization is running (`map -> base_link` exists)
- Ensure robot launch completed successfully

---

## 6) Dry Run (No Cubes)

This verifies timeout/stop behavior:

```python
fn = fetch_red_cubes_to_start.func if hasattr(fetch_red_cubes_to_start, "func") else fetch_red_cubes_to_start
print(fn(max_cubes=2, detection_timeout_s=10.0))
```

Expected:

- Mission starts
- Exploration toggles on/off
- Ends with message like `no new cube found`
- Does not crash

---

## 7) Single Cube Functional Test

1. Place one red cube in reachable area.
2. Park robot at intended home/drop pose.
3. Run:

```python
print(fn(max_cubes=2, detection_timeout_s=60.0))
```

Expected:

- `snapshot` succeeds
- `approach` succeeds
- `pick` succeeds
- robot navigates back to locked home pose
- `place` succeeds
- summary contains `delivered 1/2` (or `1/1` if you run with `max_cubes=1`)

---

## 8) Two Cube Test (max_cubes=2)

1. Place two red cubes with clear spacing (more than dedup radius, default `0.10m`).
2. Run:

```python
print(fn(max_cubes=2, detection_timeout_s=90.0))
```

Expected:

- Two full cycles if both cubes are found/reachable
- No immediate re-targeting of the same cube location
- summary like `delivered 2/2, processed=2` (or lower delivered if any step failed)

---

## 9) Explicit Home Pose Test (manual coordinates)

If you want fixed map coordinates instead of locking current pose:

```python
fn_fetch = fetch_red_cubes.func if hasattr(fetch_red_cubes, "func") else fetch_red_cubes
print(fn_fetch(
    delivery_x=0.0,
    delivery_y=0.0,
    delivery_yaw_deg=0.0,
    max_cubes=2,
    detection_timeout_s=90.0,
    dedup_radius_m=0.10
))
```

---

## 10) Failure-Path Tests (recommended)

Run each scenario and verify graceful behavior in logs/summary:

- Block path to cube: approach should fail, cycle should skip delivery.
- Force bad grasp (wrong height): pick should fail, no delivery count increment.
- Block return path home: nav-home failure should skip place.

Expected pattern:

- No crashes
- Clear `FAIL` or `Skipping ...` entries
- Accurate final summary `delivered X/Y, processed=Z`

---

## 11) Arm Drop Tuning Test

To tune place behavior at home pose, relaunch with overrides:

```bash
roslaunch limo_rosa_bridge autonomy_manipulation.launch \
  use_custom_arm_angles:=true \
  place_angles:="[-90,-25,-70,7,0,-45]" \
  home_angles:="[-90,0,0,0,0,0]" \
  place_speed:=40 \
  place_gripper_open:=80
```

Then repeat the single-cube test and adjust until drop location is correct.

---

## 12) Quick Troubleshooting Commands

Check critical services:

```bash
rosservice list | rg "exploration_enabled|red_cube_detector|approach_object|arm_control"
```

Check detector outputs:

```bash
rostopic echo -n 1 /red_cubes/found
rostopic echo -n 1 /red_cubes/latest_pose
```

Check TF tree availability:

```bash
rosrun tf tf_echo map base_link
```

---

## 13) Pass Criteria

Mission is considered validated when:

- Unit tests pass
- Home pose capture works reliably
- Dry run exits cleanly on timeout
- One-cube run succeeds end-to-end
- Two-cube run with `max_cubes=2` works without duplicate re-targeting
- Failure paths are handled without crashes and with correct accounting

