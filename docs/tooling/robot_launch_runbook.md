# Robot Launch Runbook

This runbook covers robot-side setup and validation for the hybrid autonomy stack:

- Core stack always-on via launch
- Perception/grasp stacks on-demand via ROSA tools
- On-robot debug video windows for detection nodes

## 1) Build and source on the robot

Run on the robot:

```bash
cd ~/llm-controlled-robots
python3 -m venv venv
source venv/bin/activate
pip install -r requirements-robot.txt

cd ~/llm-controlled-robots/catkin_ws
catkin_make
source devel/setup.bash
```

If you use a virtual environment for Python nodes:

```bash
source ~/llm-controlled-robots/venv/bin/activate
```

## 2) Launch robot stack

Start full bridge + core autonomy:

```bash
roslaunch limo_rosa_bridge rosa_bridge.launch
```

What this should start:

- `limo_bringup` base stack
- `rosbridge_websocket`
- `autonomy_core.launch`:
  - `cam_coverage_node.py`
  - one planner (frontier by default)

## 3) Start ROSA from remote PC

On your remote PC:

```bash
cd ~/llm-controlled-robots
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
source venv/bin/activate
pip install -r requirements-remote.txt

# Solution 1: treat autonomy as launch-managed on robot; ROSA tools only command/query.
export LIMO_AUTONOMY_LAUNCH_MANAGED=true

python src/launch_rosa.py
```

Make sure ROS networking points to robot ROS master (`ROS_MASTER_URI`) and remote host (`ROS_IP`/`ROS_HOSTNAME`) as needed in your environment.

## 4) ROSA validation sequence

In ROSA chat, run these in order:

1. `Get autonomy status.`
2. `Start object finder with prompt "a water bottle", threshold 0.15, show_debug_window true.`
3. `Get autonomy status.`
4. `Update object query to "a blue cube".`
5. `Stop autonomy node object_finder.`
6. `Start blue cube grasper with show_debug_window true.`
7. `Get autonomy status.`

Expected:

- `/object_detection_ready` becomes `true` when object finder is running.
- `/object_found` toggles `true` when detected.
- `/object_pose` gets populated.
- On robot display, debug windows appear:
  - `object_finder_debug`
  - `blue_cube_debug`

## 5) Topic/service checks (robot or remote terminal)

```bash
rostopic echo -n1 /object_detection_ready
rostopic echo -n1 /object_found
rostopic echo -n1 /object_pose
rosservice list | grep cam_coverage/reset
```

## 6) Common troubleshooting

- No debug window appears:
  - Ensure node param `show_debug_window` is `true`.
  - Ensure robot has active display (`DISPLAY=:0` typically).
  - Confirm OpenCV GUI support is installed on robot image.

- No detection though object is visible:
  - Verify camera topics:
    - `/camera/color/image_raw`
    - `/camera/depth/image_raw`
    - `/camera/color/camera_info`
  - Check TF chain includes camera frame to `base_link`.
  - Tune thresholds:
    - object finder `threshold`, `min_hits`
    - blue grasper `h_low`, `h_high`, `s_low`, `v_low`, `min_area_px`

- Grasper window appears but no grasp:
  - Confirm `pymycobot` access, serial port, and permissions.
  - Verify arm reach and frame mapping.

## 7) Stop commands

From ROSA:

- `Stop all autonomy nodes.`

From terminal (if needed):

```bash
rosnode list
rosnode kill /object_finder /blue_cube_grasper /cam_coverage_node /frontier_goal_selector /straight_explore_planner
```

## 8) Minimal success criteria

- Core launch starts without node crashes.
- ROSA can start/stop perception tools on demand.
- `/object_found` and `/object_pose` are published when targets are in view.
- Debug window is visible on robot display while detection node runs.
