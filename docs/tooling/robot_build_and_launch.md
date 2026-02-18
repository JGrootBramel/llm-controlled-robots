# `robot_build_and_launch.sh`

This document describes the helper script at repo root: `robot_build_and_launch.sh`.

It automates the robot-side preparation steps after project updates:

- create or reuse `venv`
- install `requirements-robot.txt`
- source ROS Noetic
- run `catkin_make` in `catkin_ws`
- source `catkin_ws/devel/setup.bash`
- launch `limo_rosa_bridge/rosa_bridge.launch`

It follows the workflow in `docs/tooling/robot_launch_runbook.md`.

## Location

- Script: `robot_build_and_launch.sh`

## Prerequisites

- Ubuntu/Linux robot with ROS Noetic installed (`/opt/ros/noetic/setup.bash`)
- Repository checked out on robot (expected path is script's parent directory)
- `python3` available for virtual environment creation

## One-time setup

Make the script executable:

```bash
chmod +x robot_build_and_launch.sh
```

## Usage

Run from repo root on the robot:

```bash
./robot_build_and_launch.sh
```

This command prepares the environment and then launches:

```bash
roslaunch limo_rosa_bridge rosa_bridge.launch
```

By default, this launch starts a camera preview window on the robot display via
`autonomy_core.launch` (`image_view`, topic `/camera/color/image_raw`, window `limo_camera_preview`).

## Options

- `--skip-install`  
  Skip `pip install -r requirements-robot.txt`

- `--skip-build`  
  Skip `catkin_make`

- `--no-launch`  
  Prepare/build only; do not run `roslaunch`

- `--upgrade-pip`  
  Run `pip install --upgrade pip` before dependency install

- `--`  
  Pass remaining args directly to `roslaunch`

- `-h`, `--help`  
  Show usage help

## Examples

Prepare and launch everything:

```bash
./robot_build_and_launch.sh
```

Prepare only (no launch):

```bash
./robot_build_and_launch.sh --no-launch
```

Skip dependency install and build, launch only:

```bash
./robot_build_and_launch.sh --skip-install --skip-build
```

Pass launch args:

```bash
./robot_build_and_launch.sh -- --screen
```

Disable camera preview window when launching:

```bash
./robot_build_and_launch.sh -- start_camera_preview:=false
```

Override preview display/topic:

```bash
./robot_build_and_launch.sh -- camera_preview_display:=:0 camera_preview_topic:=/camera/color/image_raw
```

## Notes

- The script is safe to rerun after code updates.
- If launch-managed mode is used for ROSA, perception nodes are expected to be launched on robot side, and this script helps ensure the base robot stack is up and rebuilt.
- If preview does not appear, verify `image_view` is installed on the robot (`ros-noetic-image-view`) and that `DISPLAY`/X11 access is correct.
