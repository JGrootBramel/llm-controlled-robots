#!/usr/bin/env bash

set -euo pipefail

# Clean, deterministic robot-side start for the full ROSA stack.
# Use this on the robot instead of raw `roslaunch ...` when debugging.

REPO_ROOT="${REPO_ROOT:-$HOME/llm-controlled-robots}"
AGILEX_SETUP="${AGILEX_SETUP:-$HOME/agilex_ws/devel/setup.bash}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
MAP_FILE="${1:-}"

echo "[start_robot_rosa_full_clean] repo=${REPO_ROOT}"

if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
else
  echo "ERROR: Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ -f "${AGILEX_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${AGILEX_SETUP}"
else
  echo "ERROR: Missing AgileX setup: ${AGILEX_SETUP}" >&2
  exit 1
fi

cd "${REPO_ROOT}/catkin_ws"
if [[ ! -f "devel/setup.bash" ]]; then
  echo "ERROR: catkin workspace not built at ${REPO_ROOT}/catkin_ws" >&2
  echo "Run: cd ${REPO_ROOT}/catkin_ws && catkin_make" >&2
  exit 1
fi
# shellcheck source=/dev/null
source devel/setup.bash

echo "[start_robot_rosa_full_clean] stopping conflicting ROS processes..."
pkill -f roslaunch || true
pkill -f rosmaster || true
pkill -f rosout || true
pkill -f rosbridge_websocket || true
pkill -f "python.*rosbridge" || true
sleep 2

echo "[start_robot_rosa_full_clean] starting robot_rosa_full.launch"
if [[ -n "${MAP_FILE}" ]]; then
  exec roslaunch limo_rosa_bridge robot_rosa_full.launch map_file:="${MAP_FILE}"
else
  exec roslaunch limo_rosa_bridge robot_rosa_full.launch
fi
