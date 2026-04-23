#!/usr/bin/env bash
# Run ROSA / rosa_agent with ROS Noetic Python bindings visible (tf2, rospy, etc.).
# Default entrypoint is src/rosa_agent.py, which dynamically imports tools from
# src/limo_llm_control/tools via that package's __all__.
#
# Usage (from repo root, after filling in network):
#   export ROS_MASTER_URI=http://<robot_ip>:11311
#   export ROS_IP=<this_pc_ip>
#   ./scripts/run_rosa_with_ros.sh src/rosa_agent.py
#
# Optional: point at your catkin workspace (for custom messages):
#   export LIMO_CATKIN_SETUP=$HOME/llm-controlled-robots/catkin_ws/devel/setup.bash

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/noetic/setup.bash
else
  echo "No /opt/ros/noetic/setup.bash — install ROS Noetic or edit this script." >&2
  exit 1
fi

WS_SETUP="${LIMO_CATKIN_SETUP:-$REPO/catkin_ws/devel/setup.bash}"
if [[ -f "$WS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$WS_SETUP"
fi

export PYTHONPATH="$REPO/src${PYTHONPATH:+:$PYTHONPATH}"
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_DOMAIN_ID

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "Set ROS_MASTER_URI, e.g. export ROS_MASTER_URI=http://192.168.0.105:11311" >&2
  exit 1
fi

if ! python3 - <<'PY'
import tf2_ros

print("tf2_ros: OK (ROSA cube tools use manual transforms; PyKDL not required on this machine)")
PY
then
  echo "tf2_ros import failed. Map-frame cube tools need ROS Noetic Python. Run:" >&2
  echo "  sudo apt install -y ros-noetic-tf2-ros" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

PY="src/rosa_agent.py"
if [[ $# -gt 0 && "$1" != -* ]]; then
  PY="$1"
  shift
fi

if [[ -d "$REPO/venv" ]]; then
  # shellcheck source=/dev/null
  source "$REPO/venv/bin/activate"
  if ! python3 -c "import tf2_ros" 2>/dev/null; then
    echo "venv hides ROS packages. Recreate with: python3 -m venv venv --system-site-packages" >&2
    exit 1
  fi
fi

if [[ "$PY" = /* ]]; then
  TARGET_PY="$PY"
else
  TARGET_PY="$REPO/$PY"
fi

if [[ ! -f "$TARGET_PY" ]]; then
  echo "Python entrypoint not found: $TARGET_PY" >&2
  exit 1
fi

exec python3 "$TARGET_PY" "$@"
