#!/usr/bin/env bash

set -euo pipefail

# One-shot: SSH to robot, fetch/pull current branch, start test_map.launch in background,
# then on this PC source env and start ROSA remote (launch_rosa.py).
# See docs/tooling/robot_launch_runbook.md.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
VENV_DIR="${REPO_ROOT}/venv"
CATKIN_DIR="${REPO_ROOT}/catkin_ws"

# Robot SSH config (same env vars as limo_llm_control.tools._node_runner)
ROBOT_HOST="${LIMO_ROBOT_HOST:-}"
ROBOT_USER="${LIMO_ROBOT_USER:-agilex}"
ROBOT_PORT="${LIMO_ROBOT_PORT:-22}"
ROBOT_WORKDIR="${LIMO_ROBOT_WORKDIR:-~/llm-controlled-robots}"

ROBOT_LAUNCH_LOG="robot_launch.log"
ROSCORE_WAIT_MAX=60
ROSCORE_WAIT_INTERVAL=3

usage() {
  cat <<'EOF'
Usage: ./sync_robot_and_start_remote.sh

Syncs the current git branch on the robot, starts roslaunch limo_rosa_bridge test_map.launch
on the robot in the background, then starts the ROSA remote (python src/launch_rosa.py) on this PC.

Required environment:
  LIMO_ROBOT_HOST    Robot hostname or IP (e.g. 192.168.0.151 or robot.local)

Optional environment (same as ROSA tools):
  LIMO_ROBOT_USER    SSH user (default: agilex)
  LIMO_ROBOT_PORT    SSH port (default: 22)
  LIMO_ROBOT_WORKDIR Repo root on robot (default: ~/llm-controlled-robots)

Example:
  export LIMO_ROBOT_HOST=192.168.0.151
  ./sync_robot_and_start_remote.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "${ROBOT_HOST}" ]]; then
  echo "Error: LIMO_ROBOT_HOST is not set." >&2
  usage >&2
  exit 1
fi

if [[ ! -d "${REPO_ROOT}/.git" ]]; then
  echo "Error: Not a git repo: ${REPO_ROOT}" >&2
  exit 1
fi

BRANCH="$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD)"
if [[ -z "${BRANCH}" ]]; then
  echo "Error: Could not resolve current branch." >&2
  exit 1
fi

if [[ -n "${ROBOT_USER}" ]]; then
  SSH_TARGET="${ROBOT_USER}@${ROBOT_HOST}"
else
  SSH_TARGET="${ROBOT_HOST}"
fi
SSH_OPTS=(-o ConnectTimeout=10)
if [[ "${ROBOT_PORT}" != "22" ]]; then
  SSH_OPTS+=(-p "${ROBOT_PORT}")
fi

echo "=================================================="
echo "Sync robot and start remote"
echo "Robot: ${SSH_TARGET} (port ${ROBOT_PORT})"
echo "Workdir: ${ROBOT_WORKDIR}"
echo "Branch: ${BRANCH}"
echo "=================================================="

echo "[1/4] Fetch and pull on robot..."
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "cd ${ROBOT_WORKDIR} && git fetch && git checkout ${BRANCH} && git pull"

echo "[2/4] Starting test_map.launch on robot (background)..."
ssh "${SSH_OPTS[@]}" "${SSH_TARGET}" "cd ${ROBOT_WORKDIR} && nohup bash -c 'source /opt/ros/noetic/setup.bash && source catkin_ws/devel/setup.bash && roslaunch limo_rosa_bridge test_map.launch' > ${ROBOT_LAUNCH_LOG} 2>&1 &"
echo "      Robot launch log: ${ROBOT_WORKDIR}/${ROBOT_LAUNCH_LOG}"
echo "      To stop later: SSH to robot and kill the roslaunch process (or run 'Stop all autonomy nodes' from ROSA)."

echo "[3/4] Waiting for robot roscore (up to ${ROSCORE_WAIT_MAX}s)..."
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Error: ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${CATKIN_DIR}/devel/setup.bash" ]]; then
  echo "Error: Catkin workspace not built: ${CATKIN_DIR}/devel/setup.bash" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${CATKIN_DIR}/devel/setup.bash"
export ROS_MASTER_URI="http://${ROBOT_HOST}:11311"
waited=0
while [[ ${waited} -lt ${ROSCORE_WAIT_MAX} ]]; do
  if rostopic list &>/dev/null; then
    echo "      roscore is up."
    break
  fi
  sleep "${ROSCORE_WAIT_INTERVAL}"
  waited=$((waited + ROSCORE_WAIT_INTERVAL))
done
if [[ ${waited} -ge ${ROSCORE_WAIT_MAX} ]]; then
  echo "      Warning: roscore did not respond in time. Starting remote anyway; you may need to retry or wait." >&2
fi

echo "[4/4] Starting ROSA remote on this PC..."
if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Error: venv not found: ${VENV_DIR}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

export LIMO_AUTONOMY_LAUNCH_MANAGED=true
export ROS_MASTER_URI="http://${ROBOT_HOST}:11311"

cd "${REPO_ROOT}"
exec python src/launch_rosa.py
