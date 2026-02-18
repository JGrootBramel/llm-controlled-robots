#!/usr/bin/env bash

set -euo pipefail

# Build and launch helper for robot-side setup after project updates.
# Matches docs/tooling/robot_launch_runbook.md steps.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
VENV_DIR="${REPO_ROOT}/venv"
CATKIN_DIR="${REPO_ROOT}/catkin_ws"
REQ_FILE="${REPO_ROOT}/requirements-robot.txt"
ROS_SETUP="/opt/ros/noetic/setup.bash"

DO_INSTALL=true
DO_BUILD=true
DO_LAUNCH=true
PIP_UPGRADE=false

usage() {
  cat <<'EOF'
Usage: ./robot_build_and_launch.sh [options] [-- roslaunch_args...]

Options:
  --skip-install   Skip pip install -r requirements-robot.txt
  --skip-build     Skip catkin_make
  --no-launch      Prepare environment and build, but do not roslaunch
  --upgrade-pip    Run pip install --upgrade pip before installing requirements
  -h, --help       Show this help

Examples:
  ./robot_build_and_launch.sh
  ./robot_build_and_launch.sh --no-launch
  ./robot_build_and_launch.sh -- --screen
EOF
}

LAUNCH_ARGS=()
while (($# > 0)); do
  case "$1" in
    --skip-install)
      DO_INSTALL=false
      shift
      ;;
    --skip-build)
      DO_BUILD=false
      shift
      ;;
    --no-launch)
      DO_LAUNCH=false
      shift
      ;;
    --upgrade-pip)
      PIP_UPGRADE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  echo "Install ROS Noetic or update ROS_SETUP in this script." >&2
  exit 1
fi

if [[ ! -d "${CATKIN_DIR}" ]]; then
  echo "catkin workspace not found: ${CATKIN_DIR}" >&2
  exit 1
fi

if [[ ! -f "${REQ_FILE}" ]]; then
  echo "requirements file not found: ${REQ_FILE}" >&2
  exit 1
fi

echo "=================================================="
echo "Robot build + launch"
echo "Repo: ${REPO_ROOT}"
echo "Install deps: ${DO_INSTALL}"
echo "Build catkin: ${DO_BUILD}"
echo "Launch stack: ${DO_LAUNCH}"
echo "=================================================="

cd "${REPO_ROOT}"

if [[ ! -d "${VENV_DIR}" ]]; then
  echo "[1/4] Creating Python virtual environment..."
  python3 -m venv "${VENV_DIR}"
else
  echo "[1/4] Reusing existing Python virtual environment..."
fi

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

if [[ "${DO_INSTALL}" == "true" ]]; then
  echo "[2/4] Installing robot Python requirements..."
  if [[ "${PIP_UPGRADE}" == "true" ]]; then
    pip install --upgrade pip
  fi
  pip install -r "${REQ_FILE}"
else
  echo "[2/4] Skipping Python dependency install."
fi

echo "[3/4] Sourcing ROS environment..."
# shellcheck source=/dev/null
source "${ROS_SETUP}"

if [[ "${DO_BUILD}" == "true" ]]; then
  echo "[4/4] Building catkin workspace..."
  cd "${CATKIN_DIR}"
  catkin_make
else
  echo "[4/4] Skipping catkin build."
  cd "${CATKIN_DIR}"
fi

# shellcheck source=/dev/null
source "${CATKIN_DIR}/devel/setup.bash"

cd "${REPO_ROOT}"

echo
echo "Environment prepared."
echo "venv: ${VENV_DIR}"
echo "catkin: ${CATKIN_DIR}"
echo

if [[ "${DO_LAUNCH}" == "true" ]]; then
  echo "Launching robot stack: roslaunch limo_rosa_bridge rosa_bridge.launch ${LAUNCH_ARGS[*]-}"
  exec roslaunch limo_rosa_bridge rosa_bridge.launch "${LAUNCH_ARGS[@]}"
fi

echo "Launch skipped (--no-launch)."
