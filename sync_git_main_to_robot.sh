#!/usr/bin/env bash

set -euo pipefail

# Quick sync script:
# - Push local main to a temporary branch on the robot
# - Fast-forward robot main from that temp branch over SSH
# This avoids "refusing to update checked out branch" on non-bare repos.

ROBOT_USER="agilex"
ROBOT_HOST="192.168.0.105"
ROBOT_BRANCH="main"
REMOTE_REPO_PATH="/home/agilex/llm-controlled-robots/.git"
REMOTE_WORKTREE_PATH="/home/agilex/llm-controlled-robots"
REMOTE_NAME="robot"
SYNC_BRANCH="sync-from-remote-main"
# AgileX vendor workspace (limo_bringup, astra_camera, limo_base, ...). Our
# catkin_ws overlays this. Non-interactive SSH sessions don't source the
# robot's ~/.bashrc, so we source it explicitly below.
REMOTE_AGILEX_WS_SETUP="/home/agilex/agilex_ws/devel/setup.bash"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "${SCRIPT_DIR}/.git" ]]; then
  echo "Error: run this from inside the git repo." >&2
  exit 1
fi

cd "${SCRIPT_DIR}"

if ! git show-ref --verify --quiet "refs/heads/${ROBOT_BRANCH}"; then
  echo "Error: local branch '${ROBOT_BRANCH}' does not exist." >&2
  exit 1
fi

REMOTE_URL="${ROBOT_USER}@${ROBOT_HOST}:${REMOTE_REPO_PATH}"

if git remote get-url "${REMOTE_NAME}" >/dev/null 2>&1; then
  CURRENT_URL="$(git remote get-url "${REMOTE_NAME}")"
  if [[ "${CURRENT_URL}" != "${REMOTE_URL}" ]]; then
    echo "Updating remote '${REMOTE_NAME}' -> ${REMOTE_URL}"
    git remote set-url "${REMOTE_NAME}" "${REMOTE_URL}"
  fi
else
  echo "Adding remote '${REMOTE_NAME}' -> ${REMOTE_URL}"
  git remote add "${REMOTE_NAME}" "${REMOTE_URL}"
fi

echo "Pushing '${ROBOT_BRANCH}' to robot temporary branch '${SYNC_BRANCH}'..."
git push "${REMOTE_NAME}" "${ROBOT_BRANCH}:${SYNC_BRANCH}"

echo "Updating robot working tree to '${ROBOT_BRANCH}'..."
ssh "${ROBOT_USER}@${ROBOT_HOST}" \
  "cd '${REMOTE_WORKTREE_PATH}' && git checkout '${ROBOT_BRANCH}' && git merge --ff-only '${SYNC_BRANCH}' && git branch -D '${SYNC_BRANCH}'"

echo "Shutting down ROS, rebuilding catkin_ws, and relaunching rosa_bridge on robot..."
echo "(rosa_bridge.launch will run in the foreground. Press Ctrl-C to stop it.)"
ssh -t "${ROBOT_USER}@${ROBOT_HOST}" bash <<EOF
set -e

# Point GUI nodes (RViz) at the robot's local X server so they show up
# on the robot's own monitor. Non-interactive SSH shells don't inherit
# these from the desktop session, so we set them explicitly.
export DISPLAY="\${DISPLAY:-:0}"
export XAUTHORITY="\${XAUTHORITY:-/home/${ROBOT_USER}/.Xauthority}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

# Source the AgileX vendor workspace BEFORE building, so our workspace
# overlays it and 'limo_bringup', 'astra_camera', etc. are resolvable
# inside rosa_bridge.launch. Interactive shells get this from ~/.bashrc,
# but this heredoc runs a non-interactive shell that does not.
if [ -f "${REMOTE_AGILEX_WS_SETUP}" ]; then
  echo "Sourcing AgileX workspace: ${REMOTE_AGILEX_WS_SETUP}"
  source "${REMOTE_AGILEX_WS_SETUP}"
else
  echo "ERROR: AgileX workspace setup not found at ${REMOTE_AGILEX_WS_SETUP}"
  echo "       limo_bringup and other vendor packages will not be found."
  echo "       Edit REMOTE_AGILEX_WS_SETUP in sync_git_main_to_robot.sh."
  exit 1
fi

echo "Stopping any running ROS processes..."
pkill -f roslaunch   2>/dev/null || true
pkill -f rosmaster   2>/dev/null || true
pkill -f rosout      2>/dev/null || true
pkill -f 'python.*ros' 2>/dev/null || true
sleep 2

echo "Building catkin workspace..."
cd '${REMOTE_WORKTREE_PATH}/catkin_ws'
catkin_make
source devel/setup.bash

echo "Launching rosa_bridge.launch..."
exec roslaunch limo_rosa_bridge rosa_bridge.launch
EOF

echo "Sync complete."
