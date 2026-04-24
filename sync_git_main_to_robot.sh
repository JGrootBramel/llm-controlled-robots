#!/usr/bin/env bash

set -euo pipefail

# Quick sync script:
# - Push the selected local branch to a temporary branch on the robot
# - Fast-forward the robot's matching branch from that temp branch over SSH
# This avoids "refusing to update checked out branch" on non-bare repos.
#
# Usage:
#   ./sync_git_main_to_robot.sh                  # syncs the current local branch
#   ./sync_git_main_to_robot.sh <branch-name>    # syncs an explicit local branch
#
# The script pushes straight into the robot's on-disk .git over SSH, so the
# robot does NOT need internet access for this to work.

ROBOT_USER="agilex"
ROBOT_HOST="192.168.0.105"
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

# Branch to sync: explicit arg wins, otherwise use the current local branch.
if [[ $# -ge 1 && -n "$1" ]]; then
  LOCAL_BRANCH="$1"
else
  LOCAL_BRANCH="$(git symbolic-ref --quiet --short HEAD || true)"
  if [[ -z "${LOCAL_BRANCH}" ]]; then
    echo "Error: HEAD is detached. Pass an explicit branch name, e.g.:" >&2
    echo "  ./sync_git_main_to_robot.sh my-branch" >&2
    exit 1
  fi
fi

ROBOT_BRANCH="${LOCAL_BRANCH}"
SYNC_BRANCH="sync-from-remote-${LOCAL_BRANCH}"

if ! git show-ref --verify --quiet "refs/heads/${LOCAL_BRANCH}"; then
  echo "Error: local branch '${LOCAL_BRANCH}' does not exist." >&2
  exit 1
fi

echo "Syncing local branch '${LOCAL_BRANCH}' -> robot branch '${ROBOT_BRANCH}'"

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

echo "Pushing '${LOCAL_BRANCH}' to robot temporary branch '${SYNC_BRANCH}'..."
git push "${REMOTE_NAME}" "${LOCAL_BRANCH}:${SYNC_BRANCH}"

echo "Updating robot working tree to '${ROBOT_BRANCH}'..."
# Create the branch on the robot if it doesn't exist yet, otherwise check it
# out and fast-forward. Non-FF updates fail loudly so we don't silently drop
# commits that exist only on the robot.
ssh "${ROBOT_USER}@${ROBOT_HOST}" bash <<EOF
set -euo pipefail
cd '${REMOTE_WORKTREE_PATH}'
if git show-ref --verify --quiet "refs/heads/${ROBOT_BRANCH}"; then
  git checkout '${ROBOT_BRANCH}'
  git merge --ff-only '${SYNC_BRANCH}'
else
  git checkout -b '${ROBOT_BRANCH}' '${SYNC_BRANCH}'
fi
git branch -D '${SYNC_BRANCH}'
EOF

echo "Shutting down ROS, rebuilding catkin_ws, and relaunching robot_rosa_full on robot..."
echo "(robot_rosa_full.launch will run in the foreground. Press Ctrl-C to stop it.)"
ssh -t "${ROBOT_USER}@${ROBOT_HOST}" bash <<EOF
set -e

# Point GUI nodes (RViz) at the robot's local X server so they show up
# on the robot's own monitor. Non-interactive SSH shells don't inherit
# these from the desktop session, so we set them explicitly.
export DISPLAY="\${DISPLAY:-:0}"
export XAUTHORITY="\${XAUTHORITY:-/home/${ROBOT_USER}/.Xauthority}"

# Ensure ROS node/service XML-RPC URIs are reachable from remote clients.
# If ROS_HOSTNAME is left as an unresolvable alias (e.g. "master"), tools can
# see topics/services in the graph but fail to connect to their endpoints.
ROBOT_LAN_IP="\$(hostname -I | awk '{print \$1}')"
if [ -n "\${ROBOT_LAN_IP}" ]; then
  export ROS_IP="\${ROS_IP:-\${ROBOT_LAN_IP}}"
  export ROS_HOSTNAME="\${ROS_HOSTNAME:-\${ROS_IP}}"
  echo "Using robot ROS network identity: ROS_IP=\${ROS_IP} ROS_HOSTNAME=\${ROS_HOSTNAME}"
fi

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
# --force-cmake rewrites devel/setup.bash using the current CMAKE_PREFIX_PATH
# (which now includes agilex_ws). Without this, a cached CMakeCache.txt from
# a previous build would omit agilex_ws from the overlay chain and sourcing
# devel/setup.bash below would wipe limo_bringup out of ROS_PACKAGE_PATH.
catkin_make --force-cmake

# Sourcing our workspace's devel/setup.bash PREPENDS our workspace while
# preserving the agilex_ws + /home/agilex/catkin_ws chain recorded at build
# time. Do NOT re-source agilex_ws after this -- that would wipe our
# workspace back out of ROS_PACKAGE_PATH, because agilex_ws's recorded
# chain was fixed years ago and cannot know about our repo.
source devel/setup.bash

echo "ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}"
echo "Sanity check: locating vendor + our packages..."
rospack find limo_bringup
rospack find limo_rosa_bridge

# echo "Launching robot_rosa_full.launch..."
# exec roslaunch limo_rosa_bridge robot_rosa_full.launch
EOF

echo "Sync complete."
