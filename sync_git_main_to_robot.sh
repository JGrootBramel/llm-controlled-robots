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

echo "Sync complete."
