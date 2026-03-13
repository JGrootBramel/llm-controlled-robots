#!/bin/bash
# Wait for robot's roscore to be reachable, then start ROSA (limo_llm_control.main).
# Used by the "remote" service in docker-compose.

set -e

echo "Waiting for ROS master at ${ROS_MASTER_URI:-http://robot:11311}..."
source /opt/ros/noetic/setup.bash
until rostopic list 2>/dev/null; do
  sleep 1
done
echo "ROS master is up."

source /opt/rosa-venv/bin/activate
source /root/catkin_ws/devel/setup.bash
export PYTHONPATH="${PYTHONPATH:-/workspace/llm-controlled-robots/src:/workspace/llm-controlled-robots/src/tools}"

exec python -m limo_llm_control.main
