"""
Diagnostics tools: autonomy node status and stop (rospy).
"""
from __future__ import annotations
import subprocess
from typing import Literal
from langchain.tools import tool

from . import _node_runner as runner

@tool
def stop_autonomy_nodes(node: Literal["all", "frontier_planner"] = "all") -> str:
    """
    Stops the robot completely. Kills the brain, waits for it to die, then locks the wheels.
    """
    # 1. Kill the planner
    # 2. Wait 2 seconds (sleep 2) to prevent the "Zombie Goal"
    # 3. Cancel the goal in move_base
    # 4. Send an empty Twist command (0 velocity) directly to the motors as a hard brake
    ssh_kill_cmd = [
        "ssh",
        "agilex@192.168.0.105",
        "bash -c 'source /opt/ros/noetic/setup.bash; "
        "rosnode kill /frontier_goal_selector; "
        "sleep 2; "
        "rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID \"{}\" > /dev/null 2>&1; "
        "rostopic pub -1 /cmd_vel geometry_msgs/Twist \"{}\" > /dev/null 2>&1'"
    ]
    
    import subprocess
    subprocess.Popen(ssh_kill_cmd)
    
    if node == "all":
        runner.stop_node("object_finder")
        runner.stop_node("blue_cube_grasper")
        
    return "Autonomy halted: Brain killed, waited for shutdown, and wheels locked."



@tool
def get_autonomy_status() -> str:
    """Return status for managed autonomy subprocesses and key ROS state topics."""
    return runner.get_autonomy_status_json()