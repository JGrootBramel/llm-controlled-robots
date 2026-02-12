"""
Diagnostics tools: autonomy node status and stop (rospy).

Query managed process state and ROS object topics; stop one or all autonomy nodes.
"""

from __future__ import annotations

from typing import Literal

from langchain.tools import tool

from . import _node_runner as runner


@tool
def stop_autonomy_nodes(
    node: Literal[
        "all",
        "cam_coverage",
        "frontier_planner",
        "straight_planner",
        "object_finder",
        "blue_cube_grasper",
    ] = "all",
) -> str:
    """
    Stop one managed autonomy node, or stop all managed autonomy nodes.

    Args:
        node: Which node to stop. Use "all" to stop every managed node.
    """
    if node == "all":
        return runner.stop_all_nodes()
    return runner.stop_node(node)


@tool
def get_autonomy_status() -> str:
    """
    Return status for managed autonomy subprocesses and key ROS state topics.

    Includes process PID/aliveness and latest values from:
    /object_detection_ready, /object_found, /object_approached, /object_pose.
    """
    return runner.get_autonomy_status_json()
