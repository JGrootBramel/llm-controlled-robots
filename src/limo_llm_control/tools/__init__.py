"""
ROSA tools (capability wrappers) for LIMO cobot.

Structured per docs/project-structure.md and docs/architecture/architecture-robot-remote.md:
- motion: base velocity /cmd_vel (turn, drive)
- navigation: exploration, coverage, planners
- perception: object detection, finder, blue-cube grasper
- diagnostics: status, stop nodes

All communication uses rospy (native ROS1) per communication-options_robot-remote_pc.md.
"""

from .diagnostics import get_autonomy_status, stop_autonomy_nodes
from .motion import drive_distance, turn_in_place
from .navigation import (
    reset_cam_coverage,
    start_cam_coverage_node,
    start_frontier_planner_node,
    start_straight_planner_node,
)
from .perception import (
    start_blue_cube_grasper_node,
    start_object_finder_node,
    update_object_query,
)

__all__ = [
    "turn_in_place",
    "drive_distance",
    "start_cam_coverage_node",
    "reset_cam_coverage",
    "start_frontier_planner_node",
    "start_straight_planner_node",
    "start_object_finder_node",
    "update_object_query",
    "start_blue_cube_grasper_node",
    "stop_autonomy_nodes",
    "get_autonomy_status",
]
