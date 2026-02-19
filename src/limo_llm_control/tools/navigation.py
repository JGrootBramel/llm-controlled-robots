"""
Navigation tools: exploration and coverage (rospy).

Start/stop nodes that publish goals to move_base and manage camera coverage.
Robot-side runs move_base and perception; remote calls capabilities and services.
"""

from __future__ import annotations
import subprocess
from time import time
import os
import threading

from langchain.tools import tool

from . import _node_runner as runner

# Global variable to keep track of the timer so we can cancel it if needed
_exploration_timer = None

def _stop_exploration_callback():
    """Internal function to stop the robot when time is up."""
    # 1. Cancel the current goal so the robot stops moving immediately
    os.system("rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' > /dev/null 2>&1")
    
    # 2. Kill the planner node on the robot via the network
    # This stops the 'got new plan' loop
    os.system("rosnode kill /frontier_goal_selector > /dev/null 2>&1")
    print("Autonomy Stop: Timer expired or manual stop triggered.")

@tool
def start_cam_coverage_node(
    range_m: float = 1.5,
    map_topic: str = "/map",
    coverage_topic: str = "/cam_coverage",
    frame_camera: str = "camera_link",
    camera_info_topic: str = "/camera/color/camera_info",
) -> str:
    """
    Start the camera coverage mapping node (raycasts camera FOV into seen/unseen grid).

    Wraps cam_coverage.py. Publishes OccupancyGrid with seen=100, unseen=-1.

    Args:
        range_m: Max raycast range in meters (0.1..10.0).
        map_topic: Static/SLAM map topic.
        coverage_topic: Output topic for accumulated camera coverage.
        frame_camera: TF camera frame for ray origins.
        camera_info_topic: CameraInfo for FOV.
    """
    err = runner.validate_float("range_m", range_m, 0.1, 10.0)
    if err:
        return err
    if not map_topic.strip() or not coverage_topic.strip() or not frame_camera.strip():
        return "Invalid topic/frame arguments: empty strings are not allowed."
    params = {
        "range_m": float(range_m),
        "map_topic": map_topic.strip(),
        "coverage_topic": coverage_topic.strip(),
        "frame_camera": frame_camera.strip(),
        "camera_info_topic": camera_info_topic.strip(),
    }
    return runner.spawn_node("cam_coverage", params)


@tool
def reset_cam_coverage() -> str:
    """
    Reset the accumulated camera coverage map (calls /cam_coverage/reset via rospy).
    """
    from ..ros_clients import ensure_rospy

    ensure_rospy()
    return runner.call_reset_cam_coverage()


@tool
def start_frontier_planner_node(
    coverage_topic: str = "/cam_coverage",
    goal_topic: str = "/move_base_simple/goal",
    global_costmap_topic: str = "/move_base/global_costmap/costmap",
    min_frontier_dist_m: float = 0.3,
    safety_radius_m: float = 0.2,
    strategy: str = "closest",
) -> str:
    """
    Start the frontier-based exploration planner node.

    Wraps frontier_planner.py: selects frontier goals from coverage + costmap.

    Args:
        coverage_topic: Input camera coverage topic.
        goal_topic: Published exploration goal topic.
        global_costmap_topic: move_base global costmap.
        min_frontier_dist_m: Minimum robot-to-frontier distance (m).
        safety_radius_m: Local clearance check radius (m).
        strategy: Candidate ordering strategy.
    """
    err1 = runner.validate_float("min_frontier_dist_m", min_frontier_dist_m, 0.0, 5.0)
    err2 = runner.validate_float("safety_radius_m", safety_radius_m, 0.05, 2.0)
    if err1:
        return err1
    if err2:
        return err2
    if not coverage_topic.strip() or not goal_topic.strip() or not global_costmap_topic.strip():
        return "Invalid topic arguments: empty strings are not allowed."
    params = {
        "coverage_topic": coverage_topic.strip(),
        "goal_topic": goal_topic.strip(),
        "global_costmap_topic": global_costmap_topic.strip(),
        "min_frontier_dist_m": float(min_frontier_dist_m),
        "safety_radius_m": float(safety_radius_m),
        "strategy": strategy.strip() if strategy else "closest",
    }
    return runner.spawn_node("frontier_planner", params)


@tool
def start_straight_planner_node(
    coverage_topic: str = "/cam_coverage",
    goal_topic: str = "/move_base_simple/goal",
    global_costmap_topic: str = "/move_base/global_costmap/costmap",
    replan_period_s: float = 2.0,
    segment_max_len_m: float = 3.0,
    min_forward_free_m: float = 0.4,
    heading_samples: int = 16,
) -> str:
    """
    Start the straight-segment exploration planner node.

    Wraps straight_planner.py: drives straight until blocked, then picks new headings.

    Args:
        coverage_topic: Input camera coverage topic.
        goal_topic: Goal output topic.
        global_costmap_topic: Global costmap topic.
        replan_period_s: Planner timer period (s).
        segment_max_len_m: Max straight segment length (m).
        min_forward_free_m: Required free distance to keep heading (m).
        heading_samples: Number of heading candidates over 360°.
    """
    e1 = runner.validate_float("replan_period_s", replan_period_s, 0.1, 30.0)
    e2 = runner.validate_float("segment_max_len_m", segment_max_len_m, 0.1, 20.0)
    e3 = runner.validate_float("min_forward_free_m", min_forward_free_m, 0.05, 5.0)
    e4 = runner.validate_int("heading_samples", heading_samples, 4, 360)
    for err in (e1, e2, e3, e4):
        if err:
            return err
    params = {
        "coverage_topic": coverage_topic.strip(),
        "goal_topic": goal_topic.strip(),
        "global_costmap_topic": global_costmap_topic.strip(),
        "replan_period": float(replan_period_s),
        "segment_max_len": float(segment_max_len_m),
        "min_forward_free": float(min_forward_free_m),
        "heading_samples": int(heading_samples),
    }
    return runner.spawn_node("straight_planner", params)

@tool
def start_mapping_exploration() -> str:
    """
    Starts the exploration planner directly on the robot via SSH SILENTLY.
    ROSA must use this to start exploration without spamming the terminal.
    """
    workspace_setup = "~/llm-controlled-robots/catkin_ws/devel/setup.bash" 
    
    # Notice the '> /dev/null 2>&1 &' at the very end.
    # This completely mutes the node and runs it in the background!
    ssh_command = [
        "ssh",
        "agilex@192.168.0.105",
        f"bash -c 'source /opt/ros/noetic/setup.bash && source {workspace_setup} && export ROS_MASTER_URI=http://localhost:11311 && rosrun limo_rosa_bridge frontier_planner_node.py _coverage_topic:=/cam_coverage > /dev/null 2>&1 &'"
    ]
    
    # Fire and forget. No logs will come back to the PC terminal.
    subprocess.Popen(ssh_command)
    
    return "Exploration started silently. The robot is moving. Terminal remains clean."