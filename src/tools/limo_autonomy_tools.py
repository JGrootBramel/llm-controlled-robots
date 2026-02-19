#!/usr/bin/env python3
"""
ROSA tools for LIMO cobot autonomy workflows.

These tools wrap the behavior implemented in:
- cam_coverage.py
- frontier_planner.py
- straight_planner.py
- object_finder.py
- pick_cube_blue.py

The tools start/stop those nodes as subprocesses and provide helper actions
for query updates, coverage resets, and runtime state inspection.
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Literal, Optional
from limo_llm_control.tools import _node_runner as runner
import os
from actionlib_msgs.msg import GoalID

import rospy
from geometry_msgs.msg import PoseStamped
from langchain.tools import tool
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


SRC_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = SRC_DIR / "logs"

_NODE_SCRIPTS: Dict[str, str] = {
    "cam_coverage": "cam_coverage.py",
    "frontier_planner": "frontier_planner.py",
    "straight_planner": "straight_planner.py",
    "object_finder": "object_finder.py",
    "blue_cube_grasper": "pick_cube_blue.py",
}

_PROCESSES: Dict[str, subprocess.Popen] = {}
_PROCESS_META: Dict[str, dict] = {}
_PROCESS_LOG_HANDLES: Dict[str, object] = {}
_LOCK = threading.Lock()
_QUERY_PUB = None

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

def _ensure_rospy() -> None:
    """Initialize a lightweight ROS node context for tool-side helpers."""
    if not rospy.core.is_initialized():
        rospy.init_node("llm_limo_autonomy_tools", anonymous=True)


def _validate_float(name: str, value: float, low: float, high: float) -> Optional[str]:
    """Validate a float range and return an error message, or None if valid."""
    try:
        val = float(value)
    except (TypeError, ValueError):
        return f"Invalid '{name}': expected a numeric value."
    if not (low <= val <= high):
        return f"Invalid '{name}': expected {low} <= {name} <= {high}, got {val}."
    return None


def _validate_int(name: str, value: int, low: int, high: int) -> Optional[str]:
    """Validate an int range and return an error message, or None if valid."""
    try:
        val = int(value)
    except (TypeError, ValueError):
        return f"Invalid '{name}': expected an integer value."
    if not (low <= val <= high):
        return f"Invalid '{name}': expected {low} <= {name} <= {high}, got {val}."
    return None


def _build_ros_private_param_args(params: Dict[str, object]) -> list[str]:
    """Convert {param: value} into ROS private remap args like _param:=value."""
    args = []
    for key, value in params.items():
        args.append(f"_{key}:={value}")
    return args


def _spawn_node(node_key: str, params: Dict[str, object]) -> str:
    """Spawn one autonomy node process, replacing an existing one if already running."""
    if node_key not in _NODE_SCRIPTS:
        return f"Unknown node key '{node_key}'. Valid keys: {sorted(_NODE_SCRIPTS.keys())}."

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    script = SRC_DIR / _NODE_SCRIPTS[node_key]
    if not script.exists():
        return f"Cannot start '{node_key}': script not found at {script}."

    with _LOCK:
        # Stop existing process if still alive.
        old_proc = _PROCESSES.get(node_key)
        if old_proc is not None and old_proc.poll() is None:
            try:
                old_proc.terminate()
                old_proc.wait(timeout=2.0)
            except Exception:
                old_proc.kill()
            finally:
                _PROCESSES.pop(node_key, None)
                _PROCESS_META.pop(node_key, None)
                old_log = _PROCESS_LOG_HANDLES.pop(node_key, None)
                if old_log is not None:
                    try:
                        old_log.close()
                    except Exception:
                        pass

        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = LOG_DIR / f"{node_key}_{stamp}.log"
        log_handle = open(log_path, "a", encoding="utf-8")

        cmd = [sys.executable, str(script)] + _build_ros_private_param_args(params)
        proc = subprocess.Popen(
            cmd,
            cwd=str(SRC_DIR),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )

        _PROCESSES[node_key] = proc
        _PROCESS_LOG_HANDLES[node_key] = log_handle
        _PROCESS_META[node_key] = {
            "pid": proc.pid,
            "started_unix_s": time.time(),
            "script": str(script),
            "params": params,
            "log_file": str(log_path),
            "cmd": cmd,
        }

    # Short health check so we can return quick startup errors.
    time.sleep(0.2)
    if proc.poll() is not None:
        return (
            f"Failed to start '{node_key}' (exit code {proc.returncode}). "
            f"Check log: {log_path}."
        )

    return f"Started '{node_key}' (pid={proc.pid}). Log: {log_path}."


def _stop_node(node_key: str) -> str:
    """Stop one node process if active."""
    with _LOCK:
        proc = _PROCESSES.get(node_key)
        if proc is None:
            return f"Node '{node_key}' is not running."

        try:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

        _PROCESSES.pop(node_key, None)
        meta = _PROCESS_META.pop(node_key, None)
        log_handle = _PROCESS_LOG_HANDLES.pop(node_key, None)
        if log_handle is not None:
            try:
                log_handle.close()
            except Exception:
                pass

    pid = meta["pid"] if meta else "?"
    return f"Stopped '{node_key}' (former pid={pid})."


def _stop_all_nodes() -> str:
    """Stop all managed autonomy node subprocesses."""
    with _LOCK:
        keys = list(_PROCESSES.keys())
    if not keys:
        return "No managed autonomy nodes are currently running."

    results = [_stop_node(k) for k in keys]
    return " | ".join(results)


@tool
def start_cam_coverage_node(
    range_m: float = 1.5,
    map_topic: str = "/map",
    coverage_topic: str = "/cam_coverage",
    frame_camera: str = "camera_link",
    camera_info_topic: str = "/camera/color/camera_info",
) -> str:
    """
    Start the camera coverage mapping node that raycasts camera FOV into a seen/unseen grid.

    This wraps `src/cam_coverage.py`. It continuously publishes an `OccupancyGrid`
    where seen cells are 100 and unseen cells are -1.

    Args:
        range_m: Max raycast range in meters (0.1..10.0).
        map_topic: Static/SLAM map topic.
        coverage_topic: Output topic for accumulated camera coverage.
        frame_camera: TF camera frame name used for ray origins.
        camera_info_topic: CameraInfo topic used to derive FOV.
    """
    err = _validate_float("range_m", range_m, 0.1, 10.0)
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
    return _spawn_node("cam_coverage", params)


@tool
def reset_cam_coverage() -> str:
    """
    Reset the accumulated camera coverage map to unknown values.

    Calls service `/cam_coverage/reset` from `src/cam_coverage.py`.
    """
    _ensure_rospy()
    try:
        rospy.wait_for_service("/cam_coverage/reset", timeout=2.0)
        reset_srv = rospy.ServiceProxy("/cam_coverage/reset", Empty)
        reset_srv()
        return "Coverage reset service call succeeded."
    except Exception as exc:
        return f"Coverage reset failed: {exc}"
    
@tool
def start_exploration_workflow() -> str:
    """
    Starts the complete Mapping-System (Coverage + Frontier Planner).
    """
    res1 = start_cam_coverage_node()
    res2 = start_frontier_planner_node()
    return f"Mapping started: {res1} and {res2}. You can see the progress in RViz on topic /cam_coverage."


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

    This wraps `src/frontier_planner.py`, which selects frontier goals from
    camera coverage + global costmap and can switch into object-approach mode.

    Args:
        coverage_topic: Input camera coverage topic.
        goal_topic: Published exploration goal topic.
        global_costmap_topic: Global costmap topic from move_base.
        min_frontier_dist_m: Minimum robot-to-frontier distance in meters.
        safety_radius_m: Radius for local clearance checks around goals.
        strategy: Candidate ordering strategy string used by the node.
    """
    err1 = _validate_float("min_frontier_dist_m", min_frontier_dist_m, 0.0, 5.0)
    err2 = _validate_float("safety_radius_m", safety_radius_m, 0.05, 2.0)
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
    return _spawn_node("frontier_planner", params)


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

    This wraps `src/straight_planner.py`, which drives straight until blocked and
    chooses new headings from coverage gain while checking reachability.

    Args:
        coverage_topic: Input camera coverage topic.
        goal_topic: Goal output topic.
        global_costmap_topic: Global costmap topic.
        replan_period_s: Main planner timer period in seconds.
        segment_max_len_m: Max straight-line segment length.
        min_forward_free_m: Required free distance before keeping current heading.
        heading_samples: Number of heading candidates over 360 degrees.
    """
    e1 = _validate_float("replan_period_s", replan_period_s, 0.1, 30.0)
    e2 = _validate_float("segment_max_len_m", segment_max_len_m, 0.1, 20.0)
    e3 = _validate_float("min_forward_free_m", min_forward_free_m, 0.05, 5.0)
    e4 = _validate_int("heading_samples", heading_samples, 4, 360)
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
    return _spawn_node("straight_planner", params)


@tool
def start_object_finder_node(
    prompt: str = "a water bottle",
    threshold: float = 0.15,
    min_hits: int = 3,
    target_frame: str = "map",
    base_frame: str = "base_link",
    publish_debug: bool = True,
) -> str:
    """
    Start the RGB-D object detection, approach, and grasp node.

    This wraps `src/object_finder.py`, which publishes object pose/found flags
    and runs its own FSM for approach, alignment, close-in, and grasp.

    Args:
        prompt: Text query used by OWL-ViT (e.g., "a blue cube", "a bottle").
        threshold: Detector score threshold in [0.01, 0.95].
        min_hits: Consecutive detections needed to mark object found.
        target_frame: Global frame for published object pose.
        base_frame: Robot base frame.
        publish_debug: Whether to publish debug images on `/object_debug`.
    """
    if not prompt.strip():
        return "Invalid 'prompt': provide a non-empty object query."
    e1 = _validate_float("threshold", threshold, 0.01, 0.95)
    e2 = _validate_int("min_hits", min_hits, 1, 50)
    for err in (e1, e2):
        if err:
            return err

    params = {
        "prompt": prompt.strip(),
        "threshold": float(threshold),
        "min_hits": int(min_hits),
        "target_frame": target_frame.strip(),
        "base_frame": base_frame.strip(),
        "publish_debug": bool(publish_debug),
    }
    return _spawn_node("object_finder", params)


@tool
def update_object_query(query: str) -> str:
    """
    Update the target object query used by `object_finder` at runtime.

    Publishes to `/object_query` (std_msgs/String).
    """
    _ensure_rospy()
    q = query.strip()
    if not q:
        return "Invalid query: provide a non-empty object description."

    global _QUERY_PUB
    if _QUERY_PUB is None:
        _QUERY_PUB = rospy.Publisher("/object_query", String, queue_size=1, latch=False)
        rospy.sleep(0.05)

    try:
        _QUERY_PUB.publish(String(data=q))
        return f"Published new object query to /object_query: '{q}'."
    except Exception as exc:
        return f"Failed to publish object query: {exc}"


@tool
def start_blue_cube_grasper_node(
    stable_hits: int = 3,
    depth_min: float = 0.10,
    depth_max: float = 2.50,
    min_area_px: int = 900,
    base_frame: str = "base_link",
) -> str:
    """
    Start the blue-cube specific RGB-D detection and front-grasp node.

    This wraps `src/pick_cube_blue.py`.

    Args:
        stable_hits: Required consecutive detections before grasp attempt.
        depth_min: Minimum accepted depth in meters.
        depth_max: Maximum accepted depth in meters.
        min_area_px: Minimum blob area for blue detection.
        base_frame: TF base frame for transforms.
    """
    e1 = _validate_int("stable_hits", stable_hits, 1, 100)
    e2 = _validate_float("depth_min", depth_min, 0.05, 5.0)
    e3 = _validate_float("depth_max", depth_max, 0.1, 10.0)
    e4 = _validate_int("min_area_px", min_area_px, 10, 500000)
    for err in (e1, e2, e3, e4):
        if err:
            return err
    if float(depth_max) <= float(depth_min):
        return "Invalid depth bounds: depth_max must be greater than depth_min."

    params = {
        "stable_hits": int(stable_hits),
        "depth_min": float(depth_min),
        "depth_max": float(depth_max),
        "min_area_px": int(min_area_px),
        "base_frame": base_frame.strip(),
    }
    return _spawn_node("blue_cube_grasper", params)


@tool
def stop_autonomy_nodes(node: str = "all") -> str:
    """
    Stops the robot's movement and kills the planner node.
    Use this to pause exploration for picking up cubes.
    """
    global _exploration_timer
    if _exploration_timer is not None:
        _exploration_timer.cancel()
    
    _stop_exploration_callback()
    return "Autonomy stopped. The 'new plan' loop is broken. RViz remains open."


@tool
def get_autonomy_status() -> str:
    """
    Return status for managed autonomy subprocesses and key ROS state topics.

    Includes process PID/aliveness and the latest values from:
    - /object_detection_ready
    - /object_found
    - /object_approached
    - /object_pose (if available)
    """
    _ensure_rospy()

    with _LOCK:
        proc_snapshot = {}
        for node_key, proc in _PROCESSES.items():
            meta = _PROCESS_META.get(node_key, {})
            proc_snapshot[node_key] = {
                "pid": meta.get("pid"),
                "alive": proc.poll() is None,
                "started_unix_s": meta.get("started_unix_s"),
                "log_file": meta.get("log_file"),
            }

    ros_state = {
        "object_detection_ready": None,
        "object_found": None,
        "object_approached": None,
        "object_pose": None,
    }

    try:
        msg = rospy.wait_for_message("/object_detection_ready", Bool, timeout=0.4)
        ros_state["object_detection_ready"] = bool(msg.data)
    except Exception:
        pass

    try:
        msg = rospy.wait_for_message("/object_found", Bool, timeout=0.4)
        ros_state["object_found"] = bool(msg.data)
    except Exception:
        pass

    try:
        msg = rospy.wait_for_message("/object_approached", Bool, timeout=0.4)
        ros_state["object_approached"] = bool(msg.data)
    except Exception:
        pass

    try:
        pose_msg = rospy.wait_for_message("/object_pose", PoseStamped, timeout=0.4)
        ros_state["object_pose"] = {
            "frame_id": pose_msg.header.frame_id,
            "x": float(pose_msg.pose.position.x),
            "y": float(pose_msg.pose.position.y),
            "z": float(pose_msg.pose.position.z),
        }
    except Exception:
        pass

    payload = {"managed_nodes": proc_snapshot, "ros_state": ros_state}
    return json.dumps(payload, ensure_ascii=True)


@tool
def show_camera_feed() -> str:
    """
    Starts a visual camera feed window on the PC using rqt_image_view.
    """
    import subprocess  # Nutze das Standard-Subprocess Modul, NICHT asyncio
    try:
        # Wir starten rqt_image_view und geben das Topic direkt mit
        # 'shell=False' ist sicherer und 'Popen' blockiert ROSA nicht.
        subprocess.Popen(["rqt_image_view", "/camera/color/image_raw"])
        return "Kamera-Feed was opened in a new Window (rqt_image_view)."
    except FileNotFoundError:
        return "Error: 'rqt_image_view' was not found. Please install it with 'sudo apt install ros-noetic-rqt-image-view'."
    except Exception as e:
        return f"Unexpected error while starting the feed: {str(e)}"

@tool
def start_mapping_exploration(duration_minutes: float = 5.0) -> str:
    """
    Since the planner starts automatically with the bridge, this tool
    now manages the 'active window' for exploration.
    """
    global _exploration_timer
    
    # If the node was killed previously, we need to restart it on the robot
    # We use a network command to 'rosrun' it back into existence
    os.system("rosrun limo_rosa_bridge frontier_planner_node.py _coverage_topic:=/cam_coverage &")
    
    # Start the timer to kill it later
    if _exploration_timer is not None:
        _exploration_timer.cancel()
        
    seconds = duration_minutes * 60
    _exploration_timer = threading.Timer(seconds, _stop_exploration_callback)
    _exploration_timer.start()
    
    return f"Exploration is active. I will automatically kill the planner in {duration_minutes} minutes."