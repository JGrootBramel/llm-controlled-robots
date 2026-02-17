"""
Perception tools: object detection and grasp workflows (rospy).

Start object-finder / blue-cube nodes and update object query via topics.
Robot-side runs perception; remote calls services/topics per communication-options.
"""

from __future__ import annotations

import rospy
from langchain.tools import tool
from std_msgs.msg import String

from ..ros_clients import ensure_rospy
from . import _node_runner as runner

_QUERY_PUB = None


@tool
def start_object_finder_node(
    prompt: str = "a water bottle",
    threshold: float = 0.15,
    min_hits: int = 3,
    target_frame: str = "map",
    base_frame: str = "base_link",
    publish_debug: bool = True,
    show_debug_window: bool = True,
) -> str:
    """
    Start the RGB-D object detection, approach, and grasp node.

    Wraps object_finder.py: publishes object pose/found flags and runs approach/grasp FSM.

    Args:
        prompt: Text query for detector (e.g. "a blue cube", "a bottle").
        threshold: Detector score threshold [0.01, 0.95].
        min_hits: Consecutive detections to mark object found.
        target_frame: Frame for published object pose.
        base_frame: Robot base frame.
        publish_debug: Publish debug images on /object_debug.
        show_debug_window: Show live OpenCV debug window on robot display.
    """
    if not prompt.strip():
        return "Invalid 'prompt': provide a non-empty object query."
    e1 = runner.validate_float("threshold", threshold, 0.01, 0.95)
    e2 = runner.validate_int("min_hits", min_hits, 1, 50)
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
        "show_debug_window": bool(show_debug_window),
    }
    return runner.spawn_node("object_finder", params)


@tool
def update_object_query(query: str) -> str:
    """
    Update the target object query used by object_finder at runtime (publishes to /object_query).
    """
    global _QUERY_PUB
    ensure_rospy()
    q = query.strip()
    if not q:
        return "Invalid query: provide a non-empty object description."
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
    show_debug_window: bool = True,
) -> str:
    """
    Start the blue-cube specific RGB-D detection and front-grasp node.

    Wraps pick_cube_blue.py.

    Args:
        stable_hits: Consecutive detections before grasp attempt.
        depth_min: Minimum accepted depth (m).
        depth_max: Maximum accepted depth (m).
        min_area_px: Minimum blob area for blue detection.
        base_frame: TF base frame.
        show_debug_window: Show live OpenCV debug window on robot display.
    """
    e1 = runner.validate_int("stable_hits", stable_hits, 1, 100)
    e2 = runner.validate_float("depth_min", depth_min, 0.05, 5.0)
    e3 = runner.validate_float("depth_max", depth_max, 0.1, 10.0)
    e4 = runner.validate_int("min_area_px", min_area_px, 10, 500000)
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
        "show_debug_window": bool(show_debug_window),
    }
    return runner.spawn_node("blue_cube_grasper", params)
