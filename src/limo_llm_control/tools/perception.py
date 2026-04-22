"""Perception tools: thin clients for the robot-side red cube detector."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped
from langchain.tools import tool
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from ..ros_clients import ensure_rospy


_ENABLE_SRV = "/red_cube_detector/enable"
_SNAPSHOT_SRV = "/red_cube_detector/snapshot"
_LATEST_POSE_TOPIC = "/red_cubes/latest_pose"
_FOUND_TOPIC = "/red_cubes/found"


@tool
def enable_red_cube_detector(enabled: bool = True) -> str:
    """Turn the robot-side red cube detector on/off.

    Calls ``/red_cube_detector/enable`` (``std_srvs/SetBool``). When
    disabled, the detector stops publishing ``/red_cubes/found`` / pose
    updates. Useful for hand-off between exploration and manipulation.
    """
    ensure_rospy()
    try:
        rospy.wait_for_service(_ENABLE_SRV, timeout=2.0)
    except Exception:
        return f"Service '{_ENABLE_SRV}' is unavailable."
    try:
        proxy = rospy.ServiceProxy(_ENABLE_SRV, SetBool)
        resp = proxy(bool(enabled))
        status = "enabled" if enabled else "disabled"
        return f"red_cube_detector {status}: {resp.message}"
    except Exception as exc:
        return f"Failed to toggle detector: {exc}"


@tool
def snapshot_red_cube() -> str:
    """Force the detector to republish its best-known pose right now.

    Calls ``/red_cube_detector/snapshot`` (``std_srvs/Trigger``). Use this
    after stopping exploration and before calling ``approach_object`` so
    downstream nodes receive a fresh latched pose.
    """
    ensure_rospy()
    try:
        rospy.wait_for_service(_SNAPSHOT_SRV, timeout=2.0)
    except Exception:
        return f"Service '{_SNAPSHOT_SRV}' is unavailable."
    try:
        proxy = rospy.ServiceProxy(_SNAPSHOT_SRV, Trigger)
        resp = proxy()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"snapshot failed: {exc}"


@tool
def get_latest_red_cube(timeout_s: float = 1.0) -> str:
    """Read the last detected red cube pose in map frame.

    Waits up to ``timeout_s`` for a fresh message on
    ``/red_cubes/latest_pose``. Returns a human-readable string; ROSA can
    feed the coordinates back into ``go_to_map_pose`` or the mission tool.
    """
    ensure_rospy()
    try:
        msg = rospy.wait_for_message(_LATEST_POSE_TOPIC, PoseStamped, timeout=float(timeout_s))
    except Exception:
        return f"No pose on {_LATEST_POSE_TOPIC} within {timeout_s:.1f} s."
    x = float(msg.pose.position.x)
    y = float(msg.pose.position.y)
    frame = msg.header.frame_id or "?"
    return f"Last red cube at ({x:.2f}, {y:.2f}) in '{frame}'."


@tool
def is_red_cube_found(timeout_s: float = 0.5) -> str:
    """Return whether the detector is currently reporting a red cube."""
    ensure_rospy()
    try:
        msg = rospy.wait_for_message(_FOUND_TOPIC, Bool, timeout=float(timeout_s))
    except Exception:
        return "found=unknown (no message yet)"
    return "found=True" if msg.data else "found=False"
