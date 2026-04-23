"""Perception tools: thin clients for the robot-side red cube detector."""

from __future__ import annotations

import socket
from urllib.parse import urlparse

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


def _resolve_host_hint(host: str) -> str:
    if not host:
        return ""
    try:
        socket.gethostbyname(host)
        return ""
    except Exception:
        return (
            f"Peer host '{host}' is not resolvable from this machine. "
            "Set ROS_IP/ROS_HOSTNAME on the robot to its reachable LAN IP before launching ROS."
        )


def _service_connectivity_hint(service_name: str) -> str:
    try:
        master = rospy.get_master()
        code, _, uri = master.lookupService(rospy.get_name(), service_name)
        if code != 1 or not uri:
            return ""
        host = urlparse(uri).hostname or ""
        return _resolve_host_hint(host)
    except Exception:
        return ""


def _topic_connectivity_hint(topic_name: str) -> str:
    try:
        master = rospy.get_master()
        code, _, state = master.getSystemState()
        if code != 1:
            return ""
        pubs = dict(state[0]).get(topic_name, [])
        if not pubs:
            return ""
        # Check first publisher URI host for reachability.
        code, _, node_uri = master.lookupNode(rospy.get_name(), pubs[0])
        if code != 1 or not node_uri:
            return ""
        host = urlparse(node_uri).hostname or ""
        return _resolve_host_hint(host)
    except Exception:
        return ""


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
        hint = _service_connectivity_hint(_ENABLE_SRV)
        base = f"Service '{_ENABLE_SRV}' is unavailable."
        return f"{base} {hint}".strip()
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
        hint = _service_connectivity_hint(_SNAPSHOT_SRV)
        base = f"Service '{_SNAPSHOT_SRV}' is unavailable."
        return f"{base} {hint}".strip()
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
    """Return whether the detector is currently reporting a red cube.

    Cross-check both ``/red_cubes/found`` and ``/red_cubes/latest_pose``.
    We treat a received pose as a positive signal even when ``found`` is
    currently false (common with brief detector flicker while a valid pose
    remains available to downstream consumers).
    """
    ensure_rospy()
    timeout = max(0.05, float(timeout_s))
    found_msg = None
    pose_msg = None

    try:
        found_msg = rospy.wait_for_message(_FOUND_TOPIC, Bool, timeout=timeout)
    except Exception:
        found_msg = None

    try:
        pose_msg = rospy.wait_for_message(_LATEST_POSE_TOPIC, PoseStamped, timeout=timeout)
    except Exception:
        pose_msg = None

    if found_msg is not None and bool(found_msg.data):
        return "found=True"
    if pose_msg is not None:
        return "found=True (latest_pose available)"
    if found_msg is not None:
        return "found=False"
    hint = _topic_connectivity_hint(_LATEST_POSE_TOPIC)
    base = "found=unknown (no message yet)"
    return f"{base}. {hint}".strip()
