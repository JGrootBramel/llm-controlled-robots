"""Manipulation tools: trigger pick/place/home on the arm_control_node."""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped
from langchain.tools import tool
from std_srvs.srv import Trigger

from ..ros_clients import ensure_rospy


_PICK_SRV = "/arm_control/pick"
_PICK_VENDOR_SYNC_SRV = "/arm_control/pick_vendor_sync"
_PLACE_SRV = "/arm_control/place"
_HOME_SRV = "/arm_control/go_home"
_APPROACH_SRV = "/approach_object/approach"
_CANCEL_APPROACH_SRV = "/approach_object/cancel"
_TARGET_OVERRIDE_TOPIC = "/arm_control/target_pose_override"


def _trigger(service_name: str) -> str:
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
    except Exception:
        return f"Service '{service_name}' is unavailable."
    try:
        proxy = rospy.ServiceProxy(service_name, Trigger)
        resp = proxy()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"Call to {service_name} failed: {exc}"


@tool
def approach_object() -> str:
    """Drive to the latest red cube and stop at the configured standoff.

    Calls ``/approach_object/approach`` (``std_srvs/Trigger``). The server
    reads the latest ``/red_cubes/latest_pose``, sends a ``move_base``
    goal to a safe standoff, and yaw-aligns on the target.
    """
    return _trigger(_APPROACH_SRV)


@tool
def cancel_approach() -> str:
    """Cancel an in-flight ``approach_object`` call."""
    return _trigger(_CANCEL_APPROACH_SRV)


@tool
def pick_object() -> str:
    """Close the gripper on the latest target pose.

    Calls ``/arm_control/pick`` (``std_srvs/Trigger``). The arm node uses
    whatever was last published on its ``~target_pose`` topic (by default
    the detector's ``/red_cubes/latest_pose``).
    """
    return _trigger(_PICK_SRV)


@tool
def pick_object_vendor_sync() -> str:
    """Pick using pymycobot blocking moves (vendor sync API).

    Same target sources as ``pick_object`` (stream on ``~target_pose`` or
    one-shot ``/arm_control/target_pose_override``). Calls
    ``/arm_control/pick_vendor_sync`` so the arm uses ``sync_send_coords``
    / ``sync_send_angles`` instead of async ``send_coords`` + sleeps.
    Use to compare behavior against the default pick when debugging grasps.
    """
    return _trigger(_PICK_VENDOR_SYNC_SRV)


@tool
def pick_at_pose(
    x_m: float,
    y_m: float,
    z_m: float,
    frame_id: str = "map",
) -> str:
    """Pick at an explicit 3D pose, bypassing the perception stack.

    Publishes a one-shot ``geometry_msgs/PoseStamped`` to
    ``/arm_control/target_pose_override`` and immediately calls
    ``/arm_control/pick``. The arm node consumes the override (so the next
    pick falls back to the detector stream) and transforms the pose into
    ``base_link`` itself, so any TF frame is fine.

    Use this when you already know exactly where the object is — for
    example after the user supplies coordinates, or after a custom
    perception pass. ``z_m`` is required because the cube height varies
    and the arm needs the full 3D position; do not guess it from x/y.

    Tip: drive the base near the target first with ``go_to_map_pose``
    (the arm only reaches ~0.28 m from ``base_link``).
    """
    ensure_rospy()
    frame_id = frame_id.strip() if frame_id else ""
    if not frame_id:
        return "Invalid frame_id: empty string is not allowed."

    pub = rospy.Publisher(_TARGET_OVERRIDE_TOPIC, PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.2)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x_m)
    pose.pose.position.y = float(y_m)
    pose.pose.position.z = float(z_m)
    pose.pose.orientation.w = 1.0
    pub.publish(pose)
    rospy.sleep(0.2)
    pick_result = _trigger(_PICK_SRV)
    return (
        f"Override pose published to {_TARGET_OVERRIDE_TOPIC} in frame "
        f"'{frame_id}' at (x={x_m:.3f}, y={y_m:.3f}, z={z_m:.3f}). "
        f"Pick result: {pick_result}"
    )


@tool
def place_object() -> str:
    """Drop the cube at the left-side tray and return home.

    Calls ``/arm_control/place`` (``std_srvs/Trigger``).
    """
    return _trigger(_PLACE_SRV)


@tool
def arm_go_home() -> str:
    """Move the arm back to its home/tucked pose (``/arm_control/go_home``)."""
    return _trigger(_HOME_SRV)
