"""Manipulation tools: trigger pick/place/home on the arm_control_node."""

from __future__ import annotations

import rospy
from langchain.tools import tool
from std_srvs.srv import Trigger

from ..ros_clients import ensure_rospy


_PICK_SRV = "/arm_control/pick"
_PLACE_SRV = "/arm_control/place"
_HOME_SRV = "/arm_control/go_home"
_APPROACH_SRV = "/approach_object/approach"
_CANCEL_APPROACH_SRV = "/approach_object/cancel"


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
def place_object() -> str:
    """Drop the cube at the left-side tray and return home.

    Calls ``/arm_control/place`` (``std_srvs/Trigger``).
    """
    return _trigger(_PLACE_SRV)


@tool
def arm_go_home() -> str:
    """Move the arm back to its home/tucked pose (``/arm_control/go_home``)."""
    return _trigger(_HOME_SRV)
