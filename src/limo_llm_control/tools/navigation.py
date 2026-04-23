"""Navigation ROSA tools: thin rospy clients around robot-side services.

The robot brings up its own autonomy stack via launch files (see
``limo_rosa_bridge/launch/rosa_bridge.launch``). These tools only talk to
that stack over topics/services. No subprocess spawning, no SSH.
"""

from __future__ import annotations

import math
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from langchain.tools import tool
from std_srvs.srv import Empty, SetBool, Trigger
from tf.transformations import quaternion_from_euler

from ..ros_clients import ensure_rospy


_EXPLORATION_ENABLE_SRV = "/exploration_enabled"
_EXPLORATION_RESET_SRV = "/exploration_reset"
_CAM_COVERAGE_RESET_SRV = "/cam_coverage/reset"
_MAP_SAVE_SRV = "/map_update_manager/save_map"
_MAP_NEXT_NAME_PARAM = "/map_update_manager/next_map_name"


def _call_set_bool(service_name: str, value: bool, timeout: float = 2.0) -> str:
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return (
            f"Service '{service_name}' is unavailable. Is the robot autonomy stack running?"
        )
    try:
        proxy = rospy.ServiceProxy(service_name, SetBool)
        resp = proxy(bool(value))
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"Call to {service_name} failed: {exc}"


def _call_trigger(service_name: str, timeout: float = 2.0) -> str:
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return f"Service '{service_name}' is unavailable."
    try:
        proxy = rospy.ServiceProxy(service_name, Trigger)
        resp = proxy()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"Call to {service_name} failed: {exc}"


# --------------------------------------------------------------- exploration


@tool
def start_exploration() -> str:
    """Resume frontier exploration on the robot.

    Calls ``/exploration_enabled`` (``std_srvs/SetBool``) with ``data=True``
    so the ``frontier_explorer`` node on the robot re-starts picking goals
    and publishing them to ``move_base``.
    """
    return _call_set_bool(_EXPLORATION_ENABLE_SRV, True)


@tool
def stop_exploration() -> str:
    """Pause frontier exploration.

    Calls ``/exploration_enabled`` with ``data=False``; the explorer
    cancels its current ``move_base`` goal and stops publishing new ones.
    """
    return _call_set_bool(_EXPLORATION_ENABLE_SRV, False)


@tool
def reset_exploration() -> str:
    """Drop the current frontier goal and restart goal selection.

    Calls ``/exploration_reset`` (``std_srvs/Trigger``).
    """
    return _call_trigger(_EXPLORATION_RESET_SRV)


# ------------------------------------------------------------ cam coverage


@tool
def reset_cam_coverage() -> str:
    """Clear the accumulated camera coverage map (``/cam_coverage/reset``)."""
    ensure_rospy()
    try:
        rospy.wait_for_service(_CAM_COVERAGE_RESET_SRV, timeout=2.0)
        proxy = rospy.ServiceProxy(_CAM_COVERAGE_RESET_SRV, Empty)
        proxy()
        return "Coverage reset."
    except Exception as exc:
        return f"Coverage reset failed: {exc}"


# --------------------------------------------------- move_base single pose


@tool
def go_to_map_pose(
    x_m: float,
    y_m: float,
    yaw_deg: float = 0.0,
    frame_id: str = "map",
    goal_topic: str = "/move_base_simple/goal",
) -> str:
    """Publish a single navigation goal in map coordinates.

    Sends a ``geometry_msgs/PoseStamped`` on ``goal_topic`` (default
    ``/move_base_simple/goal``). Returns as soon as the goal is published.
    """
    ensure_rospy()
    goal_topic = goal_topic.strip() if goal_topic else ""
    frame_id = frame_id.strip() if frame_id else ""
    if not goal_topic:
        return "Invalid goal_topic: empty string is not allowed."
    if not frame_id:
        return "Invalid frame_id: empty string is not allowed."

    pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
    rospy.sleep(0.2)
    yaw_rad = float(yaw_deg) * math.pi / 180.0
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rad)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = frame_id
    goal.pose.position.x = float(x_m)
    goal.pose.position.y = float(y_m)
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw
    pub.publish(goal)
    return (
        f"Published goal to {goal_topic} in frame '{frame_id}' at "
        f"(x={x_m:.2f}, y={y_m:.2f}, yaw={yaw_deg:.1f}°)."
    )


# --------------------------------------------------- emergency halt (kept)


@tool
def cancel_navigation() -> str:
    """Cancel any in-flight ``move_base`` goal and stop the wheels."""
    ensure_rospy()
    try:
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(0.2)
        cancel_pub.publish(GoalID())
        vel_pub.publish(Twist())
        return "Cancelled move_base goal and stopped the base."
    except Exception as exc:
        return f"cancel_navigation failed: {exc}"


@tool
def build_and_save_map(
    map_name: str = "limo_lab_map",
    exploration_duration_s: float = 90.0,
    reset_coverage_first: bool = True,
    reset_explorer_goal_first: bool = True,
) -> str:
    """Explore for a while, then save a new map to ``limo_rosa_bridge/maps``.

    This orchestration tool assumes a live-SLAM launch is running (e.g.
    ``rosa_bridge.launch``) so ``/map`` is actively updated by gmapping.
    Workflow: optional reset(s) -> start exploration -> wait -> stop exploration
    -> call map save service.
    """
    ensure_rospy()
    map_name = (map_name or "").strip()
    if not map_name:
        return "Invalid map_name: provide a non-empty map base name."

    duration = max(5.0, float(exploration_duration_s))
    notes: list[str] = []

    if reset_coverage_first:
        notes.append(f"coverage_reset={_call_trigger(_CAM_COVERAGE_RESET_SRV)}")
    if reset_explorer_goal_first:
        notes.append(f"explorer_reset={_call_trigger(_EXPLORATION_RESET_SRV)}")

    start_msg = _call_set_bool(_EXPLORATION_ENABLE_SRV, True)
    notes.append(f"start_exploration={start_msg}")

    end_time = time.time() + duration
    while not rospy.is_shutdown() and time.time() < end_time:
        rospy.sleep(0.2)

    stop_msg = _call_set_bool(_EXPLORATION_ENABLE_SRV, False)
    notes.append(f"stop_exploration={stop_msg}")

    try:
        rospy.set_param(_MAP_NEXT_NAME_PARAM, map_name)
        rospy.wait_for_service(_MAP_SAVE_SRV, timeout=5.0)
        resp = rospy.ServiceProxy(_MAP_SAVE_SRV, Trigger)()
    except Exception as exc:
        notes.append(f"save_map_error={exc}")
        return (
            f"Map exploration finished after {duration:.0f}s, but map save failed. "
            + "; ".join(notes)
        )

    prefix = "Saved new map successfully." if resp.success else "Map save failed."
    return (
        f"{prefix} map_name='{map_name}', exploration_duration_s={duration:.0f}. "
        f"save_result={resp.message}. "
        + "; ".join(notes)
    )
