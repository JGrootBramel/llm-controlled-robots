"""Mission-level convenience tools that compose the primitive tools.

The primary mission for the LIMO test task is "explore the room, find red
cubes, pick them up, and deliver them to a fixed coordinate". Decomposing
this on the robot would require yet another FSM node, so we keep it on
the remote side where ROSA can also narrate / reason over each step.

Everything here calls directly into ROS (same way the primitive ROSA
tools do) so we don't rely on langchain-specific tool call forwarding.
"""

from __future__ import annotations

import math
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from langchain.tools import tool
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf.transformations import quaternion_from_euler

from ..ros_clients import ensure_rospy


def _trigger(service_name: str, timeout: float = 2.0) -> str:
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return f"FAIL: '{service_name}' unavailable"
    try:
        resp = rospy.ServiceProxy(service_name, Trigger)()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {service_name}: {resp.message}"
    except Exception as exc:
        return f"FAIL: {service_name}: {exc}"


def _set_bool(service_name: str, value: bool, timeout: float = 2.0) -> str:
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return f"FAIL: '{service_name}' unavailable"
    try:
        resp = rospy.ServiceProxy(service_name, SetBool)(bool(value))
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {service_name}({value}): {resp.message}"
    except Exception as exc:
        return f"FAIL: {service_name}({value}): {exc}"


def _wait_for_found(timeout_s: float) -> bool:
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        remain = max(0.1, deadline - time.time())
        try:
            msg = rospy.wait_for_message(
                "/red_cubes/found", Bool, timeout=min(1.0, remain)
            )
            if msg.data:
                return True
        except Exception:
            pass
    return False


def _publish_delivery_goal(x: float, y: float, yaw_deg: float) -> str:
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(0.2)
    yaw = float(yaw_deg) * math.pi / 180.0
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    pub.publish(ps)
    return f"OK: delivery goal published ({x:.2f},{y:.2f},{yaw_deg:.0f}°)"


@tool
def fetch_red_cubes(
    delivery_x: float = 0.0,
    delivery_y: float = 0.0,
    delivery_yaw_deg: float = 0.0,
    max_cubes: int = 1,
    detection_timeout_s: float = 90.0,
) -> str:
    """Full mission: explore, find red cubes, pick, deliver, place.

    Sequence for each cube (up to ``max_cubes``):

    1. Enable the red cube detector.
    2. Start frontier exploration.
    3. Wait for ``/red_cubes/found`` to go True (up to
       ``detection_timeout_s`` seconds).
    4. Stop exploration, take a fresh pose snapshot.
    5. ``approach_object``, ``pick``, drive to the delivery coordinate,
       and ``place``.

    Args:
        delivery_x: Drop-off X in map frame (m).
        delivery_y: Drop-off Y in map frame (m).
        delivery_yaw_deg: Final orientation at drop-off (deg).
        max_cubes: Cap on how many cubes to fetch in this mission.
        detection_timeout_s: How long to explore before giving up per cube.
    """
    ensure_rospy()
    log = [_set_bool("/red_cube_detector/enable", True)]
    delivered = 0

    for i in range(int(max_cubes)):
        log.append(_set_bool("/exploration_enabled", True))
        found = _wait_for_found(detection_timeout_s)
        log.append(_set_bool("/exploration_enabled", False))
        if not found:
            log.append(f"FAIL: cube #{i + 1} not found within {detection_timeout_s}s")
            break

        log.append(_trigger("/red_cube_detector/snapshot"))
        log.append(_trigger("/approach_object/approach"))
        log.append(_trigger("/arm_control/pick"))
        log.append(_publish_delivery_goal(delivery_x, delivery_y, delivery_yaw_deg))
        rospy.sleep(1.0)
        log.append(_trigger("/arm_control/place"))
        delivered += 1

    header = f"fetch_red_cubes: delivered {delivered}/{max_cubes}"
    return header + "\n" + "\n".join(log)
