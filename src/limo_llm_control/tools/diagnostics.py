"""Diagnostics tools: report stack status and halt motion.

Follows the refactored architecture: no subprocess/SSH plumbing, no
``_node_runner``. All work is done over ROS topics/services.
"""

from __future__ import annotations

import json

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from langchain.tools import tool
from std_srvs.srv import SetBool

from ..ros_clients import ensure_rospy


_KEY_SERVICES = [
    "/exploration_enabled",
    "/exploration_reset",
    "/cam_coverage/reset",
    "/map_update_manager/save_map",
    "/red_cube_detector/enable",
    "/red_cube_detector/snapshot",
    "/approach_object/approach",
    "/approach_object/cancel",
    "/arm_control/pick",
    "/arm_control/place",
    "/arm_control/go_home",
]

_KEY_TOPICS = [
    "/red_cubes/latest_pose",
    "/red_cubes/found",
    "/move_base_simple/goal",
    "/cmd_vel",
    "/cam_coverage",
]


@tool
def halt_robot() -> str:
    """Hard-stop the base: disable exploration, cancel move_base, zero /cmd_vel."""
    ensure_rospy()
    msgs = []
    try:
        rospy.wait_for_service("/exploration_enabled", timeout=1.5)
        rospy.ServiceProxy("/exploration_enabled", SetBool)(False)
        msgs.append("exploration disabled")
    except Exception as exc:
        msgs.append(f"exploration disable skipped: {exc}")
    try:
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(0.2)
        cancel_pub.publish(GoalID())
        vel_pub.publish(Twist())
        msgs.append("move_base cancelled + /cmd_vel zeroed")
    except Exception as exc:
        msgs.append(f"cancel/stop failed: {exc}")
    return "; ".join(msgs)


# Legacy name preserved so existing docs / scripts keep working.
stop_autonomy_nodes = halt_robot


@tool
def get_autonomy_status() -> str:
    """Return a JSON summary of autonomy topics/services visible on the ROS graph."""
    ensure_rospy()
    try:
        master = rospy.get_master()
        code, _, topics = master.getPublishedTopics("")
        published = {name for name, _ in topics} if code == 1 else set()
    except Exception as exc:
        published = set()
        published_err = str(exc)
    else:
        published_err = ""
    try:
        code, _, services = master.getSystemState()
        advertised = set(s[0] for s in services[2]) if code == 1 else set()
    except Exception:
        advertised = set()

    status = {
        "topics": {t: (t in published) for t in _KEY_TOPICS},
        "services": {s: (s in advertised) for s in _KEY_SERVICES},
    }
    if published_err:
        status["warnings"] = [published_err]
    return json.dumps(status, indent=2, sort_keys=True)
