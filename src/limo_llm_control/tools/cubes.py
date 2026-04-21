"""Object-scan tools based on object_finder outputs in map frame."""

from __future__ import annotations

import math
from typing import List, Tuple

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from langchain.tools import tool

from .motion import turn_in_place
from .perception import update_object_query
_OBJECT_POSE_TOPIC = "/object_pose"


def _run_scan(
    duration_seconds: float,
    spin: bool,
    pose_topic: str = _OBJECT_POSE_TOPIC,
    merge_distance_m: float = 0.05,
    min_range_m: float = 0.2,
    max_range_m: float = 1.5,
    lateral_limit_m: float = 0.5,
    round_precision: int = 1,
):
    """Run a generic object scan and return merged map (x, y) coordinates."""
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    found_points = set()

    def _add_if_valid(x: float, y: float):
        r = (x * x + y * y) ** 0.5
        if r < min_range_m or r > max_range_m:
            return
        if abs(y) > lateral_limit_m:
            return
        gx, gy = round(x, round_precision), round(y, round_precision)
        found_points.add((gx, gy))

    def pose_cb(msg: PoseStamped):
        _add_if_valid(msg.pose.position.x, msg.pose.position.y)
    sub = rospy.Subscriber(pose_topic, PoseStamped, pose_cb)

    end_time = rospy.Time.now() + rospy.Duration(duration_seconds)
    if spin:
        # Reuse motion tool turn verification to perform one full 360° scan turn.
        turn_in_place(direction="left", angular_speed=0.5, angle_rad=2.0 * math.pi)

    rate = rospy.Rate(10)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        rate.sleep()

    cmd_pub.publish(Twist())
    sub.unregister()

    merged = []
    for (x, y) in sorted(found_points):
        if not merged:
            merged.append((x, y))
            continue
        d_min = min(math.hypot(x - mx, y - my) for (mx, my) in merged)
        if d_min >= merge_distance_m:
            merged.append((x, y))
    return merged


def _format_scan_result(object_name_plural: str, merged: List[Tuple[float, float]]) -> str:
    if not merged:
        return f"Scan complete. No {object_name_plural} detected."
    result = f"Scan complete. Found {object_name_plural} at the following map coordinates:\n"
    for idx, (x, y) in enumerate(merged):
        result += f"- Object {idx + 1}: x={x}, y={y}\n"
    return result


@tool
def scan_for_objects(
    object_query: str = "a red cube",
    duration_seconds: int = 25,
    spin: bool = True,
    merge_distance_m: float = 0.08,
    max_range_m: float = 2.5,
    lateral_limit_m: float = 1.2,
) -> str:
    """
    Scan for detected objects in map frame.

    This tool updates `/object_query` and then listens on `/object_pose`.
    """
    q = object_query.strip()
    if q:
        update_object_query(q)
    merged = _run_scan(
        duration_seconds=float(duration_seconds),
        spin=spin,
        pose_topic=_OBJECT_POSE_TOPIC,
        merge_distance_m=max(0.01, float(merge_distance_m)),
        max_range_m=max(0.3, float(max_range_m)),
        lateral_limit_m=max(0.2, float(lateral_limit_m)),
    )
    return _format_scan_result(q or "objects", merged)


