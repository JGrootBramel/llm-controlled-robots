"""
Motion tools: base velocity control via /cmd_vel using rospy.

Exposes turn-in-place and drive distance as ROSA capability wrappers.
Robot-side runs the actual controllers; remote PC publishes Twist commands.
"""

from __future__ import annotations

import math
from typing import Literal, Optional

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, Twist
from langchain.tools import tool

from ..ros_clients import ensure_rospy

_CMD_VEL_PUB = None
_TF_BUFFER = None
_TF_LISTENER = None


def _ensure_pub_and_tf() -> None:
    global _CMD_VEL_PUB, _TF_BUFFER, _TF_LISTENER
    ensure_rospy()
    if _CMD_VEL_PUB is None:
        _CMD_VEL_PUB = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.sleep(0.05)
    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        _TF_LISTENER = tf2_ros.TransformListener(_TF_BUFFER)
        rospy.sleep(0.1)


def _yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z), 1 - 2.0 * (q.z * q.z))


def _angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _get_robot_yaw(frame_id: str = "map", base_frame: str = "base_link") -> Optional[float]:
    _ensure_pub_and_tf()
    if _TF_BUFFER is None:
        return None
    try:
        transform = _TF_BUFFER.lookup_transform(
            frame_id, base_frame, rospy.Time(0), rospy.Duration(0.5)
        )
        return _yaw_from_quat(transform.transform.rotation)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None


def _publish_twist_for(cmd: Twist, duration_s: float, rate_hz: float = 10.0) -> None:
    _ensure_pub_and_tf()
    assert _CMD_VEL_PUB is not None
    duration_s = max(0.0, float(duration_s))
    rate_hz = max(1.0, float(rate_hz))
    end_time = rospy.Time.now() + rospy.Duration.from_sec(duration_s)
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        _CMD_VEL_PUB.publish(cmd)
        rate.sleep()
    stop = Twist()
    _CMD_VEL_PUB.publish(stop)


def _turn_with_verification(
    cmd: Twist,
    target_angle_rad: Optional[float],
    direction: str,
    angular_speed: float,
    max_duration_s: float,
    tolerance_rad: float = 0.05,
    frame_id: str = "map",
    base_frame: str = "base_link",
) -> tuple:
    _ensure_pub_and_tf()
    assert _CMD_VEL_PUB is not None
    initial_yaw = _get_robot_yaw(frame_id, base_frame)
    if initial_yaw is None:
        rospy.logwarn("[turn_in_place] TF unavailable, using duration-based turn")
        _publish_twist_for(cmd, max_duration_s)
        requested_rad = angular_speed * max_duration_s
        return requested_rad, requested_rad

    if target_angle_rad is not None and target_angle_rad > 0:
        requested_rad = target_angle_rad
        estimated_duration = min(max_duration_s, requested_rad / max(angular_speed, 0.01))
    else:
        requested_rad = angular_speed * max_duration_s
        estimated_duration = max_duration_s

    rate = rospy.Rate(20)
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration.from_sec(estimated_duration)
    last_yaw = initial_yaw
    cumulative_rotation = 0.0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        if target_angle_rad is not None and target_angle_rad > 0:
            current_yaw = _get_robot_yaw(frame_id, base_frame)
            if current_yaw is not None:
                delta_yaw = _angle_diff(current_yaw, last_yaw)
                cumulative_rotation += abs(delta_yaw)
                last_yaw = current_yaw
                if cumulative_rotation >= (target_angle_rad - tolerance_rad):
                    break
        if current_time >= end_time:
            break
        _CMD_VEL_PUB.publish(cmd)
        rate.sleep()

    stop = Twist()
    for _ in range(5):
        _CMD_VEL_PUB.publish(stop)
        rate.sleep()

    final_yaw = _get_robot_yaw(frame_id, base_frame)
    if final_yaw is not None:
        actual_rotation = abs(_angle_diff(final_yaw, initial_yaw))
    else:
        actual_rotation = cumulative_rotation if cumulative_rotation > 0 else requested_rad
    return actual_rotation, requested_rad


@tool
def turn_in_place(
    direction: Literal["left", "right"] = "left",
    angular_speed: float = 0.5,
    duration_s: Optional[float] = None,
    angle_rad: Optional[float] = None,
) -> str:
    """
    Turn the mobile base on the spot using /cmd_vel and verify the actual rotation.

    Tracks orientation via TF and reports actual vs requested rotation.

    Args:
        direction: "left" (counter‑clockwise) or "right" (clockwise).
        angular_speed: Angular velocity in rad/s; clamped to [0.0, 1.0].
        duration_s: How long to turn (seconds). Default 1.0 if angle_rad is None.
        angle_rad: Target rotation in radians; takes precedence over duration_s.
    """
    ang = max(0.0, min(1.0, float(angular_speed)))
    if ang == 0.0:
        return "No rotation requested (zero angular speed)."

    if angle_rad is not None and angle_rad > 0:
        max_duration = max(1.0, (angle_rad / max(ang, 0.01)) * 1.5)
        use_angle = True
        target_angle = float(angle_rad)
    else:
        duration_s = float(duration_s) if duration_s and duration_s > 0 else 1.0
        max_duration = duration_s
        use_angle = False
        target_angle = None

    cmd = Twist()
    cmd.angular.z = ang if direction == "left" else -ang

    actual_rad, requested_rad = _turn_with_verification(
        cmd=cmd,
        target_angle_rad=target_angle,
        direction=direction,
        angular_speed=ang,
        max_duration_s=max_duration,
        tolerance_rad=0.05,
    )

    direction_text = "left (counter‑clockwise)" if direction == "left" else "right (clockwise)"
    actual_deg = math.degrees(actual_rad)
    requested_deg = math.degrees(requested_rad)
    error_deg = abs(actual_deg - requested_deg)
    error_percent = (error_deg / requested_deg * 100) if requested_deg > 0 else 0.0

    if use_angle:
        return (
            f"Turned in place {direction_text} by {actual_deg:.1f}° "
            f"(requested: {requested_deg:.1f}°, error: {error_deg:.1f}° / {error_percent:.1f}%). "
            f"Angular speed: {ang:.2f} rad/s."
        )
    return (
        f"Turned in place {direction_text} for {max_duration:.2f} s at {ang:.2f} rad/s. "
        f"Actual rotation: {actual_deg:.1f}° (expected: {requested_deg:.1f}°, "
        f"error: {error_deg:.1f}° / {error_percent:.1f}%)."
    )


@tool
def drive_distance(meters: float, speed: float = 0.2) -> str:
    """
    Drive the robot forward or backward by a given distance using /cmd_vel (rospy).

    Positive meters = forward, negative = backward. Uses a constant linear speed
    and publishes Twist for the required duration, then stops.

    Args:
        meters: Distance in meters (positive forward, negative backward).
        speed: Linear speed in m/s (absolute value used); clamped to [0.05, 1.0].
    """
    linear = max(0.05, min(1.0, abs(float(speed))))
    distance = abs(float(meters))
    if distance < 1e-6:
        return "No movement requested (zero distance)."
    duration_s = distance / linear
    if meters < 0:
        linear = -linear

    _ensure_pub_and_tf()
    assert _CMD_VEL_PUB is not None
    cmd = Twist()
    cmd.linear.x = linear
    end_time = rospy.Time.now() + rospy.Duration.from_sec(duration_s)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        _CMD_VEL_PUB.publish(cmd)
        rate.sleep()
    stop = Twist()
    _CMD_VEL_PUB.publish(stop)
    return f"Drove {meters:.2f} m at {abs(linear):.2f} m/s."
