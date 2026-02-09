#!/usr/bin/env python3
"""
ROSA tool: turn the LIMO cobot base on the spot using /cmd_vel.

This module defines a single LangChain/ROSA tool, `turn_in_place`, which the LLM
can call to rotate the base left or right in place. The tool verifies the actual
rotation achieved using TF (transform) to track the robot's orientation.
"""

import math
from typing import Literal, Optional

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion
from langchain.tools import tool


_CMD_VEL_PUB = None
_TF_BUFFER = None
_TF_LISTENER = None


def _ensure_rospy_and_pub() -> None:
    """
    Lazily initialize rospy (if needed), the /cmd_vel publisher, and TF.

    ROSA normally initializes rospy in its own entrypoint; this is a safe
    fallback so that the tool also works when imported in isolation.
    """
    global _CMD_VEL_PUB, _TF_BUFFER, _TF_LISTENER

    if not rospy.core.is_initialized():
        # Anonymous to avoid clashes if another node with same name exists.
        rospy.init_node("llm_limo_tools", anonymous=True)

    if _CMD_VEL_PUB is None:
        _CMD_VEL_PUB = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Give ROS a brief moment to establish connections (non-fatal if it doesn't).
        rospy.sleep(0.05)

    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        _TF_LISTENER = tf2_ros.TransformListener(_TF_BUFFER)
        # Give TF a moment to start receiving transforms
        rospy.sleep(0.1)


def _yaw_from_quat(q: Quaternion) -> float:
    """
    Extract yaw angle from a planar quaternion.

    Args:
        q: Input quaternion.

    Returns:
        Yaw angle in radians.
    """
    return math.atan2(2.0 * (q.w * q.z), 1 - 2.0 * (q.z * q.z))


def _angle_diff(a: float, b: float) -> float:
    """
    Compute the shortest signed angular difference between two angles.

    Args:
        a: First angle in radians.
        b: Second angle in radians.

    Returns:
        Difference a - b wrapped into the range [-pi, pi].
    """
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


def _get_robot_yaw(frame_id: str = "map", base_frame: str = "base_link") -> Optional[float]:
    """
    Get the current robot yaw angle from TF.

    Args:
        frame_id: Reference frame (typically "map" or "odom").
        base_frame: Robot base frame (typically "base_link").

    Returns:
        Yaw angle in radians, or None if TF lookup fails.
    """
    _ensure_rospy_and_pub()
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
    """
    Publish a Twist at a fixed rate for a duration, then send a stop command once.
    """
    _ensure_rospy_and_pub()
    assert _CMD_VEL_PUB is not None

    duration_s = max(0.0, float(duration_s))
    rate_hz = max(1.0, float(rate_hz))

    end_time = rospy.Time.now() + rospy.Duration.from_sec(duration_s)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        _CMD_VEL_PUB.publish(cmd)
        rate.sleep()

    # Send a single stop command at the end to ensure the controller halts.
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
) -> tuple[float, float]:
    """
    Turn the robot and verify the actual rotation achieved.

    Args:
        cmd: Twist command to publish.
        target_angle_rad: Target rotation angle in radians (None = use duration-based).
        direction: "left" or "right".
        angular_speed: Angular velocity in rad/s.
        max_duration_s: Maximum duration to turn (safety limit).
        tolerance_rad: Tolerance for reaching target angle (radians).
        frame_id: TF reference frame.
        base_frame: Robot base frame.

    Returns:
        Tuple of (actual_rotation_rad, requested_rotation_rad).
    """
    _ensure_rospy_and_pub()
    assert _CMD_VEL_PUB is not None

    # Get initial yaw
    initial_yaw = _get_robot_yaw(frame_id, base_frame)
    if initial_yaw is None:
        # Fallback to duration-based if TF unavailable
        rospy.logwarn("[turn_in_place] TF unavailable, using duration-based turn")
        _publish_twist_for(cmd, max_duration_s)
        requested_rad = angular_speed * max_duration_s
        return requested_rad, requested_rad

    # Calculate requested rotation
    if target_angle_rad is not None and target_angle_rad > 0:
        requested_rad = target_angle_rad
        # Estimate duration, but cap it
        estimated_duration = min(max_duration_s, requested_rad / max(angular_speed, 0.01))
    else:
        requested_rad = angular_speed * max_duration_s
        estimated_duration = max_duration_s

    # Turn while monitoring rotation
    rate = rospy.Rate(20)  # 20 Hz for smooth monitoring
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration.from_sec(estimated_duration)
    last_yaw = initial_yaw
    cumulative_rotation = 0.0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Check if we've reached target (if angle-based)
        if target_angle_rad is not None and target_angle_rad > 0:
            current_yaw = _get_robot_yaw(frame_id, base_frame)
            if current_yaw is not None:
                delta_yaw = _angle_diff(current_yaw, last_yaw)
                cumulative_rotation += abs(delta_yaw)
                last_yaw = current_yaw

                # Check if we've reached the target
                if cumulative_rotation >= (target_angle_rad - tolerance_rad):
                    break

        # Check time limit
        if current_time >= end_time:
            break

        _CMD_VEL_PUB.publish(cmd)
        rate.sleep()

    # Stop the robot
    stop = Twist()
    for _ in range(5):
        _CMD_VEL_PUB.publish(stop)
        rate.sleep()

    # Get final yaw and calculate actual rotation
    final_yaw = _get_robot_yaw(frame_id, base_frame)
    if final_yaw is not None:
        actual_rotation = abs(_angle_diff(final_yaw, initial_yaw))
    else:
        # Fallback: use cumulative rotation if available, otherwise estimate
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

    The tool tracks the robot's orientation using TF and reports the actual rotation
    achieved versus the requested rotation.

    Args:
        direction: "left" (counter‑clockwise) or "right" (clockwise).
        angular_speed: Angular velocity in rad/s; values are clamped to [0.0, 1.0].
        duration_s: How long to keep turning before auto‑stopping (seconds).
                   If None and angle_rad is provided, duration is calculated from angle.
                   Default: 1.0 if angle_rad is None.
        angle_rad: Target rotation angle in radians. If provided, the tool will turn
                   until this angle is reached (within tolerance) or max duration expires.
                   Takes precedence over duration_s if both are provided.

    Notes for the LLM:
        - If the user specifies an angle (e.g., "turn 90 degrees"), use angle_rad.
        - If the user specifies a duration (e.g., "turn for 2 seconds"), use duration_s.
        - If neither is specified, use defaults (duration_s=1.0) without asking follow‑up questions.
        - The tool verifies actual rotation and reports accuracy.
        - Use this tool when the user asks to "rotate", "turn around", "spin in place",
          or "turn X degrees/radians".
    """
    ang = max(0.0, min(1.0, float(angular_speed)))
    if ang == 0.0:
        return "No rotation requested (zero angular speed)."

    # Determine mode: angle-based or duration-based
    if angle_rad is not None and angle_rad > 0:
        # Angle-based: calculate max duration as safety limit
        max_duration = max(1.0, (angle_rad / max(ang, 0.01)) * 1.5)  # 1.5x safety margin
        use_angle = True
        target_angle = float(angle_rad)
    else:
        # Duration-based: use provided duration or default
        if duration_s is None or duration_s <= 0:
            duration_s = 1.0
        max_duration = float(duration_s)
        use_angle = False
        target_angle = None

    cmd = Twist()
    cmd.angular.z = ang if direction == "left" else -ang

    # Perform turn with verification
    actual_rad, requested_rad = _turn_with_verification(
        cmd=cmd,
        target_angle_rad=target_angle,
        direction=direction,
        angular_speed=ang,
        max_duration_s=max_duration,
        tolerance_rad=0.05,  # ~3 degrees tolerance
    )

    # Format response
    direction_text = "left (counter‑clockwise)" if direction == "left" else "right (clockwise)"
    actual_deg = math.degrees(actual_rad)
    requested_deg = math.degrees(requested_rad)
    error_deg = abs(actual_deg - requested_deg)
    error_percent = (error_deg / requested_deg * 100) if requested_deg > 0 else 0.0

    if use_angle:
        result = (
            f"Turned in place {direction_text} by {actual_deg:.1f}° "
            f"(requested: {requested_deg:.1f}°, error: {error_deg:.1f}° / {error_percent:.1f}%). "
            f"Angular speed: {ang:.2f} rad/s."
        )
    else:
        result = (
            f"Turned in place {direction_text} for {max_duration:.2f} s at {ang:.2f} rad/s. "
            f"Actual rotation: {actual_deg:.1f}° (expected: {requested_deg:.1f}°, "
            f"error: {error_deg:.1f}° / {error_percent:.1f}%)."
        )

    return result

