#!/usr/bin/env python3
"""
ROSA tool: turn the LIMO cobot base on the spot using /cmd_vel.

This module defines a single LangChain/ROSA tool, `turn_in_place`, which the LLM
can call to rotate the base left or right in place for a specified duration.
"""

from typing import Literal

import rospy
from geometry_msgs.msg import Twist
from langchain.tools import tool


_CMD_VEL_PUB = None


def _ensure_rospy_and_pub() -> None:
    """
    Lazily initialize rospy (if needed) and the /cmd_vel publisher.

    ROSA normally initializes rospy in its own entrypoint; this is a safe
    fallback so that the tool also works when imported in isolation.
    """
    global _CMD_VEL_PUB

    if not rospy.core.is_initialized():
        # Anonymous to avoid clashes if another node with same name exists.
        rospy.init_node("llm_limo_tools", anonymous=True)

    if _CMD_VEL_PUB is None:
        _CMD_VEL_PUB = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Give ROS a brief moment to establish connections (non-fatal if it doesn't).
        rospy.sleep(0.05)


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


@tool
def turn_in_place(
    direction: Literal["left", "right"] = "left",
    angular_speed: float = 0.5,
    duration_s: float = 1.0,
) -> str:
    """
    Turn the mobile base on the spot using /cmd_vel.

    Args:
        direction: "left" (counter‑clockwise) or "right" (clockwise).
        angular_speed: Angular velocity in rad/s; values are clamped to [0.0, 1.0].
        duration_s: How long to keep turning before auto‑stopping (seconds).

    Notes for the LLM:
        - If the user doesn't specify angular_speed or duration_s, use the defaults
          without asking follow‑up questions.
        - Use this tool when the user asks to "rotate", "turn around", or "spin in place".
    """
    ang = max(0.0, min(1.0, float(angular_speed)))
    if ang == 0.0 or duration_s <= 0.0:
        return "No rotation requested (zero angular speed or duration)."

    cmd = Twist()
    cmd.angular.z = ang if direction == "left" else -ang

    _publish_twist_for(cmd, duration_s=duration_s, rate_hz=10.0)

    direction_text = "left (counter‑clockwise)" if direction == "left" else "right (clockwise)"
    return f"Turned in place {direction_text} at {ang:.2f} rad/s for {duration_s:.2f} s."

