"""Tests for motion ROSA tools (mocked ROS I/O)."""

from __future__ import annotations

from unittest.mock import patch

import geometry_msgs.msg as geometry_msgs
import pytest

from limo_llm_control.tools import motion

pytestmark = pytest.mark.unit


@pytest.fixture(autouse=True)
def _reset_motion_globals() -> None:
    yield
    motion._CMD_VEL_PUB = None
    motion._TF_BUFFER = None
    motion._TF_LISTENER = None


@patch.object(motion, "_turn_with_verification", return_value=(0.5, 0.5, "cumulative_tf"))
def test_turn_in_place_zero_speed_returns_early(_mock: object) -> None:
    out = motion.turn_in_place(angular_speed=0.0)
    assert "No rotation" in out


@patch.object(motion, "_turn_with_verification", return_value=(0.5, 0.5, "cumulative_tf"))
def test_turn_in_place_left_default(_mock: object) -> None:
    out = motion.turn_in_place(direction="left", angular_speed=0.5, duration_s=1.0)
    assert "left" in out.lower()
    assert "Measurement" in out


@patch.object(motion, "_turn_with_verification", return_value=(1.0, 1.0, "cumulative_tf"))
def test_turn_in_place_angle_mode_message(_mock: object) -> None:
    out = motion.turn_in_place(direction="right", angular_speed=0.4, angle_rad=0.785)
    assert "requested" in out.lower()


@patch.object(motion, "_ensure_pub_and_tf")
def test_drive_distance_zero_distance(_mock: object) -> None:
    out = motion.drive_distance(0.0)
    assert "No movement" in out


@patch.object(motion, "_ensure_pub_and_tf")
def test_drive_distance_publishes_forward_twist(_mock_ensure: object) -> None:
    pub = motion.rospy.Publisher("/cmd_vel", geometry_msgs.Twist, queue_size=10)
    motion._CMD_VEL_PUB = pub

    T = motion.rospy.Time
    seq = [T(0.0), T(0.0), T(100.0)]
    with patch.object(motion.rospy.Time, "now", side_effect=seq):
        out = motion.drive_distance(0.4, speed=0.2)

    assert "Drove" in out
    assert "0.40" in out
    published = getattr(motion._CMD_VEL_PUB, "published", [])
    assert published
    forward_cmds = [m for m in published if m.linear.x > 0]
    assert forward_cmds
