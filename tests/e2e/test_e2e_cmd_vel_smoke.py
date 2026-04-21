"""Short integration smoke: publish to /cmd_vel (opt-in via RUN_E2E)."""

from __future__ import annotations

import os

import pytest

pytest.importorskip("rospy")

pytestmark = pytest.mark.e2e


def _e2e_enabled() -> bool:
    return os.environ.get("RUN_E2E") == "1" and bool(os.environ.get("ROS_MASTER_URI", "").strip())


@pytest.mark.skipif(
    not _e2e_enabled(),
    reason="Set RUN_E2E=1 and ROS_MASTER_URI for end-to-end smoke",
)
def test_publish_cmd_vel_zero_twist() -> None:
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("pytest_e2e_cmd_vel", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.3)
    pub.publish(Twist())
    rospy.sleep(0.1)
