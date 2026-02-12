# ROS client layer: use native ROS1 (rospy) for robot communication where possible.
# See docs/architecture/communication-options_robot-remote_pc.md.

from .ros1_client import ensure_rospy

__all__ = ["ensure_rospy"]
