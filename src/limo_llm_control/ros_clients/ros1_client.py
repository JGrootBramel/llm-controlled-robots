"""
Native ROS1 client using rospy (recommended when remote PC can join robot's ROS master).

Use this for: topics, services, actions, TF. Configure ROS_MASTER_URI and ROS_IP/ROS_HOSTNAME
so the remote PC connects to the robot's roscore.
"""

from __future__ import annotations

import rospy


def ensure_rospy(node_name: str = "limo_llm_control_tools", anonymous: bool = True) -> None:
    """
    Initialize a lightweight ROS node context for tool-side calls.
    Safe to call multiple times; only initializes once.
    """
    if not rospy.core.is_initialized():
        rospy.init_node(node_name, anonymous=anonymous)
