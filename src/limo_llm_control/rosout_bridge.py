"""
Copy robot (and any ROS) log output into the central run log.

Subscribes to /rosout (rosgraph_msgs/Log) and writes each message to the
same log file used by the remote PC. No changes required on the robot.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)

# Level constants from rosgraph_msgs/Log (ROS1)
_ROS_DEBUG = 8
_ROS_INFO = 4
_ROS_WARN = 2
_ROS_ERROR = 1
_ROS_FATAL = 0


def _ros_level_to_name(level: int) -> str:
    if level <= _ROS_FATAL:
        return "FATAL"
    if level <= _ROS_ERROR:
        return "ERROR"
    if level <= _ROS_WARN:
        return "WARN"
    if level <= _ROS_INFO:
        return "INFO"
    return "DEBUG"


_spin_thread: Optional[threading.Thread] = None


def _rosout_cb(msg) -> None:
    """Callback for /rosout: write one line to the central log."""
    level_name = _ros_level_to_name(msg.level)
    # Use a dedicated logger so we can prefix; it propagates to limo_llm_control -> same file
    rosout_log = logging.getLogger("limo_llm_control.rosout")
    line = f"[rosout] {level_name} | {msg.name} | {msg.msg}"
    if msg.level <= _ROS_ERROR:
        rosout_log.error(line)
    elif msg.level <= _ROS_WARN:
        rosout_log.warning(line)
    else:
        rosout_log.info(line)


def start_rosout_bridge() -> None:
    """
    Subscribe to /rosout and run rospy.spin() in a daemon thread so robot (and ROS) logs
    are copied into the central log file. Call after rospy is initialized (e.g. ensure_rospy()).
    """
    global _spin_thread
    try:
        import rospy
        from rosgraph_msgs.msg import Log
    except ImportError as e:
        logger.warning("Cannot start rosout bridge (ROS not available): %s", e)
        return

    if not rospy.core.is_initialized():
        logger.warning("Cannot start rosout bridge: rospy not initialized. Call ensure_rospy() first.")
        return

    rospy.Subscriber("/rosout", Log, _rosout_cb, queue_size=100)
    logger.info("Subscribed to /rosout; robot logs will be copied to this run's log file.")

    def spin():
        try:
            rospy.spin()
        except Exception as e:
            logger.exception("rosout bridge spin failed: %s", e)

    _spin_thread = threading.Thread(target=spin, daemon=True)
    _spin_thread.start()
