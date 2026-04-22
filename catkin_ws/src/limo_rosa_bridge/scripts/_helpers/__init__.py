"""Shared pure-Python helpers for limo_rosa_bridge nodes.

These modules contain no ROS I/O (no ``rospy`` import) so they can be unit
tested on a host without a ROS installation. Every node under
``limo_rosa_bridge/scripts/`` is free to import them as::

    from _helpers import hsv, projection, mycobot_helpers, standoff
"""

__all__ = ["hsv", "projection", "mycobot_helpers", "standoff"]
