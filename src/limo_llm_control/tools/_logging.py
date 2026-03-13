"""Log every ROSA tool invocation to ROS and stdout."""
from __future__ import annotations

import rospy


def log_rosa_tool(tool_name: str, **kwargs) -> None:
    """Log that a ROSA tool was invoked (for debugging and runbooks)."""
    parts = [f"{k}={v!r}" for k, v in sorted(kwargs.items())]
    msg = f"[ROSA TOOL] {tool_name}({', '.join(parts)})"
    print(msg, flush=True)
    try:
        rospy.loginfo(msg)
    except Exception:
        pass
