"""
Minimal ROS1 stand-ins so limo_llm_control tools import on hosts without ROS.

Not a simulation — only enough API surface for imports and unit tests.
"""

from __future__ import annotations

import math
import sys
import types
from types import SimpleNamespace
from typing import Any, Callable, List, Optional
def _identity_tool(f: Callable[..., Any]) -> Callable[..., Any]:
    """LangChain @tool replacement for CI (no langchain package required)."""
    return f


_INSTALLED = False


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return

    # --- langchain.tools (decorator only; real tools remain plain callables) ---
    lc_tools = types.ModuleType("langchain.tools")
    lc_tools.tool = _identity_tool  # type: ignore[attr-defined]
    sys.modules["langchain.tools"] = lc_tools

    # --- rospy ---
    class Duration:
        __slots__ = ("secs",)

        def __init__(self, secs: float = 0.0) -> None:
            self.secs = float(secs)

        def to_sec(self) -> float:
            return self.secs

        @classmethod
        def from_sec(cls, s: float) -> "Duration":
            return cls(float(s))

    class _Time:
        _sec: float = 0.0

        def __init__(self, sec: Optional[float] = None) -> None:
            self.sec = float(_Time._sec if sec is None else sec)

        @classmethod
        def now(cls) -> "_Time":
            return cls(cls._sec)

        def __lt__(self, other: object) -> bool:
            if not isinstance(other, _Time):
                return NotImplemented
            return self.sec < other.sec

        def __add__(self, other: object) -> "_Time":
            if isinstance(other, Duration):
                return _Time(self.sec + other.secs)
            return NotImplemented

    class Rate:
        def __init__(self, hz: float) -> None:
            self.hz = float(hz)

        def sleep(self) -> None:
            pass

    class Publisher:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.published: List[Any] = []

        def publish(self, msg: Any) -> None:
            self.published.append(msg)

    class Subscriber:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

        def unregister(self) -> None:
            pass

    class _Core:
        _initialized = False

        @staticmethod
        def is_initialized() -> bool:
            return _Core._initialized

    def init_node(*args: Any, **kwargs: Any) -> None:
        _Core._initialized = True

    def sleep(_dt: float = 0.0) -> None:
        pass

    def is_shutdown() -> bool:
        return False

    def logwarn(*args: Any, **kwargs: Any) -> None:
        pass

    def wait_for_service(_name: str, timeout: float = 0.0) -> None:
        pass

    def wait_for_message(_topic: str, _type: Any, timeout: float = 0.0) -> Any:
        raise TimeoutError("stub")

    class ServiceProxy:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

        def __call__(self, *args: Any, **kwargs: Any) -> SimpleNamespace:
            return SimpleNamespace(success=True, message="stub")

    rospy_mod = types.ModuleType("rospy")
    rospy_mod.Duration = Duration
    rospy_mod.Time = _Time
    rospy_mod.Rate = Rate
    rospy_mod.Publisher = Publisher
    rospy_mod.Subscriber = Subscriber
    rospy_mod.init_node = init_node
    rospy_mod.sleep = sleep
    rospy_mod.is_shutdown = is_shutdown
    rospy_mod.logwarn = logwarn
    rospy_mod.wait_for_service = wait_for_service
    rospy_mod.wait_for_message = wait_for_message
    rospy_mod.ServiceProxy = ServiceProxy
    rospy_mod.core = _Core
    sys.modules["rospy"] = rospy_mod

    # --- geometry_msgs.msg ---
    class Twist:
        def __init__(self) -> None:
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class Quaternion:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class PoseStamped:
        def __init__(self) -> None:
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(),
            )

    geom = types.ModuleType("geometry_msgs.msg")
    geom.Twist = Twist
    geom.Quaternion = Quaternion
    geom.PoseStamped = PoseStamped
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geom

    # --- std_msgs.msg ---
    class Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = data

    class String:
        def __init__(self, data: str = "") -> None:
            self.data = data

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Bool = Bool
    std_msgs.String = String
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std_msgs

    # --- std_srvs.srv ---
    class Empty:
        pass

    class Trigger:
        pass

    std_srvs = types.ModuleType("std_srvs.srv")
    std_srvs.Empty = Empty
    std_srvs.Trigger = Trigger
    sys.modules["std_srvs"] = types.ModuleType("std_srvs")
    sys.modules["std_srvs.srv"] = std_srvs

    # --- actionlib_msgs.msg ---
    class GoalID:
        pass

    actionlib_msgs = types.ModuleType("actionlib_msgs.msg")
    actionlib_msgs.GoalID = GoalID
    sys.modules["actionlib_msgs"] = types.ModuleType("actionlib_msgs")
    sys.modules["actionlib_msgs.msg"] = actionlib_msgs

    # --- tf.transformations ---
    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (x, y, z, w)

    tf_trans = types.ModuleType("tf.transformations")
    tf_trans.quaternion_from_euler = quaternion_from_euler
    sys.modules["tf"] = types.ModuleType("tf")
    sys.modules["tf.transformations"] = tf_trans

    # --- tf2_ros ---
    class LookupException(Exception):
        pass

    class ConnectivityException(Exception):
        pass

    class ExtrapolationException(Exception):
        pass

    class Buffer:
        def __init__(self, cache_time: Any = None) -> None:
            self.cache_time = cache_time

        def lookup_transform(self, *args: Any, **kwargs: Any) -> Any:
            raise LookupException("stub")

    class TransformListener:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = Buffer
    tf2_ros.TransformListener = TransformListener
    tf2_ros.LookupException = LookupException
    tf2_ros.ConnectivityException = ConnectivityException
    tf2_ros.ExtrapolationException = ExtrapolationException
    sys.modules["tf2_ros"] = tf2_ros

    _INSTALLED = True
