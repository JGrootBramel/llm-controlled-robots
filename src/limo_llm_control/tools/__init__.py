"""ROSA tool wrappers for the refactored LIMO autonomy stack.

Each tool is a thin rospy client around a service or topic exposed by the
robot-side nodes (see ``catkin_ws/src/limo_rosa_bridge/scripts/*``). The
robot's launch files bring those nodes up; we don't spawn processes from
the remote PC anymore.

Modules:

- ``motion``       — ``turn_in_place``, ``drive_distance`` over ``/cmd_vel``
- ``navigation``   — exploration enable/reset, ``move_base`` goal publish
- ``perception``   — red cube detector enable/snapshot + pose read
- ``manipulation`` — approach/pick/place/home triggers
- ``mission``      — ``fetch_red_cubes`` end-to-end composition
- ``diagnostics``  — status + hard stop
"""

from .diagnostics import get_autonomy_status, halt_robot, stop_autonomy_nodes
from .manipulation import (
    approach_object,
    arm_go_home,
    cancel_approach,
    pick_object,
    place_object,
)
from .mission import fetch_red_cubes
from .motion import drive_distance, turn_in_place
from .navigation import (
    cancel_navigation,
    go_to_map_pose,
    reset_cam_coverage,
    reset_exploration,
    start_exploration,
    stop_exploration,
)
from .perception import (
    enable_red_cube_detector,
    get_latest_red_cube,
    is_red_cube_found,
    snapshot_red_cube,
)

__all__ = [
    # motion
    "turn_in_place",
    "drive_distance",
    # navigation
    "start_exploration",
    "stop_exploration",
    "reset_exploration",
    "reset_cam_coverage",
    "go_to_map_pose",
    "cancel_navigation",
    # perception
    "enable_red_cube_detector",
    "snapshot_red_cube",
    "get_latest_red_cube",
    "is_red_cube_found",
    # manipulation
    "approach_object",
    "cancel_approach",
    "pick_object",
    "place_object",
    "arm_go_home",
    # mission
    "fetch_red_cubes",
    # diagnostics
    "halt_robot",
    "stop_autonomy_nodes",
    "get_autonomy_status",
]
