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

from .diagnostics import (
    get_autonomy_status,
    halt_robot,
    healthcheck_autonomy_stack,
    stop_autonomy_nodes,
)
from .manipulation import (
    approach_object,
    arm_go_home,
    cancel_approach,
    drop_at_pose,
    pick_at_pose,
    pick_object,
    pick_object_vendor_sync,
    place_object,
)
from .mission import (
    explore_and_fetch_all_cubes,
    fetch_red_cubes,
    fetch_red_cubes_to_start,
    process_clicked_points,
)
from .motion import drive_distance, turn_in_place
from .navigation import (
    build_and_save_map,
    cancel_navigation,
    get_current_map_pose,
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
    "get_current_map_pose",
    "cancel_navigation",
    "build_and_save_map",
    # perception
    "enable_red_cube_detector",
    "snapshot_red_cube",
    "get_latest_red_cube",
    "is_red_cube_found",
    # manipulation
    "approach_object",
    "cancel_approach",
    "pick_object",
    "pick_object_vendor_sync",
    "pick_at_pose",
    "drop_at_pose",
    "place_object",
    "arm_go_home",
    # mission
    "fetch_red_cubes",
    "fetch_red_cubes_to_start",
    "explore_and_fetch_all_cubes",
    "process_clicked_points",
    # diagnostics
    "halt_robot",
    "stop_autonomy_nodes",
    "get_autonomy_status",
    "healthcheck_autonomy_stack",
]
