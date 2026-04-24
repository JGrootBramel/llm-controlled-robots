"""Mission-level convenience tools that compose the primitive tools.

The primary mission for the LIMO test task is "explore the room, find red
cubes, pick them up, and deliver them to a fixed coordinate". Decomposing
this on the robot would require yet another FSM node, so we keep it on
the remote side where ROSA can also narrate / reason over each step.

This version provides a coordinate-based fetching tool that uses stubs
for underlying actions to demonstrate the mission sequence.
"""

from __future__ import annotations

import json
import math
import time
import queue
from typing import Iterable, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from langchain.tools import tool
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ..ros_clients import ensure_rospy


def _trigger(service_name: str, timeout: float = 2.0) -> str:
    """Calls a std_srvs/Trigger service and returns a status string."""
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return f"FAIL: Service '{service_name}' is unavailable."
    try:
        resp = rospy.ServiceProxy(service_name, Trigger)()
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"FAIL: Call to {service_name} failed: {exc}"


def _set_bool(service_name: str, value: bool, timeout: float = 2.0) -> str:
    """Calls a std_srvs/SetBool service and returns a status string."""
    ensure_rospy()
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except Exception:
        return (
            f"FAIL: Service '{service_name}' is unavailable. Is the robot autonomy stack running?"
        )
    try:
        proxy = rospy.ServiceProxy(service_name, SetBool)
        resp = proxy(bool(value))
        prefix = "OK" if resp.success else "FAIL"
        return f"{prefix}: {resp.message}"
    except Exception as exc:
        return f"FAIL: Call to {service_name} failed: {exc}"


def _set_bool_with_retry(
    service_name: str,
    value: bool,
    total_timeout_s: float = 20.0,
    retry_interval_s: float = 1.0,
) -> str:
    """Retries SetBool calls during startup races and returns final status."""
    ensure_rospy()
    deadline = time.time() + float(total_timeout_s)
    last_result = ""
    while time.time() < deadline and not rospy.is_shutdown():
        # Use a short per-attempt timeout so we can retry quickly.
        last_result = _set_bool(service_name, value, timeout=min(2.0, retry_interval_s))
        if last_result.startswith("OK:"):
            return last_result
        rospy.sleep(float(retry_interval_s))
    if last_result:
        return (
            f"{last_result} (retried for {total_timeout_s:.1f}s waiting for service startup)"
        )
    return (
        f"FAIL: Service '{service_name}' did not become ready within "
        f"{total_timeout_s:.1f}s."
    )


def _go_to_map_pose(x: float, y: float, yaw_deg: float) -> str:
    """Navigates the robot to a map pose and waits for completion."""
    import actionlib
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

    ensure_rospy()
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        return "FAIL: move_base action server not available"

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)

    yaw = float(yaw_deg) * math.pi / 180.0
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(60.0)):  # 60s timeout for nav
        state = client.get_state()
        # GoalStatus: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
        if state == actionlib.GoalStatus.SUCCEEDED:
            return f"OK: Reached goal ({x:.2f}, {y:.2f})"
        else:
            return f"FAIL: Navigation failed with state {state}"
    else:
        client.cancel_goal()
        return "FAIL: Navigation timed out after 60s"


def _publish_dummy_pick_pose() -> str:
    """Publishes a fixed pose in front of the robot for the arm to target."""
    pub = rospy.Publisher("/arm_control/target_pose", PoseStamped, queue_size=1)
    rospy.sleep(0.2)  # wait for publisher to connect

    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "base_link"  # arm controller expects pose in base_link
    ps.pose.position.x = 0.25  # 25cm in front of the robot base
    ps.pose.position.y = 0.0
    ps.pose.position.z = 0.10  # 5cm above the ground
    ps.pose.orientation.w = 1.0  # level gripper

    pub.publish(ps)
    rospy.sleep(0.2)  # wait for message to be processed
    return "OK: Published dummy pick pose to /arm_control/target_pose"


def _wait_for_found(timeout_s: float) -> bool:
    """Wait for `/red_cubes/found` to publish True within timeout."""
    ensure_rospy()
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline and not rospy.is_shutdown():
        remaining = deadline - time.time()
        if remaining <= 0.0:
            break
        try:
            msg = rospy.wait_for_message("/red_cubes/found", Bool, timeout=remaining)
        except rospy.ROSException:
            return False
        if bool(msg.data):
            return True
    return False


def _is_ok(result: str) -> bool:
    return str(result).startswith("OK:")


def _xy_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _extract_latest_cube_xy(timeout_s: float = 0.5) -> Optional[Tuple[float, float]]:
    """Read the latest detector pose and return map XY, if available."""
    ensure_rospy()
    try:
        msg = rospy.wait_for_message("/red_cubes/latest_pose", PoseStamped, timeout=float(timeout_s))
    except Exception:
        return None
    return (float(msg.pose.position.x), float(msg.pose.position.y))


def _wait_for_unique_cube(
    detection_timeout_s: float,
    known_cubes_xy: Iterable[Tuple[float, float]],
    dedup_radius_m: float,
) -> Optional[Tuple[float, float]]:
    """Wait for a red-cube pose that is not near already processed cubes."""
    ensure_rospy()
    deadline = time.time() + float(detection_timeout_s)
    known = list(known_cubes_xy)
    while time.time() < deadline and not rospy.is_shutdown():
        remaining = max(0.05, min(1.0, deadline - time.time()))
        xy = _extract_latest_cube_xy(timeout_s=remaining)
        if xy is None:
            continue
        if all(_xy_distance(xy, k) > float(dedup_radius_m) for k in known):
            return xy
    return None


@tool
def fetch_cubes_at_coordinates(coordinates_json: str) -> str:
    """
    Drives to a list of specified map coordinates, attempts a pick, and then a place for each.
    This is a sequence of stub actions to fetch multiple objects.

    The robot will navigate to each coordinate, then perform a pre-canned 'pick' motion
    (closing the gripper at a fixed point in front of the robot), and then a pre-canned
    'place' motion (dropping at a fixed tray location).

    Args:
        coordinates_json: A JSON string representing a list of objects with 'x' and 'y'
                          map coordinates. Example: '[{"x": 1.5, "y": -0.5}, {"x": 1.5, "y": 0.5}]'
    """
    ensure_rospy()
    try:
        coordinates = json.loads(coordinates_json)
        if not isinstance(coordinates, list):
            return "FAIL: coordinates_json must be a JSON array."
    except json.JSONDecodeError:
        return "FAIL: Invalid JSON in coordinates_json argument."

    log = []
    placed_count = 0

    for i, coord in enumerate(coordinates):
        log.append(f"--- Processing cube #{i+1} at {coord} ---")
        x = coord.get("x")
        y = coord.get("y")
        if x is None or y is None:
            log.append(f"FAIL: Cube #{i+1} is missing 'x' or 'y'.")
            continue

        # 1. Navigate to the cube's location.
        nav_result = _go_to_map_pose(x, y, 0.0)  # Navigate to (x,y) with 0 yaw
        log.append(nav_result)
        if "FAIL" in nav_result:
            log.append("Skipping pick/place due to navigation failure.")
            continue

        # 2. Attempt to pick the object.
        # To make the 'pick' service work, we publish a dummy pose in front of the robot.
        log.append(_publish_dummy_pick_pose())
        pick_result = _trigger("/arm_control/pick")
        log.append(pick_result)

        # 3. Attempt to place the object.
        # The existing `/arm_control/place` service moves to a fixed tray location, which is a good stub.
        if "OK" in pick_result:
            place_result = _trigger("/arm_control/place")
            log.append(place_result)
            if "OK" in place_result:
                placed_count += 1
        else:
            log.append("Skipping place because pick failed.")

    header = f"Mission complete. Attempted to fetch {len(coordinates)} cubes. Successfully placed {placed_count}."
    return header + "\n" + "\n".join(log)


@tool
def fetch_red_cubes(
    delivery_x: float = 0.0,
    delivery_y: float = 0.0,
    delivery_yaw_deg: float = 0.0,
    max_cubes: int = 2,
    detection_timeout_s: float = 90.0,
    dedup_radius_m: float = 0.10,
    stop_after_no_new_cubes: bool = True,
) -> str:
    """Full mission: explore, find red cubes, pick, deliver, place.

    Sequence for each cube (up to ``max_cubes``):

    1. Enable the red cube detector.
    2. Start frontier exploration.
    3. Wait for ``/red_cubes/found`` to go True (up to
       ``detection_timeout_s`` seconds).
    4. Stop exploration, take a fresh pose snapshot.
    5. ``approach_object``, ``pick``, drive to the delivery coordinate,
       and ``place``.

    Args:
        delivery_x: Drop-off X in map frame (m).
        delivery_y: Drop-off Y in map frame (m).
        delivery_yaw_deg: Final orientation at drop-off (deg).
        max_cubes: Cap on how many cubes to fetch in this mission.
        detection_timeout_s: How long to explore before giving up per cube.
        dedup_radius_m: Ignore detections near already processed cubes.
        stop_after_no_new_cubes: Stop early if no unique cube appears.
    """
    ensure_rospy()
    log = [_set_bool_with_retry("/red_cube_detector/enable", True)]
    delivered = 0
    processed = 0
    seen_cubes_xy: list[Tuple[float, float]] = []

    for i in range(int(max_cubes)):
        log.append(_set_bool_with_retry("/exploration_enabled", True))
        candidate_xy = _wait_for_unique_cube(
            detection_timeout_s=detection_timeout_s,
            known_cubes_xy=seen_cubes_xy,
            dedup_radius_m=dedup_radius_m,
        )
        log.append(_set_bool_with_retry("/exploration_enabled", False))
        if candidate_xy is None:
            log.append(f"FAIL: no new cube found within {detection_timeout_s}s")
            if stop_after_no_new_cubes:
                break
            continue

        seen_cubes_xy.append(candidate_xy)
        processed += 1

        snapshot = _trigger("/red_cube_detector/snapshot")
        log.append(snapshot)
        if not _is_ok(snapshot):
            log.append("Skipping cube because snapshot failed.")
            continue

        approach = _trigger("/approach_object/approach")
        log.append(approach)
        if not _is_ok(approach):
            log.append("Skipping cube because approach failed.")
            continue

        pick = _trigger("/arm_control/pick")
        log.append(pick)
        if not _is_ok(pick):
            log.append("Skipping delivery because pick failed.")
            continue

        nav_home = _go_to_map_pose(delivery_x, delivery_y, delivery_yaw_deg)
        log.append(nav_home)
        if not _is_ok(nav_home):
            log.append("Skipping place because navigation to home failed.")
            continue

        rospy.sleep(1.0)
        place = _trigger("/arm_control/place")
        log.append(place)
        if _is_ok(place):
            delivered += 1

    header = f"fetch_red_cubes: delivered {delivered}/{max_cubes}, processed={processed}"
    return header + "\n" + "\n".join(log)


@tool
def fetch_red_cubes_to_start(
    max_cubes: int = 2,
    detection_timeout_s: float = 90.0,
    dedup_radius_m: float = 0.10,
    map_frame: str = "map",
    base_frame: str = "base_link",
) -> str:
    """Capture current map pose as home and fetch red cubes back to that pose."""
    ensure_rospy()
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf)
    try:
        t = tfbuf.lookup_transform(
            str(map_frame),
            str(base_frame),
            rospy.Time(0),
            rospy.Duration(0.5),
        )
    except Exception as exc:
        return (
            f"FAIL: Could not read current pose in '{map_frame}' from '{base_frame}': {exc}"
        )

    q = t.transform.rotation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    yaw_deg = math.degrees(yaw)
    x = float(t.transform.translation.x)
    y = float(t.transform.translation.y)
    prefix = (
        f"Home pose locked at x={x:.3f}, y={y:.3f}, yaw_deg={yaw_deg:.1f}. "
        f"Running mission with max_cubes={int(max_cubes)}."
    )
    result = fetch_red_cubes.func(  # type: ignore[attr-defined]
        delivery_x=x,
        delivery_y=y,
        delivery_yaw_deg=yaw_deg,
        max_cubes=int(max_cubes),
        detection_timeout_s=float(detection_timeout_s),
        dedup_radius_m=float(dedup_radius_m),
    ) if hasattr(fetch_red_cubes, "func") else fetch_red_cubes(
        delivery_x=x,
        delivery_y=y,
        delivery_yaw_deg=yaw_deg,
        max_cubes=int(max_cubes),
        detection_timeout_s=float(detection_timeout_s),
        dedup_radius_m=float(dedup_radius_m),
    )
    return prefix + "\n" + result


@tool
def process_clicked_points(wait_timeout_s: float = 60.0, max_points: int = 10) -> str:
    """Wait for points to be clicked in RViz ('Publish Point') and fetch them one by one.
    
    This tool subscribes to the '/clicked_point' topic. It will wait up to wait_timeout_s 
    for the first point. As points are clicked, the robot will drive to approximately that 
    position, attempt to pick at that explicit 3D coordinate, and place the object. 
    It tackles them one after the other, continuing even if one fails.
    
    Args:
        wait_timeout_s: How long to wait for the FIRST point before giving up.
        max_points: Maximum number of points to process.
    """
    ensure_rospy()

    q = queue.Queue()
    def cb(msg: PointStamped):
        q.put(msg)
        
    sub = rospy.Subscriber("/clicked_point", PointStamped, cb)
    
    log = []
    placed_count = 0
    processed_count = 0
    
    try:
        while processed_count < max_points:
            timeout = float(wait_timeout_s) if processed_count == 0 else 5.0
            try:
                pt = q.get(timeout=timeout)
            except queue.Empty:
                if processed_count == 0:
                    return "FAIL: No points clicked in RViz within the timeout."
                else:
                    log.append("No more points clicked in queue. Finishing up.")
                    break
                    
            processed_count += 1
            log.append(f"--- Processing clicked point #{processed_count} at ({pt.point.x:.2f}, {pt.point.y:.2f}) ---")
            
            # 1. Drive to standoff pose (0.25m away in -x direction, facing +x)
            standoff_x = pt.point.x - 0.25
            standoff_y = pt.point.y
            
            nav_result = _go_to_map_pose(standoff_x, standoff_y, 0.0)
            log.append(nav_result)
            if "FAIL" in nav_result:
                log.append("Skipping pick due to navigation failure.")
                continue
            
            # 2. Pick at explicit pose
            pub = rospy.Publisher("/arm_control/target_pose_override", PoseStamped, queue_size=1, latch=True)
            rospy.sleep(0.2)
            pose = PoseStamped()
            pose.header = pt.header
            pose.pose.position = pt.point
            pose.pose.position.z = 0.05  # Rough height for the cube to avoid hitting the floor
            pose.pose.orientation.w = 1.0
            pub.publish(pose)
            rospy.sleep(0.2)
            
            pick_result = _trigger("/arm_control/pick")
            log.append(pick_result)
            
            # 3. Place
            if "OK" in pick_result:
                place_result = _trigger("/arm_control/place")
                log.append(place_result)
                if "OK" in place_result:
                    placed_count += 1
            else:
                log.append("Skipping place because pick failed.")
                
    finally:
        sub.unregister()

    header = f"Mission complete. Processed {processed_count} clicked points. Successfully placed {placed_count}."
    return header + "\n" + "\n".join(log)


@tool
def explore_and_fetch_all_cubes(
    home_x: float,
    home_y: float,
    home_yaw_deg: float,
    exploration_duration_s: float = 30.0,
    deduplication_radius_m: float = 0.05,
) -> str:
    """
    Explores for a fixed duration, finds all unique red cubes, then fetches them one by one.

    First, it enables the red cube detector and frontier exploration for a set duration
    (default 30s). It listens for detected cubes and stores their poses, ignoring any new
    detections within a 5cm radius of an already-found cube. After exploration, it stops
    and iterates through the list of found cubes. For each cube, it drives to it, attempts
    a pick, drives to the specified home coordinates, and attempts a place. This continues
    until all found cubes have been processed. The robot will attempt to deliver and place
    even if the pick action fails.

    Args:
        home_x: The X coordinate of the home/delivery location in the map frame.
        home_y: The Y coordinate of the home/delivery location in the map frame.
        home_yaw_deg: The final orientation of the robot at the home location in degrees.
        exploration_duration_s: How long to explore for cubes in seconds.
        deduplication_radius_m: The distance to consider two cubes the same.
    """
    ensure_rospy()
    log = []
    found_cubes_poses: list[Pose] = []

    # --- 1. Exploration and Detection Phase ---
    log.append(f"--- Starting exploration for {exploration_duration_s}s ---")

    def cube_callback(msg: PoseStamped):
        # Check for duplicates
        for existing_pose in found_cubes_poses:
            dist = math.sqrt(
                (msg.pose.position.x - existing_pose.position.x) ** 2
                + (msg.pose.position.y - existing_pose.position.y) ** 2
            )
            if dist < deduplication_radius_m:
                return  # It's a duplicate, ignore it.

        # Not a duplicate, add it to the list
        log.append(f"Found new unique cube at (x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f})")
        found_cubes_poses.append(msg.pose)

    sub = rospy.Subscriber("/red_cubes/latest_pose", PoseStamped, cube_callback)
    
    try:
        log.append(_set_bool_with_retry("/red_cube_detector/enable", True))
        log.append(_set_bool_with_retry("/exploration_enabled", True))

        rospy.sleep(float(exploration_duration_s))

        log.append(_set_bool_with_retry("/exploration_enabled", False))
        log.append(_set_bool_with_retry("/red_cube_detector/enable", False))
    finally:
        sub.unregister()

    log.append(f"--- Exploration finished. Found {len(found_cubes_poses)} unique cubes. ---")

    if not found_cubes_poses:
        return "Exploration finished, but no cubes were found." + "\n" + "\n".join(log)

    # --- 2. Fetch and Deliver Phase ---
    delivered_count = 0
    for i, cube_pose in enumerate(found_cubes_poses):
        log.append(f"--- Processing cube #{i+1}/{len(found_cubes_poses)} at (x={cube_pose.position.x:.2f}, y={cube_pose.position.y:.2f}) ---")

        # 1. Drive to a standoff pose near the cube
        standoff_x = cube_pose.position.x - 0.25
        standoff_y = cube_pose.position.y
        yaw_rad = math.atan2(cube_pose.position.y - standoff_y, cube_pose.position.x - standoff_x)
        yaw_deg = math.degrees(yaw_rad)

        nav_result = _go_to_map_pose(standoff_x, standoff_y, yaw_deg)
        log.append(f"Navigation to cube: {nav_result}")
        if "FAIL" in nav_result:
            log.append("Skipping this cube due to navigation failure.")
            continue

        # 2. Attempt to pick the object at its exact pose
        pub = rospy.Publisher("/arm_control/target_pose_override", PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.2)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map" # The detector publishes in map frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose = cube_pose
        pub.publish(pose_msg)
        rospy.sleep(0.2)
        
        pick_result = _trigger("/arm_control/pick")
        log.append(f"Pick attempt: {pick_result}")

        # 3. Drive to home location and 4. Place
        nav_home_result = _go_to_map_pose(home_x, home_y, home_yaw_deg)
        log.append(f"Navigation to home: {nav_home_result}")
        place_result = _trigger("/arm_control/place")
        log.append(f"Place attempt: {place_result}")
        if "OK" in place_result:
            delivered_count += 1

    header = f"Mission complete. Found {len(found_cubes_poses)} cubes. Successfully placed {delivered_count}."
    return header + "\n" + "\n".join(log)
