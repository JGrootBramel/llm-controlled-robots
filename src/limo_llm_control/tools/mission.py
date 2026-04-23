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

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from langchain.tools import tool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler

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


def _go_to_map_pose(x: float, y: float, yaw_deg: float) -> str:
    """Navigates the robot to a map pose and waits for completion."""
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
    max_cubes: int = 1,
    detection_timeout_s: float = 90.0,
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
    """
    ensure_rospy()
    log = [_set_bool("/red_cube_detector/enable", True)]
    delivered = 0

    for i in range(int(max_cubes)):
        log.append(_set_bool("/exploration_enabled", True))
        found = _wait_for_found(detection_timeout_s)
        log.append(_set_bool("/exploration_enabled", False))
        if not found:
            log.append(f"FAIL: cube #{i + 1} not found within {detection_timeout_s}s")
            break

        log.append(_trigger("/red_cube_detector/snapshot"))
        log.append(_trigger("/approach_object/approach"))
        log.append(_trigger("/arm_control/pick"))
        log.append(_publish_delivery_goal(delivery_x, delivery_y, delivery_yaw_deg))
        rospy.sleep(1.0)
        log.append(_trigger("/arm_control/place"))
        delivered += 1

    header = f"fetch_red_cubes: delivered {delivered}/{max_cubes}"
    return header + "\n" + "\n".join(log)


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
