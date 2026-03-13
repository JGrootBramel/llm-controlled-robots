import rospy
import math
from geometry_msgs.msg import Twist, PointStamped
from std_srvs.srv import Trigger
from langchain.tools import tool
from tf.transformations import euler_from_quaternion

try:
    import tf2_ros
    import tf2_geometry_msgs
except Exception:  # pragma: no cover - allows remote PC without ROS tf2/PyKDL
    tf2_ros = None
    tf2_geometry_msgs = None

from ..ros_clients import ensure_rospy

# --- Shared scan + merge (returns list of (x,y) in map frame) ---
def _run_scan(duration_seconds: float, spin: bool, merge_distance_m: float = 0.05):
    """Run cube scan, return merged list of (x, y) map coordinates."""
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    found_cubes = set()

    def cube_cb(msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        r = (x * x + y * y) ** 0.5
        if r < 0.2 or r > 1.5:
            return
        if abs(y) > 0.5:
            return
        gx, gy = round(x, 1), round(y, 1)
        found_cubes.add((gx, gy))

    sub = rospy.Subscriber("/blue_cube_grasper/cube_map_pose", PointStamped, cube_cb)
    tw = Twist()
    if spin:
        tw.angular.z = 0.2
    end_time = rospy.Time.now() + rospy.Duration(duration_seconds)
    rate = rospy.Rate(10)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        if spin:
            cmd_pub.publish(tw)
        rate.sleep()
    if spin:
        cmd_pub.publish(Twist())
    sub.unregister()

    # Merge cubes closer than merge_distance_m (2D)
    merged = []
    for (x, y) in sorted(found_cubes):
        if not merged:
            merged.append((x, y))
            continue
        d_min = min(math.hypot(x - mx, y - my) for (mx, my) in merged)
        if d_min >= merge_distance_m:
            merged.append((x, y))
    return merged


@tool
def scan_for_blue_cubes(duration_seconds: int = 25, spin: bool = True) -> str:
    """
    Scan for cubes (optionally while spinning). Returns list of unique cube (x, y) map coordinates.
    Cubes closer than 5 cm are merged into one. Use spin=False to scan without rotating.
    """
    merged = _run_scan(float(duration_seconds), spin, merge_distance_m=0.05)
    if not merged:
        return "Scan complete. No cubes detected."
    result = "Scan complete. Found cubes at the following map coordinates:\n"
    for idx, (x, y) in enumerate(merged):
        result += f"- Cube {idx + 1}: x={x}, y={y}\n"
    return result


def _approach_one_cube_and_grasp(cube_x: float, cube_y: float, cmd_pub, rate, tfbuf) -> str:
    """Orient toward cube, drive to sweet spot (checking every ~0.5 s), then call execute_grasp."""
    rospy.sleep(0.2)
    try:
        trans = tfbuf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    except Exception:
        return "Error: Could not get robot pose (TF)."

    dx = cube_x - rx
    dy = cube_y - ry
    distance = math.hypot(dx, dy)
    if distance < 0.08:
        # Already very close; try grasp immediately
        try:
            grasp = rospy.ServiceProxy("/blue_cube_grasper/execute_grasp", Trigger)
            resp = grasp()
            return resp.message
        except rospy.ServiceException as e:
            return f"Grasp failed: {e}"

    # 1) Orient toward cube
    yaw_to_cube = math.atan2(dy, dx)
    yaw_error = yaw_to_cube - yaw
    while yaw_error > math.pi:
        yaw_error -= 2 * math.pi
    while yaw_error < -math.pi:
        yaw_error += 2 * math.pi
    omega = 0.3
    turn_time = abs(yaw_error) / omega
    tw = Twist()
    tw.angular.z = 0.3 if yaw_error >= 0 else -0.3
    start = rospy.Time.now()
    while (rospy.Time.now() - start) < rospy.Duration(turn_time) and not rospy.is_shutdown():
        cmd_pub.publish(tw)
        rate.sleep()
    cmd_pub.publish(Twist())
    rospy.sleep(0.3)

    # 2) Drive forward in steps; every ~0.5 s check distance to cube (from last cube_map_pose in base_link)
    SWEET_SPOT_MIN = 0.18   # m in front of robot
    SWEET_SPOT_MAX = 0.42
    TOO_CLOSE = 0.12        # depth often lost
    STEP = 0.5
    v = 0.06
    last_cube_in_base = [None]  # [ (x,y,z) in base_link or None ]
    last_msg_time = [None]

    def cb(msg: PointStamped):
        try:
            t = tfbuf.lookup_transform("base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
            pt = tf2_geometry_msgs.do_transform_point(msg, t).point
            last_cube_in_base[0] = (pt.x, pt.y, pt.z)
            last_msg_time[0] = rospy.Time.now()
        except Exception:
            pass

    sub = rospy.Subscriber("/blue_cube_grasper/cube_map_pose", PointStamped, cb)
    rospy.sleep(0.5)
    no_detection_timeout = rospy.Duration(2.0)
    max_drive_time = rospy.Duration(25.0)
    drive_start = rospy.Time.now()

    while not rospy.is_shutdown():
        if rospy.Time.now() - drive_start > max_drive_time:
            cmd_pub.publish(Twist())
            sub.unregister()
            return "Stopped: max drive time reached without reaching sweet spot."
        now = rospy.Time.now()
        pos = last_cube_in_base[0]
        if pos is not None:
            d = math.hypot(pos[0], pos[1])
            if d <= TOO_CLOSE:
                cmd_pub.publish(Twist())
                sub.unregister()
                try:
                    grasp = rospy.ServiceProxy("/blue_cube_grasper/execute_grasp", Trigger)
                    resp = grasp()
                    return f"Stopped (cube very close). {resp.message}"
                except rospy.ServiceException as e:
                    return f"Stopped. Grasp failed: {e}"
            if SWEET_SPOT_MIN <= d <= SWEET_SPOT_MAX:
                cmd_pub.publish(Twist())
                sub.unregister()
                rospy.sleep(0.3)
                try:
                    grasp = rospy.ServiceProxy("/blue_cube_grasper/execute_grasp", Trigger)
                    resp = grasp()
                    return resp.message
                except rospy.ServiceException as e:
                    return f"Grasp failed: {e}"
            last_ok = now
        else:
            if last_msg_time[0] is not None and (now - last_msg_time[0]) > no_detection_timeout:
                cmd_pub.publish(Twist())
                sub.unregister()
                try:
                    grasp = rospy.ServiceProxy("/blue_cube_grasper/execute_grasp", Trigger)
                    resp = grasp()
                    return f"No detection for 2s; attempted grasp. {resp.message}"
                except rospy.ServiceException as e:
                    return f"No detection; grasp failed: {e}"

        # Drive forward for this step
        tw = Twist()
        tw.linear.x = v
        end_step = now + rospy.Duration(STEP)
        while rospy.Time.now() < end_step and not rospy.is_shutdown():
            cmd_pub.publish(tw)
            rate.sleep()
        cmd_pub.publish(Twist())
        rate.sleep()

    sub.unregister()
    return "Interrupted."


@tool
def fetch_and_store_cube(cube_x: float, cube_y: float) -> str:
    """
    Approach one cube at map (cube_x, cube_y): orient toward it, drive to sweet spot, then grasp and place on tray.
    You can use this with coordinates from a prior scan or given by the user.
    """
    if tf2_ros is None or tf2_geometry_msgs is None:
        return (
            "This tool requires ROS tf2 (tf2_ros, tf2_geometry_msgs, PyKDL) "
            "and must be run on the robot ROS environment."
        )
    ensure_rospy()
    tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2_ros.TransformListener(tfbuf)
    rospy.sleep(0.5)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)
    return _approach_one_cube_and_grasp(float(cube_x), float(cube_y), cmd_pub, rate, tfbuf)


@tool
def pick_up_cubes_in_area(duration_seconds: int = 15, spin: bool = True) -> str:
    """
    Scan for cubes, then collect them one by one: for each cube, orient toward it, drive to sweet spot, grasp and place on tray.
    Say e.g. 'Pick up the cubes in the area' to run this. Cubes closer than 5 cm are merged into one.
    """
    if tf2_ros is None or tf2_geometry_msgs is None:
        return (
            "This tool requires ROS tf2 (tf2_ros, tf2_geometry_msgs, PyKDL) "
            "and must be run on the robot ROS environment."
        )
    merged = _run_scan(float(duration_seconds), spin, merge_distance_m=0.05)
    if not merged:
        return "Scan complete. No cubes detected; nothing to pick up."

    ensure_rospy()
    tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2_ros.TransformListener(tfbuf)
    rospy.sleep(0.5)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)

    results = []
    for idx, (cx, cy) in enumerate(merged):
        msg = _approach_one_cube_and_grasp(cx, cy, cmd_pub, rate, tfbuf)
        results.append(f"Cube {idx + 1} ({cx}, {cy}): {msg}")
        rospy.sleep(0.5)

    return "Collect run finished.\n" + "\n".join(results)
