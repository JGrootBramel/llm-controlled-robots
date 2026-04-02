"""
Map-frame cube approach, RGB-D grasp (via /blue_cube_grasper/execute_grasp), and multi-cube pickup.

Tune the approach segment below for your wheel odometry / floor friction.
Needs `tf2_ros` on the machine running these tools (same Python as rospy, ROS master reachable).
Transforms use tf.transformations-style math only — **no PyKDL / tf2_geometry_msgs** so LangChain venvs
work with a normal ROS Noetic install over the network (rosbridge + shared ROS master is fine).
"""

from __future__ import annotations

import math
import re
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Twist, PointStamped
from std_srvs.srv import Trigger
from langchain.tools import tool
from tf.transformations import euler_from_quaternion

_TF2_IMPORT_ERROR: str = ""
try:
    import tf2_ros
except Exception as _e:  # pragma: no cover
    tf2_ros = None
    _TF2_IMPORT_ERROR = f"{type(_e).__name__}: {_e}"

from ..ros_clients import ensure_rospy

# ---------------------------------------------------------------------------
# Approach tuning (lead developer — adjust for reliable drive-to-cube)
# ---------------------------------------------------------------------------
# Multiplier on map-frame distance (m) → used for the *initial* forward burst after turning.
# Balance: far enough that arm coords stay inside MyCobot ±350 mm; small steps finish the approach.
APPROACH_DRIVE_DISTANCE_SCALE: float = 0.52

# Hard cap on how far (m) the *single* initial burst may travel (after scale).
APPROACH_INITIAL_BURST_MAX_M: float = 0.26

# Linear speed (m/s) for the initial approach segment and for vision-guided stepping.
APPROACH_FORWARD_SPEED_MPS: float = 0.062

# Caps on initial forward segment duration (s) after heading alignment.
APPROACH_INITIAL_DRIVE_MIN_S: float = 0.12
APPROACH_INITIAL_DRIVE_MAX_S: float = 5.0

# Closed-loop turn toward target (map frame)
APPROACH_TURN_ANGULAR_SPEED: float = 0.28
APPROACH_TURN_YAW_TOLERANCE_RAD: float = 0.12
APPROACH_TURN_MAX_S_TOTAL: float = 12.0

# Vision-guided closing: short drives, check range, repeat (no long blind rolls).
SWEET_SPOT_MIN_M: float = 0.18
# Call grasp when planar range in base_link is within this (after turn); too tight → keeps driving into scenery.
SWEET_SPOT_MAX_M: float = 0.36
TOO_CLOSE_M: float = 0.12
# Max forward travel (m) per vision iteration when cube is still beyond SWEET_SPOT_MAX_M.
VISION_STEP_MAX_M: float = 0.08
VISION_STEP_MIN_M: float = 0.035
# Upper bound on one segment’s drive time (s) as a safety cap.
VISION_STEP_TIME_CAP_S: float = 3.5
VISION_MAX_DRIVE_S: float = 35.0
NO_DETECTION_BEFORE_GRASP_S: float = 2.2
# When detection is lost, pause this long (s) before declaring timeout → grasp (no forward motion while blind).
VISION_LOST_SETTLE_S: float = 0.18
# Never drive using a cube pose older than this (s) — stale map→base with no new /cube_map_pose caused ramming.
VISION_POSE_MAX_AGE_S: float = 0.42

_GRASP_SRV = "/blue_cube_grasper/execute_grasp"
_CUBE_POSE_TOPIC = "/blue_cube_grasper/cube_map_pose"


def _tf2_ready() -> bool:
    return tf2_ros is not None


def _tf2_required_message() -> str:
    msg = (
        "Map-frame cube tools need `tf2_ros` importable in *this* Python (ROS Noetic).\n"
        "On Ubuntu 20.04 / Noetic:\n"
        "  sudo apt install -y ros-noetic-tf2-ros\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "Verify: python3 -c \"import tf2_ros; print('OK')\"\n"
        "Use a venv with system ROS packages visible:\n"
        "  python3 -m venv venv --system-site-packages\n"
        "Then run ROSA from a shell with Noetic sourced (e.g. ./scripts/run_rosa_with_ros.sh src/rosa_agent.py)\n"
        "and set ROS_MASTER_URI / ROS_IP to reach the LIMO.\n"
    )
    if _TF2_IMPORT_ERROR:
        msg += f"\n(Actual import failure: {_TF2_IMPORT_ERROR})\n"
    return msg


def _do_transform_point_xyz(
    px: float,
    py: float,
    pz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    tx: float,
    ty: float,
    tz: float,
) -> Tuple[float, float, float]:
    """Apply transform (quaternion q + translation t): p_out = R * p_in + t. Same as tf2 do_transform_point."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)
    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)
    ox = r00 * px + r01 * py + r02 * pz + tx
    oy = r10 * px + r11 * py + r12 * pz + ty
    oz = r20 * px + r21 * py + r22 * pz + tz
    return ox, oy, oz


def _point_stamped_to_base_link(msg: PointStamped, trans_stamped) -> Tuple[float, float, float]:
    """Transform PointStamped into base_link using a TransformStamped from lookup_transform."""
    p = msg.point
    T = trans_stamped.transform
    return _do_transform_point_xyz(
        float(p.x),
        float(p.y),
        float(p.z),
        float(T.rotation.x),
        float(T.rotation.y),
        float(T.rotation.z),
        float(T.rotation.w),
        float(T.translation.x),
        float(T.translation.y),
        float(T.translation.z),
    )


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

    sub = rospy.Subscriber(_CUBE_POSE_TOPIC, PointStamped, cube_cb)
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

    merged = []
    for (x, y) in sorted(found_cubes):
        if not merged:
            merged.append((x, y))
            continue
        d_min = min(math.hypot(x - mx, y - my) for (mx, my) in merged)
        if d_min >= merge_distance_m:
            merged.append((x, y))
    return merged


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _call_grasp() -> str:
    try:
        grasp = rospy.ServiceProxy(_GRASP_SRV, Trigger)
        resp = grasp()
        return resp.message
    except rospy.ServiceException as e:
        return f"Grasp failed: {e}"


def _get_pose_yaw(tfbuf) -> Optional[Tuple[float, float, float]]:
    """Return (rx, ry, yaw) in map, or None."""
    try:
        trans = tfbuf.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return rx, ry, yaw
    except Exception:
        return None


def _turnTowardCube(
    cube_x: float,
    cube_y: float,
    cmd_pub,
    rate,
    tfbuf,
) -> Optional[str]:
    """Align base yaw in map toward (cube_x, cube_y). Returns error string or None."""
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > APPROACH_TURN_MAX_S_TOTAL:
            cmd_pub.publish(Twist())
            return "Approach: timeout turning toward cube (TF or goal issue)."
        pose = _get_pose_yaw(tfbuf)
        if pose is None:
            cmd_pub.publish(Twist())
            return "Error: Could not get robot pose (TF)."
        rx, ry, yaw = pose
        dx = cube_x - rx
        dy = cube_y - ry
        yaw_to = math.atan2(dy, dx)
        err = _wrap_pi(yaw_to - yaw)
        if abs(err) <= APPROACH_TURN_YAW_TOLERANCE_RAD:
            cmd_pub.publish(Twist())
            rospy.sleep(0.15)
            return None
        tw = Twist()
        tw.angular.z = APPROACH_TURN_ANGULAR_SPEED if err > 0 else -APPROACH_TURN_ANGULAR_SPEED
        cmd_pub.publish(tw)
        rate.sleep()


def _drive_forward_distance_m(distance_m: float, cmd_pub, rate) -> None:
    """Drive forward up to distance_m (m), capped by time, then stop briefly for camera."""
    if distance_m <= 0.001:
        return
    v = max(0.04, float(APPROACH_FORWARD_SPEED_MPS))
    d = min(float(distance_m), v * float(VISION_STEP_TIME_CAP_S))
    t = max(float(APPROACH_INITIAL_DRIVE_MIN_S), d / v)
    t = min(t, float(APPROACH_INITIAL_DRIVE_MAX_S), float(VISION_STEP_TIME_CAP_S))
    tw = Twist()
    tw.linear.x = v
    t_end = rospy.Time.now() + rospy.Duration(t)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        cmd_pub.publish(tw)
        rate.sleep()
    cmd_pub.publish(Twist())
    rospy.sleep(float(VISION_LOST_SETTLE_S))


def _initial_forward_burst(distance_m: float, cmd_pub, rate) -> None:
    """Short first leg after heading align: scaled distance, capped — fine closing is vision steps."""
    scale = max(0.0, float(APPROACH_DRIVE_DISTANCE_SCALE))
    cap = max(0.0, float(APPROACH_INITIAL_BURST_MAX_M))
    dist = max(0.0, float(distance_m)) * scale
    dist = min(dist, cap)
    if dist < 0.02:
        return
    _drive_forward_distance_m(dist, cmd_pub, rate)
    rospy.sleep(0.08)


def _approach_one_cube_and_grasp(cube_x: float, cube_y: float, cmd_pub, rate, tfbuf) -> str:
    """
    Turn toward cube → short capped forward leg → vision loop: only drive on *fresh*
    /cube_map_pose (see VISION_POSE_MAX_AGE_S); if the feed goes stale, stop and later call grasp
    instead of using an old range (prevents driving into support boxes). Grasp when within SWEET_SPOT_MAX_M.
    """
    rospy.sleep(0.1)
    pose = _get_pose_yaw(tfbuf)
    if pose is None:
        return "Error: Could not get robot pose (TF)."
    rx, ry, yaw = pose

    dx = cube_x - rx
    dy = cube_y - ry
    distance = math.hypot(dx, dy)
    if distance < 0.08:
        return _call_grasp()

    err = _turnTowardCube(cube_x, cube_y, cmd_pub, rate, tfbuf)
    if err:
        return err

    pose2 = _get_pose_yaw(tfbuf)
    if pose2 is None:
        return "Error: Could not get robot pose after turn (TF)."
    rx2, ry2, _ = pose2
    dx2 = cube_x - rx2
    dy2 = cube_y - ry2
    distance2 = math.hypot(dx2, dy2)
    _initial_forward_burst(distance2, cmd_pub, rate)

    last_cube_in_base: list = [None]
    last_msg_time: list = [None]

    def cb(msg: PointStamped):
        try:
            t = tfbuf.lookup_transform("base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
            bx, by, bz = _point_stamped_to_base_link(msg, t)
            last_cube_in_base[0] = (bx, by, bz)
            last_msg_time[0] = rospy.Time.now()
        except Exception:
            pass

    sub = rospy.Subscriber(_CUBE_POSE_TOPIC, PointStamped, cb)
    rospy.sleep(0.4)
    drive_start = rospy.Time.now()

    vmax = float(VISION_STEP_MAX_M)
    vmin = float(VISION_STEP_MIN_M)

    pose_max_age = rospy.Duration(float(VISION_POSE_MAX_AGE_S))

    while not rospy.is_shutdown():
        if rospy.Time.now() - drive_start > rospy.Duration(VISION_MAX_DRIVE_S):
            cmd_pub.publish(Twist())
            sub.unregister()
            return "Stopped: vision approach timeout (no sweet spot)."
        now = rospy.Time.now()
        pos = last_cube_in_base[0]
        mt = last_msg_time[0]
        fresh = mt is not None and (now - mt) <= pose_max_age

        if pos is not None and fresh:
            d = math.hypot(pos[0], pos[1])
            if d <= TOO_CLOSE_M:
                cmd_pub.publish(Twist())
                sub.unregister()
                return f"Stopped (cube very close). {_call_grasp()}"
            if d <= SWEET_SPOT_MAX_M:
                cmd_pub.publish(Twist())
                sub.unregister()
                rospy.sleep(0.25)
                return _call_grasp()
            gap = d - float(SWEET_SPOT_MAX_M)
            step_m = min(vmax, max(vmin, gap * 0.4))
            _drive_forward_distance_m(step_m, cmd_pub, rate)
        else:
            cmd_pub.publish(Twist())
            if mt is not None and (now - mt) > rospy.Duration(
                NO_DETECTION_BEFORE_GRASP_S
            ):
                sub.unregister()
                return (
                    f"No fresh cube pose for {NO_DETECTION_BEFORE_GRASP_S:.0f}s; "
                    f"attempting grasp (avoid stale drive). {_call_grasp()}"
                )
            rospy.sleep(float(VISION_LOST_SETTLE_S))

    sub.unregister()
    return "Interrupted."


@tool
def scan_for_blue_cubes(duration_seconds: int = 25, spin: bool = True) -> str:
    """
    Scan for colored cubes (HSV on robot: see blue_cube_grasper target_color, often red).
    Optionally spin the base. Returns merged map (x, y) coordinates from /blue_cube_grasper/cube_map_pose.
    """
    merged = _run_scan(float(duration_seconds), spin, merge_distance_m=0.05)
    if not merged:
        return "Scan complete. No cubes detected."
    result = "Scan complete. Found cubes at the following map coordinates:\n"
    for idx, (x, y) in enumerate(merged):
        result += f"- Cube {idx + 1}: x={x}, y={y}\n"
    return result


def _start_approach_session():
    if not _tf2_ready():
        return None, _tf2_required_message()
    ensure_rospy()
    tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2_ros.TransformListener(tfbuf)
    rospy.sleep(0.4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(20)
    return (tfbuf, cmd_pub, rate), None


@tool
def fetch_and_store_cube(cube_x: float, cube_y: float) -> str:
    """
    Turn toward map (cube_x, cube_y), drive a distance scaled by APPROACH_DRIVE_DISTANCE_SCALE,
    then use RGB-D grasper (execute_grasp) to pick and place left of the base.

    Requires tf2 on this machine (ROS-sourced environment).
    """
    sess = _start_approach_session()
    if sess[0] is None:
        return sess[1]
    tfbuf, cmd_pub, rate = sess[0]
    return _approach_one_cube_and_grasp(float(cube_x), float(cube_y), cmd_pub, rate, tfbuf)


def _parse_map_coordinate_pairs(coordinates: str) -> List[Tuple[float, float]]:
    """
    Parse "x1,y1; x2,y2" or "x1 y1, x2 y2" — semicolon between pairs, comma or space inside pair.
    """
    s = coordinates.strip()
    if not s:
        return []
    pairs: List[Tuple[float, float]] = []
    for chunk in re.split(r"[;]+", s):
        chunk = chunk.strip()
        if not chunk:
            continue
        nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", chunk)
        if len(nums) < 2:
            continue
        pairs.append((float(nums[0]), float(nums[1])))
    return pairs


@tool
def pick_up_cubes_at_map_positions(coordinates: str, max_cubes: int = 12) -> str:
    """
    Pick up several cubes in sequence. Pass map coordinates as:
    "x1,y1; x2,y2; x3,y3" (semicolon-separated pairs).

    For each pair: turn toward the cube, drive scaled distance, vision refine, grasp, place left of base.
    max_cubes caps how many pairs are processed (safety).
    """
    if not _tf2_ready():
        return _tf2_required_message()
    pts = _parse_map_coordinate_pairs(coordinates)
    if not pts:
        return (
            "No valid coordinates. Use semicolon-separated pairs, e.g. "
            "'0.6,-0.2; 0.7,-0.1'."
        )
    cap = max(1, min(int(max_cubes), 50))
    pts = pts[:cap]

    sess = _start_approach_session()
    if sess[0] is None:
        return sess[1]
    tfbuf, cmd_pub, rate = sess[0]

    lines = []
    for idx, (cx, cy) in enumerate(pts):
        msg = _approach_one_cube_and_grasp(cx, cy, cmd_pub, rate, tfbuf)
        lines.append(f"Cube {idx + 1} ({cx}, {cy}): {msg}")
        rospy.sleep(0.6)
    return "Multi-cube run finished.\n" + "\n".join(lines)


@tool
def pick_up_cubes_in_area(duration_seconds: int = 15, spin: bool = True) -> str:
    """
    Scan for cubes, then collect each detected map position (same pipeline as fetch_and_store_cube).
    """
    if not _tf2_ready():
        return _tf2_required_message()
    merged = _run_scan(float(duration_seconds), spin, merge_distance_m=0.05)
    if not merged:
        return "Scan complete. No cubes detected; nothing to pick up."

    sess = _start_approach_session()
    if sess[0] is None:
        return sess[1]
    tfbuf, cmd_pub, rate = sess[0]

    results = []
    for idx, (cx, cy) in enumerate(merged):
        msg = _approach_one_cube_and_grasp(cx, cy, cmd_pub, rate, tfbuf)
        results.append(f"Cube {idx + 1} ({cx}, {cy}): {msg}")
        rospy.sleep(0.6)

    return "Collect run finished.\n" + "\n".join(results)
