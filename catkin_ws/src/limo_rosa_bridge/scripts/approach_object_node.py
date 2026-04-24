#!/usr/bin/env python3
"""Drive the base up to a target pose and stop at a configurable standoff.

Flow (triggered by ``~approach`` service):

1. Look at the latest pose on ``~target_pose`` (default ``/red_cubes/latest_pose``).
2. Compute a safe standoff pose behind the target using
   :func:`_helpers.standoff.standoff_candidates`.
3. Send the first reachable standoff as a ``MoveBaseGoal`` and wait.
4. Optionally (``~close_in_enabled``), if we are still too far (planar
   distance) from the target once ``move_base`` succeeded, creep forward on
   ``/cmd_vel`` until we are at ``~min_standoff`` from it. When
   ``~close_in_guard_enabled`` is true, this runs only while front lidar
   clearance is safe.
5. Yaw-align on the target so the arm/camera face it squarely.

Service:

* ``~approach``  ``std_srvs/Trigger``  — run the flow (blocking).
* ``~cancel``    ``std_srvs/Trigger``  — cancel any in-flight move_base goal.

This is the same logic as the old ``object_finder_node`` FSM, minus the
detection and manipulation concerns.
"""

import math
import os
import sys
from threading import Event, Lock

import actionlib
import rospy
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_pose/point)
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _helpers import standoff as standoff_helpers  # noqa: E402


class ApproachObject:
    def __init__(self):
        self._load_params()
        self._state_lock = Lock()
        self._latest_pose = None  # PoseStamped in map frame
        self._latest_scan = None  # LaserScan on scan_topic

        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber(
            self.target_topic, PoseStamped, self._on_target, queue_size=1
        )
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)

        self.mb = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("[approach_object] waiting for /move_base...")
        self.mb.wait_for_server()
        rospy.loginfo("[approach_object] /move_base connected")

        rospy.Service("~approach", Trigger, self._handle_approach)
        rospy.Service("~cancel", Trigger, self._handle_cancel)

    # ------------------------------------------------------------ params
    def _load_params(self):
        self.target_topic = rospy.get_param("~target_topic", "/red_cubes/latest_pose")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.standoff = float(rospy.get_param("~standoff", 0.45))
        self.min_standoff = float(rospy.get_param("~min_standoff", 0.35))
        self.standoff_arc_deg = float(rospy.get_param("~standoff_arc_deg", 10.0))
        self.standoff_max_deg = float(rospy.get_param("~standoff_max_deg", 90.0))
        self.close_in_speed = float(rospy.get_param("~close_in_speed", 0.08))
        self.close_in_enabled = bool(rospy.get_param("~close_in_enabled", True))
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.close_in_guard_enabled = bool(
            rospy.get_param("~close_in_guard_enabled", True)
        )
        self.close_in_guard_distance_m = float(
            rospy.get_param("~close_in_guard_distance_m", 0.35)
        )
        self.close_in_guard_half_angle_deg = float(
            rospy.get_param("~close_in_guard_half_angle_deg", 18.0)
        )
        self.close_in_guard_min_valid_beams = int(
            rospy.get_param("~close_in_guard_min_valid_beams", 3)
        )
        self.close_in_guard_scan_stale_s = float(
            rospy.get_param("~close_in_guard_scan_stale_s", 0.5)
        )
        self.align_angular_speed = float(
            rospy.get_param("~align_angular_speed", 0.25)
        )
        self.align_tol_rad = float(rospy.get_param("~align_tol_rad", 0.05))
        self.move_base_timeout_s = float(
            rospy.get_param("~move_base_timeout_s", 60.0)
        )
        self.close_in_timeout_s = float(rospy.get_param("~close_in_timeout_s", 15.0))
        self.close_in_without_standoff_enabled = bool(
            rospy.get_param("~close_in_without_standoff_enabled", True)
        )
        self.close_in_without_standoff_max_dist_m = float(
            rospy.get_param("~close_in_without_standoff_max_dist_m", 1.2)
        )

    # ---------------------------------------------------------- callbacks
    def _on_target(self, msg):
        with self._state_lock:
            self._latest_pose = msg

    def _handle_cancel(self, _req):
        self.mb.cancel_all_goals()
        self.pub_cmd.publish(Twist())
        return TriggerResponse(success=True, message="cancelled")

    def _on_scan(self, msg):
        with self._state_lock:
            self._latest_scan = msg

    def _handle_approach(self, _req):
        with self._state_lock:
            target = self._latest_pose
        if target is None:
            return TriggerResponse(
                success=False,
                message=f"No pose received on {self.target_topic} yet.",
            )
        target_map = self._to_map_frame(target)
        if target_map is None:
            return TriggerResponse(
                success=False, message="Failed to transform target into map frame."
            )

        current_d = self._planar_distance_to(target_map)
        standoff_pose = None
        if current_d is not None and current_d <= self.min_standoff:
            rospy.loginfo(
                "[approach_object] already within min_standoff (d=%.2f <= %.2f), "
                "skipping move_base standoff.",
                current_d,
                self.min_standoff,
            )
        else:
            ok, standoff_pose = self._send_standoff(target_map)
            if not ok:
                if self.close_in_without_standoff_enabled:
                    fallback_ok = self._close_in_without_standoff(target_map)
                    if not fallback_ok:
                        return TriggerResponse(
                            success=False,
                            message=(
                                "No reachable standoff goal found; move_base could not plan "
                                "and close-in fallback failed."
                            ),
                        )
                    rospy.logwarn(
                        "[approach_object] move_base standoff failed; recovered with "
                        "close-in fallback."
                    )
                else:
                    return TriggerResponse(
                        success=False,
                        message="No reachable standoff goal found; move_base could not plan.",
                    )

        if self.close_in_enabled:
            close_ok = self._close_in_if_needed(target_map)
            if not close_ok:
                return TriggerResponse(
                    success=False, message="Close-in driving failed or timed out."
                )
        else:
            rospy.loginfo("[approach_object] close-in disabled; holding standoff pose.")

        aligned = self._yaw_align(target_map)
        if not aligned:
            return TriggerResponse(
                success=False, message="Yaw align failed; check TF stream."
            )

        return TriggerResponse(
            success=True,
            message=(
                f"Approached target at ({target_map.pose.position.x:.2f}, "
                f"{target_map.pose.position.y:.2f}) from standoff "
                f"({standoff_pose.pose.position.x:.2f}, "
                f"{standoff_pose.pose.position.y:.2f})."
                if standoff_pose is not None
                else (
                    f"Approached target at ({target_map.pose.position.x:.2f}, "
                    f"{target_map.pose.position.y:.2f}) without a new standoff move."
                )
            ),
        )

    # ---------------------------------------------------- step 1: standoff
    def _send_standoff(self, target_map):
        tx = target_map.pose.position.x
        ty = target_map.pose.position.y
        base_yaw = self._robot_yaw_towards(tx, ty)
        for distance_m in self._standoff_distances():
            cands = standoff_helpers.standoff_candidates(
                tx,
                ty,
                base_yaw,
                distance_m=distance_m,
                arc_deg=self.standoff_arc_deg,
                max_arc_deg=self.standoff_max_deg,
            )
            for (gx, gy, yaw) in cands:
                goal_pose = self._make_goal_pose(gx, gy, yaw)
                goal = MoveBaseGoal()
                goal.target_pose = goal_pose
                rospy.loginfo(
                    "[approach_object] Sending standoff goal (%.2f, %.2f, yaw=%.2f, d=%.2f)",
                    gx,
                    gy,
                    yaw,
                    distance_m,
                )
                self.mb.send_goal(goal)
                finished = self.mb.wait_for_result(
                    rospy.Duration(self.move_base_timeout_s)
                )
                status = self.mb.get_state()
                if finished and status == GoalStatus.SUCCEEDED:
                    return True, goal_pose
                rospy.logwarn(
                    "[approach_object] standoff (%.2f, %.2f, d=%.2f) not reached (status=%s), "
                    "trying next candidate.",
                    gx,
                    gy,
                    distance_m,
                    status,
                )
                self.mb.cancel_all_goals()
        return False, None

    def _standoff_distances(self):
        """Try conservative standoff radii from farther to closer."""
        values = [float(self.standoff)]
        if self.min_standoff < self.standoff:
            mid = (self.standoff + self.min_standoff) * 0.5
            values.extend([mid, float(self.min_standoff)])
        # Deduplicate while preserving order and keeping physically valid radii.
        seen = set()
        ordered = []
        for v in values:
            r = max(0.05, round(float(v), 3))
            if r not in seen:
                seen.add(r)
                ordered.append(r)
        return ordered

    # ------------------------------------------------- step 2: close-in drive
    def _close_in_if_needed(self, target_map):
        deadline = rospy.Time.now() + rospy.Duration(self.close_in_timeout_s)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logwarn("[approach_object] close-in timed out.")
                self.pub_cmd.publish(Twist())
                return False
            d = self._planar_distance_to(target_map)
            if d is None:
                rate.sleep()
                continue
            if d <= self.min_standoff:
                self.pub_cmd.publish(Twist())
                return True
            if self.close_in_guard_enabled:
                guard_ok, why = self._front_clearance_ok()
                if not guard_ok:
                    rospy.logwarn("[approach_object] close-in blocked: %s", why)
                    self.pub_cmd.publish(Twist())
                    return False
            tw = Twist()
            tw.linear.x = float(self.close_in_speed)
            self.pub_cmd.publish(tw)
            rate.sleep()
        self.pub_cmd.publish(Twist())
        return False

    def _close_in_without_standoff(self, target_map):
        d0 = self._planar_distance_to(target_map)
        if d0 is None:
            rospy.logwarn("[approach_object] close-in fallback: no TF distance to target.")
            return False
        if d0 > self.close_in_without_standoff_max_dist_m:
            rospy.logwarn(
                "[approach_object] close-in fallback disabled at long range "
                "(d=%.2f > %.2f).",
                d0,
                self.close_in_without_standoff_max_dist_m,
            )
            return False

        deadline = rospy.Time.now() + rospy.Duration(self.close_in_timeout_s)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                self.pub_cmd.publish(Twist())
                rospy.logwarn("[approach_object] close-in fallback timed out.")
                return False

            d = self._planar_distance_to(target_map)
            if d is None:
                rate.sleep()
                continue
            if d <= self.min_standoff:
                self.pub_cmd.publish(Twist())
                return True

            if self.close_in_guard_enabled:
                guard_ok, why = self._front_clearance_ok()
                if not guard_ok:
                    self.pub_cmd.publish(Twist())
                    rospy.logwarn(
                        "[approach_object] close-in fallback blocked by guard: %s", why
                    )
                    return False

            err = self._bearing_error_to_target(target_map)
            if err is None:
                rate.sleep()
                continue

            tw = Twist()
            if abs(err) > self.align_tol_rad:
                tw.angular.z = float(
                    self.align_angular_speed if err > 0 else -self.align_angular_speed
                )
            else:
                tw.linear.x = float(self.close_in_speed)
            self.pub_cmd.publish(tw)
            rate.sleep()

        self.pub_cmd.publish(Twist())
        return False

    # ---------------------------------------------------- step 3: yaw align
    def _yaw_align(self, target_map):
        rate = rospy.Rate(20.0)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                self.pub_cmd.publish(Twist())
                return False
            try:
                T = self.tfbuf.lookup_transform(
                    self.map_frame, self.base_frame,
                    rospy.Time(0), rospy.Duration(0.2),
                )
            except Exception:
                rate.sleep()
                continue
            rx = T.transform.translation.x
            ry = T.transform.translation.y
            tx = target_map.pose.position.x
            ty = target_map.pose.position.y
            desired = math.atan2(ty - ry, tx - rx)
            current = _yaw_from_quat(T.transform.rotation)
            err = _angle_diff(desired, current)
            if abs(err) <= self.align_tol_rad:
                self.pub_cmd.publish(Twist())
                return True
            tw = Twist()
            tw.angular.z = float(
                self.align_angular_speed if err > 0 else -self.align_angular_speed
            )
            self.pub_cmd.publish(tw)
            rate.sleep()
        self.pub_cmd.publish(Twist())
        return False

    # ---------------------------------------------------------- utilities
    def _to_map_frame(self, pose):
        if pose.header.frame_id == self.map_frame:
            return pose
        try:
            T = self.tfbuf.lookup_transform(
                self.map_frame, pose.header.frame_id,
                rospy.Time(0), rospy.Duration(0.3),
            )
            return tf2_geometry_msgs.do_transform_pose(pose, T)
        except Exception as e:
            rospy.logwarn("[approach_object] TF to map failed: %s", e)
            return None

    def _planar_distance_to(self, target_map):
        try:
            T = self.tfbuf.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
        except Exception:
            return None
        dx = target_map.pose.position.x - T.transform.translation.x
        dy = target_map.pose.position.y - T.transform.translation.y
        return math.hypot(dx, dy)

    def _robot_yaw_towards(self, tx, ty):
        try:
            T = self.tfbuf.lookup_transform(
                self.map_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
            return math.atan2(ty - T.transform.translation.y,
                              tx - T.transform.translation.x)
        except Exception:
            return 0.0

    def _bearing_error_to_target(self, target_map):
        try:
            T = self.tfbuf.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2)
            )
        except Exception:
            return None
        rx = T.transform.translation.x
        ry = T.transform.translation.y
        tx = target_map.pose.position.x
        ty = target_map.pose.position.y
        desired = math.atan2(ty - ry, tx - rx)
        current = _yaw_from_quat(T.transform.rotation)
        return _angle_diff(desired, current)

    def _make_goal_pose(self, gx, gy, yaw):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _front_clearance_ok(self):
        with self._state_lock:
            scan = self._latest_scan
        if scan is None:
            return False, f"no LaserScan yet on {self.scan_topic}"

        age = (rospy.Time.now() - scan.header.stamp).to_sec()
        if scan.header.stamp.to_sec() <= 0.0:
            age = 0.0
        if age > self.close_in_guard_scan_stale_s:
            return False, f"scan stale ({age:.2f}s > {self.close_in_guard_scan_stale_s:.2f}s)"

        cone = math.radians(max(1.0, self.close_in_guard_half_angle_deg))
        min_clear = self.close_in_guard_distance_m
        valid = 0
        nearest = float("inf")

        effective_far = (
            scan.range_max if (scan.range_max and scan.range_max > 0.0) else 10.0
        )
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            if abs(ang) > cone:
                continue
            # Laser drivers often publish +inf for "no return" (clear to max range).
            if not math.isfinite(r):
                r = effective_far
            if scan.range_min > 0.0 and r < scan.range_min:
                continue
            if scan.range_max > 0.0 and r > scan.range_max:
                continue
            valid += 1
            if r < nearest:
                nearest = r

        if valid < self.close_in_guard_min_valid_beams:
            return False, (
                f"insufficient valid beams in forward cone ({valid} < "
                f"{self.close_in_guard_min_valid_beams})"
            )
        if nearest < min_clear:
            return False, f"obstacle at {nearest:.2f}m < guard {min_clear:.2f}m"
        return True, "clear"


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def _angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    rospy.init_node("approach_object")
    ApproachObject()
    rospy.loginfo("[approach_object] ready")
    rospy.spin()
