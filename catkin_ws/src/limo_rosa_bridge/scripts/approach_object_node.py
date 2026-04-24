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
from nav_msgs.srv import GetPlan
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
        self.precheck_plan_service = bool(
            rospy.get_param("~precheck_plan_service", True)
        )
        self.close_in_timeout_s = float(rospy.get_param("~close_in_timeout_s", 15.0))
        self.final_extra_forward_m = float(
            rospy.get_param("~final_extra_forward_m", 0.05)
        )
        self.guard_blocked_success_distance_m = float(
            rospy.get_param("~guard_blocked_success_distance_m", 0.33)
        )
        self.close_in_without_standoff_enabled = bool(
            rospy.get_param("~close_in_without_standoff_enabled", True)
        )
        self.close_in_without_standoff_max_dist_m = float(
            rospy.get_param("~close_in_without_standoff_max_dist_m", 1.2)
        )
        self.fallback_force_closein_near_target_m = float(
            rospy.get_param("~fallback_force_closein_near_target_m", 0.30)
        )
        self.fallback_force_closein_speed = float(
            rospy.get_param("~fallback_force_closein_speed", 0.04)
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

        close_ok, end_target, fail_reason = self._simple_approach_to_target(target_map)
        if not close_ok:
            return TriggerResponse(
                success=False,
                message=f"Simple close-in failed: {fail_reason}",
            )

        aligned = self._yaw_align(end_target)
        if not aligned:
            return TriggerResponse(
                success=False, message="Yaw align failed; check TF stream."
            )

        return TriggerResponse(
            success=True,
            message=(
                f"Approached target at ({end_target.pose.position.x:.2f}, "
                f"{end_target.pose.position.y:.2f}, {end_target.pose.position.z:.2f}) "
                f"(guard_success_d={self.guard_blocked_success_distance_m:.2f} m) "
                f"and drove an extra {self.final_extra_forward_m:.2f} m."
            ),
        )

    def _simple_approach_to_target(self, target_map):
        """Rotate toward the latest target and drive forward until near it.

        Behavior is intentionally minimal:
        1) face target,
        2) drive until ``min_standoff``,
        3) creep an extra ``final_extra_forward_m`` (~5 cm by default).
        """
        deadline = rospy.Time.now() + rospy.Duration(self.close_in_timeout_s)
        rate = rospy.Rate(10.0)
        last_target = target_map
        last_distance = None

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                self.pub_cmd.publish(Twist())
                if last_distance is None:
                    return False, last_target, "timed out before TF distance became available"
                return (
                    False,
                    last_target,
                    f"timed out at remaining distance {last_distance:.2f} m "
                    f"(timeout={self.close_in_timeout_s:.1f}s)",
                )

            live_target = self._latest_target_in_map() or target_map
            last_target = live_target
            d = self._planar_distance_to(live_target)
            if d is None:
                rate.sleep()
                continue
            last_distance = float(d)
            if d <= self.min_standoff:
                break

            if self.close_in_guard_enabled:
                guard_ok, why = self._front_clearance_ok()
                if not guard_ok:
                    # Safety-first stop: if guard blocks but we're already within
                    # a graspable distance, stop here and report success.
                    if d <= self.guard_blocked_success_distance_m:
                        rospy.logwarn(
                            "[approach_object] guard blocked at d=%.2f; "
                            "treating as success (>= safe grasp standoff).",
                            d,
                        )
                        self.pub_cmd.publish(Twist())
                        break

                    near_target = d <= self.fallback_force_closein_near_target_m
                    if not near_target:
                        rospy.logwarn("[approach_object] blocked while approaching: %s", why)
                        self.pub_cmd.publish(Twist())
                        if last_distance is None:
                            return False, last_target, f"blocked by guard: {why}"
                        return (
                            False,
                            last_target,
                            f"blocked by guard at {last_distance:.2f} m: {why}",
                        )
                    rospy.logwarn_throttle(
                        1.0,
                        "[approach_object] guard blocked near target "
                        "(d=%.2f), forcing slow close-in.",
                        d,
                    )

            err = self._bearing_error_to_target(live_target)
            if err is None:
                rate.sleep()
                continue

            tw = Twist()
            if abs(err) > self.align_tol_rad:
                tw.angular.z = float(
                    self.align_angular_speed if err > 0.0 else -self.align_angular_speed
                )
            else:
                guard_ok_linear = True
                if self.close_in_guard_enabled:
                    guard_ok_linear, _ = self._front_clearance_ok()
                tw.linear.x = float(
                    self.close_in_speed
                    if guard_ok_linear
                    else self.fallback_force_closein_speed
                )
            self.pub_cmd.publish(tw)
            rate.sleep()

        self.pub_cmd.publish(Twist())
        if self.final_extra_forward_m > 0.0:
            if not self._drive_forward_extra(self.final_extra_forward_m):
                return False, last_target, "extra forward segment blocked by guard"
        self.pub_cmd.publish(Twist())
        return True, last_target, ""

    def _drive_forward_extra(self, distance_m):
        speed = max(0.03, min(abs(self.close_in_speed), 0.10))
        duration_s = float(distance_m) / speed
        end_time = rospy.Time.now() + rospy.Duration(duration_s)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.close_in_guard_enabled:
                guard_ok, why = self._front_clearance_ok()
                if not guard_ok:
                    rospy.logwarn("[approach_object] blocked during extra forward move: %s", why)
                    self.pub_cmd.publish(Twist())
                    return False
            tw = Twist()
            tw.linear.x = speed
            self.pub_cmd.publish(tw)
            rate.sleep()
        self.pub_cmd.publish(Twist())
        return True

    # ---------------------------------------------------- step 1: standoff
    def _send_standoff(self, target_map):
        tx = target_map.pose.position.x
        ty = target_map.pose.position.y
        base_yaw = self._robot_yaw_towards(tx, ty)
        cands = standoff_helpers.standoff_candidates(
            tx,
            ty,
            base_yaw,
            distance_m=self.standoff,
            arc_deg=self.standoff_arc_deg,
            max_arc_deg=self.standoff_max_deg,
        )
        if not cands:
            return False, None
        gx, gy, yaw = cands[0]
        goal_pose = self._make_goal_pose(gx, gy, yaw)
        if self.precheck_plan_service and not self._has_plan_to(goal_pose):
            rospy.logwarn(
                "[approach_object] No plan to first standoff goal (%.2f, %.2f); "
                "skipping move_base and using fallback.",
                gx,
                gy,
            )
            return False, None
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        rospy.loginfo(
            "[approach_object] Sending standoff goal (%.2f, %.2f, yaw=%.2f, d=%.2f)",
            gx,
            gy,
            yaw,
            self.standoff,
        )
        self.mb.send_goal(goal)
        finished = self.mb.wait_for_result(rospy.Duration(self.move_base_timeout_s))
        status = self.mb.get_state()
        if finished and status == GoalStatus.SUCCEEDED:
            return True, goal_pose
        rospy.logwarn(
            "[approach_object] single standoff goal failed (status=%s); "
            "switching to fallback.",
            status,
        )
        self.mb.cancel_all_goals()
        return False, None

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
        live_target = self._latest_target_in_map() or target_map
        d0 = self._planar_distance_to(live_target)
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

            live_target = self._latest_target_in_map() or target_map
            d = self._planar_distance_to(live_target)
            if d is None:
                rate.sleep()
                continue
            if d <= self.min_standoff:
                self.pub_cmd.publish(Twist())
                return True

            if self.close_in_guard_enabled:
                guard_ok, why = self._front_clearance_ok()
                if not guard_ok:
                    if d > self.fallback_force_closein_near_target_m:
                        self.pub_cmd.publish(Twist())
                        rospy.logwarn(
                            "[approach_object] close-in fallback blocked by guard: %s", why
                        )
                        return False
                    rospy.logwarn_throttle(
                        1.0,
                        "[approach_object] guard blocked near target (d=%.2f), "
                        "forcing slow close-in.",
                        d,
                    )

            err = self._bearing_error_to_target(live_target)
            if err is None:
                rate.sleep()
                continue

            tw = Twist()
            if abs(err) > self.align_tol_rad:
                tw.angular.z = float(
                    self.align_angular_speed if err > 0 else -self.align_angular_speed
                )
            else:
                if self.close_in_guard_enabled:
                    guard_ok, _ = self._front_clearance_ok()
                    tw.linear.x = float(
                        self.close_in_speed if guard_ok else self.fallback_force_closein_speed
                    )
                else:
                    tw.linear.x = float(self.close_in_speed)
            self.pub_cmd.publish(tw)
            rate.sleep()

        self.pub_cmd.publish(Twist())
        return False

    def _latest_target_in_map(self):
        with self._state_lock:
            tgt = self._latest_pose
        if tgt is None:
            return None
        return self._to_map_frame(tgt)

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

    def _has_plan_to(self, goal_pose):
        try:
            rospy.wait_for_service("/move_base/make_plan", timeout=0.5)
            make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = self.base_frame
            start.pose.orientation.w = 1.0
            resp = make_plan(start=start, goal=goal_pose, tolerance=0.15)
            return bool(resp.plan.poses)
        except Exception as exc:
            rospy.logwarn(
                "[approach_object] make_plan precheck unavailable (%s); proceeding.",
                exc,
            )
            return True

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
