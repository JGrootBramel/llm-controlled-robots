#!/usr/bin/env python3
"""MyCobot 280 arm control node.

Pure manipulation. It does *not* know about colour detection, cameras,
odometry or move_base — everything it needs to reach for arrives through
two ``geometry_msgs/PoseStamped`` topics:

* ``~target_pose``           — continuous stream from the perception stack
                               (e.g. ``/red_cubes/latest_pose``).
* ``~target_pose_override``  — one-shot, externally supplied pose. When
                               present, it wins for the next pick and is
                               cleared afterwards. Used by the ``pick_at_pose``
                               ROSA tool so callers can hand the arm explicit
                               coordinates without racing the detector.

Both topics may be in any TF frame (transformed into ``base_link`` at
pick time).

Services (all ``std_srvs/Trigger``):

* ``~pick``              — close gripper on the latest target (override > stream)
* ``~pick_vendor_sync`` — same target selection as ``~pick`` but moves use
                          pymycobot ``sync_send_coords`` / ``sync_send_angles``
                          (vendor blocking APIs) for A/B testing
* ``~place``             — drop the cube on the left-side tray
* ``~go_home``           — return to the idle home pose
"""

import inspect
import math
import os
import sys
import time
from threading import Lock

import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _helpers import mycobot_helpers as mch  # noqa: E402

try:
    from pymycobot import MyCobot280  # type: ignore
except Exception:  # pragma: no cover - no hardware in CI
    MyCobot280 = None  # type: ignore


class ArmControl:
    def __init__(self):
        self._load_params()
        self._lock = Lock()
        self._latest_target = None
        self._override_target = None
        self._override_place_target = None
        self._last_target_error = None

        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        rospy.Subscriber(
            "~target_pose", PoseStamped, self._on_target, queue_size=1
        )
        rospy.Subscriber(
            "~target_pose_override",
            PoseStamped,
            self._on_target_override,
            queue_size=1,
        )
        rospy.Subscriber(
            "~place_pose_override",
            PoseStamped,
            self._on_place_override,
            queue_size=1,
        )

        self._mc = self._connect_arm()
        self.ready_angles = self._compute_ready_angles()

        rospy.Service("~pick", Trigger, self._handle_pick)
        rospy.Service(
            "~pick_vendor_sync", Trigger, self._handle_pick_vendor_sync
        )
        rospy.Service("~place", Trigger, self._handle_place)
        rospy.Service("~place_at_override", Trigger, self._handle_place_at_override)
        rospy.Service("~go_home", Trigger, self._handle_home)

    # ------------------------------------------------------------ params
    def _load_params(self):
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.speed = int(rospy.get_param("~speed", 40))
        self.place_speed = int(rospy.get_param("~place_speed", 40))
        self.place_gripper_open = int(rospy.get_param("~place_gripper_open", 80))
        # Physical mount yaw (CCW degrees from base_link +X to arm +X). Set to
        # 90 for the default LIMO Cobot mount (arm faces robot-left). Changing
        # this rotates the Cartesian mapping AND the home/place defaults in
        # one place, so the gripper keeps pointing forward at home regardless.
        self.mount_yaw_deg = float(
            rospy.get_param("~mount_yaw_deg", mch.DEFAULT_MOUNT_YAW_DEG)
        )
        self.place_angles = rospy.get_param(
            "~place_angles", list(mch.default_place_angles(self.mount_yaw_deg))
        )
        self.home_angles = rospy.get_param(
            "~home_angles", list(mch.default_home_angles(self.mount_yaw_deg))
        )
        self.coord_lim_mm = float(rospy.get_param("~coord_limit_mm", 280.0))
        self.pre_dy_mm = float(rospy.get_param("~pre_grasp_delta_y_mm", 30.0))
        self.pre_dz_mm = float(rospy.get_param("~pre_grasp_delta_z_mm", 5.0))
        self.grasp_z_adjust_mm = float(
            rospy.get_param("~grasp_z_adjust_mm", 140.0)
        )
        self.min_valid_target_z_m = float(
            rospy.get_param("~min_valid_target_z_m", 0.03)
        )
        self.fallback_target_z_m = float(
            rospy.get_param("~fallback_target_z_m", 0.08)
        )
        self.grasp_z_adjust_only_below_m = float(
            rospy.get_param("~grasp_z_adjust_only_below_m", 0.08)
        )
        self.apply_z_adjust_to_override = bool(
            rospy.get_param("~apply_z_adjust_to_override", False)
        )
        self.simple_pick_enabled = bool(
            rospy.get_param("~simple_pick_enabled", True)
        )
        self.simple_pick_skip_ready = bool(
            rospy.get_param("~simple_pick_skip_ready", True)
        )
        self.ik_verify_tol_mm = float(
            rospy.get_param("~ik_verify_tol_mm", 15.0)
        )
        self.grasp_rxryrz = (
            int(rospy.get_param("~grasp_coord_rx", mch.GRASP_RXRYRZ[0])),
            int(rospy.get_param("~grasp_coord_ry", mch.GRASP_RXRYRZ[1])),
            int(rospy.get_param("~grasp_coord_rz", mch.GRASP_RXRYRZ[2])),
        )
        # Staging pose tuning. The pick goes home -> ready -> pre -> grasp.
        # ``ready_angles`` is computed from IK at connect time (see
        # ``_compute_ready_angles``) unless the user pins it via param.
        # ``approach_mode`` is the send_coords mode for the pre-grasp move:
        # 0 = point-to-point joint space (smooth, wrist can't flip),
        # 1 = linear Cartesian (straight line, only safe once wrist is
        # pre-oriented by the ready pose).
        self.approach_mode = int(rospy.get_param("~approach_mode", 0))
        self.ready_point_base_m = rospy.get_param(
            "~ready_point_base_xyz_m", list(mch.DEFAULT_READY_POINT_BASE_M)
        )
        self._ready_angles_override = rospy.get_param("~ready_angles", None)
        self.vendor_sync_timeout_s = float(
            rospy.get_param("~vendor_sync_timeout_s", 20.0)
        )

    # ---------------------------------------------------------- arm wiring
    def _connect_arm(self):
        if MyCobot280 is None:
            rospy.logerr(
                "[arm_control] pymycobot not available; arm commands will error."
            )
            return None
        try:
            mc = MyCobot280(self.port, self.baud)
            mc.send_angles(list(self.home_angles), 50)
            rospy.loginfo("[arm_control] MyCobot connected on %s", self.port)
            return mc
        except Exception as e:
            rospy.logerr("[arm_control] MyCobot connect failed: %s", e)
            return None

    def _compute_ready_angles(self):
        """Decide the pre-grasp staging pose.

        If ``~ready_angles`` is set, trust the operator verbatim. Otherwise
        ask the arm firmware's on-board IK for a solution at the configured
        base-frame staging point with the grasp orientation, so the wrist
        is pre-oriented the same as the eventual grasp pose. Everything
        is wrapped in try/except: if the arm is disconnected or IK refuses
        we drop to a hand-picked fallback pose instead of crashing.
        """
        fallback = list(
            mch.default_ready_angles_fallback(
                mount_yaw_deg=self.mount_yaw_deg,
                grasp_rz_deg=self.grasp_rxryrz[2],
            )
        )
        if self._ready_angles_override:
            return [float(a) for a in self._ready_angles_override]
        if self._mc is None:
            rospy.logwarn(
                "[arm_control] arm not connected; using fallback ready pose %s",
                fallback,
            )
            return fallback
        try:
            x_m, y_m, z_m = (float(v) for v in self.ready_point_base_m)
            x_arm, y_arm, z_arm = mch.base_to_arm_mm(
                x_m, y_m, z_m, mount_yaw_deg=self.mount_yaw_deg
            )
            target = [
                x_arm, y_arm, z_arm,
                float(self.grasp_rxryrz[0]),
                float(self.grasp_rxryrz[1]),
                float(self.grasp_rxryrz[2]),
            ]
            solved = self._mc.solve_inv_kinematics(
                target, list(self.home_angles)
            )
            # pymycobot versions differ: some return 6 floats, others an int
            # error code or another type — never call len() blindly.
            if isinstance(solved, (list, tuple)) and len(solved) == 6:
                try:
                    sol_f = [float(a) for a in solved]
                except (TypeError, ValueError):
                    sol_f = None
                if sol_f and any(abs(a) > 1e-6 for a in sol_f):
                    rospy.loginfo(
                        "[arm_control] ready pose via IK: %s (target arm_mm=%s)",
                        ["%.1f" % a for a in sol_f],
                        ["%.0f" % v for v in target[:3]],
                    )
                    return sol_f
            rospy.logwarn(
                "[arm_control] solve_inv_kinematics returned %r; using fallback %s",
                solved,
                fallback,
            )
        except Exception as e:
            rospy.logwarn(
                "[arm_control] ready-pose IK failed (%s); using fallback %s",
                e, fallback,
            )
        return fallback

    # ---------------------------------------------------------- callbacks
    def _on_target(self, msg):
        with self._lock:
            self._latest_target = msg

    def _on_target_override(self, msg):
        with self._lock:
            self._override_target = msg
        rospy.loginfo(
            "[arm_control] override target latched: frame=%s xyz=(%.3f, %.3f, %.3f)",
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _on_place_override(self, msg):
        with self._lock:
            self._override_place_target = msg
        rospy.loginfo(
            "[arm_control] override place target latched: frame=%s xyz=(%.3f, %.3f, %.3f)",
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _resolve_pick_arm_mm(self):
        """Return ``(cx, cy, cz, source)`` in arm millimetres or ``None`` if no target."""
        self._last_target_error = None
        target, source = self._select_target_in_base()
        if target is None:
            self._last_target_error = (
                source if isinstance(source, str) and source else "no_target"
            )
            return None
        x_m, y_m, z_m = target
        if z_m < 0.05:
            rospy.logwarn_throttle(
                60.0,
                "[arm_control] target z in base_link=%.3f m is very low (cube on the "
                "floor in TF?). Pre-grasp Z≈%.0f mm in arm frame is often unreachable; "
                "fix depth/TF height or add ~grasp_z_adjust_mm (mm toward the table).",
                z_m,
                z_m * 1000.0 + float(self.grasp_z_adjust_mm),
            )
        if source != "override" and z_m < self.min_valid_target_z_m:
            rospy.logwarn(
                "[arm_control] replacing invalid target z=%.3f m with fallback z=%.3f m",
                z_m,
                self.fallback_target_z_m,
            )
            z_m = float(self.fallback_target_z_m)
        X_arm, Y_arm, Z_arm = mch.base_to_arm_mm(
            x_m, y_m, z_m, mount_yaw_deg=self.mount_yaw_deg
        )
        # Use measured z when available. Apply old fixed z offset only when
        # detector z is clearly unreliable (near floor) or if explicitly requested.
        apply_adjust = (source != "override" and z_m < self.grasp_z_adjust_only_below_m) or (
            source == "override" and self.apply_z_adjust_to_override
        )
        if apply_adjust:
            Z_arm = Z_arm + float(self.grasp_z_adjust_mm)
        if source == "override":
            if (
                abs(X_arm) > self.coord_lim_mm
                or abs(Y_arm) > self.coord_lim_mm
                or abs(Z_arm) > self.coord_lim_mm
            ):
                self._last_target_error = (
                    "override target out of arm reach: "
                    f"arm_mm=({X_arm:.1f}, {Y_arm:.1f}, {Z_arm:.1f}) "
                    f"limit=+/-{self.coord_lim_mm:.1f}"
                )
                rospy.logerr(
                    "[arm_control] override target out of arm cube: "
                    "(%.1f, %.1f, %.1f) mm",
                    X_arm,
                    Y_arm,
                    Z_arm,
                )
                return None
            return (X_arm, Y_arm, Z_arm, source)

        limits = mch.GraspLimits(
            coord_lim_mm=self.coord_lim_mm,
            pre_dy_mm=self.pre_dy_mm,
            pre_dz_mm=self.pre_dz_mm,
        )
        cx, cy, cz, clipped = mch.clamp_grasp_coords_mm(X_arm, Y_arm, Z_arm, limits)
        if clipped:
            rospy.logwarn(
                "[arm_control] grasp coords clamped: (%.1f,%.1f,%.1f) -> (%.1f,%.1f,%.1f)",
                X_arm, Y_arm, Z_arm, cx, cy, cz,
            )
        return (cx, cy, cz, source)

    def _handle_pick(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        resolved = self._resolve_pick_arm_mm()
        if resolved is None:
            # Coordinate-driven flow can race: the override publisher may fire
            # just before this service runs. Wait briefly for one override pose.
            try:
                override_topic = rospy.resolve_name("~target_pose_override")
                msg = rospy.wait_for_message(
                    override_topic, PoseStamped, timeout=0.7
                )
                with self._lock:
                    self._override_target = msg
                resolved = self._resolve_pick_arm_mm()
            except Exception:
                resolved = None
        if resolved is None:
            reason = self._last_target_error or "override_required"
            return TriggerResponse(
                success=False,
                message=(
                    "Pick target unavailable: "
                    f"{reason}. Publish ~target_pose_override with a reachable pose."
                ),
            )
        cx, cy, cz, source = resolved
        ok = self._execute_pick(cx, cy, cz, use_vendor_sync=False)
        return TriggerResponse(
            success=bool(ok),
            message=(
                f"pick ok at arm_mm=({cx:.0f},{cy:.0f},{cz:.0f}) [src={source}]"
                if ok
                else f"pick failed [src={source}]; see node logs"
            ),
        )

    def _handle_pick_vendor_sync(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        if not hasattr(self._mc, "sync_send_coords") or not callable(
            getattr(self._mc, "sync_send_coords")
        ):
            return TriggerResponse(
                success=False,
                message=(
                    "pymycobot sync_send_coords not available on this "
                    "MyCobot280 binding; upgrade pymycobot for vendor sync pick."
                ),
            )
        resolved = self._resolve_pick_arm_mm()
        if resolved is None:
            try:
                override_topic = rospy.resolve_name("~target_pose_override")
                msg = rospy.wait_for_message(
                    override_topic, PoseStamped, timeout=0.7
                )
                with self._lock:
                    self._override_target = msg
                resolved = self._resolve_pick_arm_mm()
            except Exception:
                resolved = None
        if resolved is None:
            reason = self._last_target_error or "override_required"
            return TriggerResponse(
                success=False,
                message=(
                    "Pick target unavailable: "
                    f"{reason}. Publish ~target_pose_override with a reachable pose."
                ),
            )
        cx, cy, cz, source = resolved
        ok = self._execute_pick(cx, cy, cz, use_vendor_sync=True)
        return TriggerResponse(
            success=bool(ok),
            message=(
                f"pick_vendor_sync ok at arm_mm=({cx:.0f},{cy:.0f},{cz:.0f}) "
                f"[src={source}]"
                if ok
                else f"pick_vendor_sync failed [src={source}]; see node logs"
            ),
        )

    def _handle_place(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        try:
            self._mc.send_angles(list(self.place_angles), self.place_speed)
            time.sleep(2.5)
            self._mc.set_gripper_state(0, self.place_gripper_open)
            time.sleep(1.0)
            self._mc.send_angles(list(self.home_angles), self.place_speed)
            time.sleep(2.0)
            return TriggerResponse(success=True, message="placed and returned home")
        except Exception as e:
            return TriggerResponse(success=False, message=f"place failed: {e}")

    def _handle_place_at_override(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        with self._lock:
            pose = self._override_place_target
            self._override_place_target = None
        if pose is None:
            return TriggerResponse(
                success=False,
                message="No place override target available; publish ~place_pose_override first.",
            )
        xyz = self._pose_in_base(pose)
        if xyz is None:
            return TriggerResponse(
                success=False,
                message=f"Could not transform place override into {self.base_frame}.",
            )
        x_m, y_m, z_m = xyz
        X_arm, Y_arm, Z_arm = mch.base_to_arm_mm(
            x_m, y_m, z_m, mount_yaw_deg=self.mount_yaw_deg
        )
        limits = mch.GraspLimits(
            coord_lim_mm=self.coord_lim_mm,
            pre_dy_mm=self.pre_dy_mm,
            pre_dz_mm=self.pre_dz_mm,
        )
        px, py, pz, _ = mch.clamp_grasp_coords_mm(X_arm, Y_arm, Z_arm, limits)
        ok = self._execute_place_at(px, py, pz)
        return TriggerResponse(
            success=bool(ok),
            message=(
                f"place_at_override ok at arm_mm=({px:.0f},{py:.0f},{pz:.0f})"
                if ok
                else "place_at_override failed; see node logs"
            ),
        )

    def _handle_home(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        try:
            self._mc.send_angles(list(self.home_angles), 50)
            time.sleep(1.5)
            return TriggerResponse(success=True, message="arm at home pose")
        except Exception as e:
            return TriggerResponse(success=False, message=f"go_home failed: {e}")

    # --------------------------------------------------------- primitives
    def _coords_reached(self, want_xyz_mm):
        """Return (ok, err_mm, got_coords) after a send_coords move.

        ``None`` if we couldn't read coords back from the arm. Lets us catch
        silent firmware IK rejections (pymycobot's ``send_coords`` is
        fire-and-forget over serial, so unreachable targets return without
        raising but the arm never moves).
        """
        try:
            got = self._mc.get_coords()
        except Exception as e:
            rospy.logwarn("[arm_control] get_coords failed: %s", e)
            return None
        if not got or len(got) < 3:
            return None
        dx = got[0] - want_xyz_mm[0]
        dy = got[1] - want_xyz_mm[1]
        dz = got[2] - want_xyz_mm[2]
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        return (err <= self.ik_verify_tol_mm, err, got)

    def _sync_send_coords(self, coords, spd, mode):
        """Call pymycobot ``sync_send_coords`` with ``timeout`` if supported."""
        fn = self._mc.sync_send_coords
        kwargs = {}
        try:
            sig = inspect.signature(fn)
            if "timeout" in sig.parameters:
                kwargs["timeout"] = self.vendor_sync_timeout_s
        except (TypeError, ValueError):
            pass
        try:
            fn(coords, spd, int(mode), **kwargs)
        except TypeError:
            fn(coords, spd, int(mode))

    def _sync_send_angles(self, angles, spd, async_sleep_s=2.0):
        """Vendor blocking joint move, or ``send_angles`` + sleep if unavailable."""
        if hasattr(self._mc, "sync_send_angles") and callable(
            self._mc.sync_send_angles
        ):
            fn = self._mc.sync_send_angles
            kwargs = {}
            try:
                sig = inspect.signature(fn)
                if "timeout" in sig.parameters:
                    kwargs["timeout"] = self.vendor_sync_timeout_s
            except (TypeError, ValueError):
                pass
            try:
                fn(list(angles), spd, **kwargs)
            except TypeError:
                fn(list(angles), spd)
        else:
            self._mc.send_angles(list(angles), spd)
            time.sleep(async_sleep_s)

    def _log_in_position_debug(self, data, flag_coord):
        """Best-effort vendor ``is_in_position`` log (flag 1 = coords, 0 = angles)."""
        fn = getattr(self._mc, "is_in_position", None)
        if not callable(fn):
            return
        try:
            ip = fn(data, flag_coord)
            rospy.loginfo("[arm_control] is_in_position -> %s", ip)
        except Exception as e:
            rospy.logwarn("[arm_control] is_in_position failed: %s", e)

    def _execute_pick(self, x_mm, y_mm, z_mm, use_vendor_sync=False):
        if use_vendor_sync:
            return self._execute_pick_vendor_sync_impl(x_mm, y_mm, z_mm)
        if self.simple_pick_enabled:
            return self._execute_pick_simple(x_mm, y_mm, z_mm)
        rx, ry, rz = self.grasp_rxryrz
        speed = self.speed
        try:
            self._mc.set_gripper_state(0, 80)
            # Stage 1: move to the ready pose in joint space. The big J1/J2
            # rotation happens here, gripper safely high, before we are near
            # the target. Wrist ends up pre-oriented like the grasp pose so
            # the subsequent Cartesian move is essentially a translation.
            self._mc.send_angles(
                list(self.ready_angles), max(20, speed // 2)
            )
            time.sleep(2.0)
            # Stage 2: pre-grasp waypoint above/behind the target.
            pre = [
                x_mm,
                y_mm + self.pre_dy_mm,
                z_mm + self.pre_dz_mm,
                rx, ry, rz,
            ]
            self._mc.send_coords(pre, speed, int(self.approach_mode))
            time.sleep(2.0)
            check = self._coords_reached(pre[:3])
            if check is None:
                rospy.logwarn(
                    "[arm_control] pre-grasp IK verify skipped (no coords read back)"
                )
            elif not check[0]:
                rospy.logerr(
                    "[arm_control] pre-grasp NOT reached (firmware likely rejected IK): "
                    "want=%s got=%s err_mm=%.1f",
                    pre[:3], list(check[2][:3]), check[1],
                )
                # Retreat via ready so the recovery motion stays clear of
                # the target instead of sweeping through it.
                self._mc.send_angles(list(self.ready_angles), 50)
                time.sleep(1.5)
                self._mc.send_angles(list(self.home_angles), 50)
                time.sleep(1.5)
                return False

            # Stage 3: final descent uses linear Cartesian for a straight
            # line down onto the cube regardless of ~approach_mode.
            grasp = [x_mm, y_mm, z_mm, rx, ry, rz]
            self._mc.send_coords(grasp, speed, 1)
            time.sleep(2.0)
            check = self._coords_reached(grasp[:3])
            if check is None:
                rospy.logwarn(
                    "[arm_control] grasp IK verify skipped (no coords read back)"
                )
            elif not check[0]:
                rospy.logerr(
                    "[arm_control] grasp NOT reached (firmware likely rejected IK): "
                    "want=%s got=%s err_mm=%.1f",
                    grasp[:3], list(check[2][:3]), check[1],
                )
                self._mc.send_angles(list(self.ready_angles), 50)
                time.sleep(1.5)
                self._mc.send_angles(list(self.home_angles), 50)
                time.sleep(1.5)
                return False

            if not self._close_gripper_strong():
                rospy.logerr("[arm_control] gripper failed to close at grasp pose")
                self._mc.send_angles(list(self.ready_angles), 50)
                time.sleep(1.5)
                self._mc.send_angles(list(self.home_angles), 50)
                time.sleep(1.5)
                return False
            # Lift via the ready pose first so the cube clears the work area
            # before J1 swings the arm back to home.
            self._mc.send_angles(list(self.ready_angles), 50)
            time.sleep(1.5)
            self._mc.send_angles(list(self.home_angles), 50)
            time.sleep(1.5)
            return True
        except Exception as e:
            rospy.logerr("[arm_control] pick error: %s", e)
            return False

    def _execute_pick_simple(self, x_mm, y_mm, z_mm):
        """Presentation-safe pick: go target -> close -> return home."""
        rx, ry, rz = self.grasp_rxryrz
        speed = self.speed
        try:
            self._mc.set_gripper_state(0, 100)
            time.sleep(0.6)
            if not self.simple_pick_skip_ready:
                self._mc.send_angles(list(self.ready_angles), max(20, speed // 2))
                time.sleep(2.0)
            grasp = [x_mm, y_mm, z_mm, rx, ry, rz]
            # Linear Cartesian move to the exact target point.
            self._mc.send_coords(grasp, speed, 1)
            time.sleep(2.2)
            check = self._coords_reached(grasp[:3])
            if check is not None and not check[0]:
                rospy.logerr(
                    "[arm_control] simple grasp NOT reached: want=%s got=%s err_mm=%.1f",
                    grasp[:3], list(check[2][:3]), check[1],
                )
                self._mc.send_angles(list(self.home_angles), 50)
                time.sleep(1.5)
                return False
            if not self._close_gripper_strong():
                rospy.logerr("[arm_control] gripper failed to close at simple grasp pose")
                self._mc.send_angles(list(self.home_angles), 50)
                time.sleep(1.5)
                return False
            self._mc.send_angles(list(self.ready_angles), 50)
            time.sleep(1.5)
            self._mc.send_angles(list(self.home_angles), 50)
            time.sleep(1.5)
            return True
        except Exception as e:
            rospy.logerr("[arm_control] simple pick error: %s", e)
            return False

    def _execute_place_at(self, x_mm, y_mm, z_mm):
        rx, ry, rz = self.grasp_rxryrz
        speed = self.speed
        try:
            self._mc.send_angles(list(self.ready_angles), max(20, speed // 2))
            time.sleep(2.0)
            pre = [x_mm, y_mm + self.pre_dy_mm, z_mm + self.pre_dz_mm, rx, ry, rz]
            self._mc.send_coords(pre, speed, int(self.approach_mode))
            time.sleep(2.0)
            place = [x_mm, y_mm, z_mm, rx, ry, rz]
            self._mc.send_coords(place, speed, 1)
            time.sleep(2.0)
            self._mc.set_gripper_state(0, self.place_gripper_open)
            time.sleep(1.2)
            self._mc.send_angles(list(self.ready_angles), 50)
            time.sleep(1.5)
            self._mc.send_angles(list(self.home_angles), 50)
            time.sleep(1.5)
            return True
        except Exception as e:
            rospy.logerr("[arm_control] place_at error: %s", e)
            return False

    def _close_gripper_strong(self):
        """Best-effort close+verify to reliably hold cubes before retreat."""
        try:
            for _ in range(3):
                self._mc.set_gripper_state(1, 100)
                time.sleep(0.35)
            get_val = getattr(self._mc, "get_gripper_value", None)
            if callable(get_val):
                v = get_val()
                if isinstance(v, (int, float)) and v <= 5:
                    return False
            return True
        except Exception as e:
            rospy.logerr("[arm_control] gripper close error: %s", e)
            return False

    def _execute_pick_vendor_sync_impl(self, x_mm, y_mm, z_mm):
        """Pick using Elephant ``sync_send_*`` APIs; requires ``sync_send_coords``."""
        rx, ry, rz = self.grasp_rxryrz
        speed = self.speed
        spd_angles = max(20, speed // 2)
        try:
            self._mc.set_gripper_state(0, 80)
            self._sync_send_angles(list(self.ready_angles), spd_angles)
            self._log_in_position_debug(list(self.ready_angles), 0)

            pre = [
                x_mm,
                y_mm + self.pre_dy_mm,
                z_mm + self.pre_dz_mm,
                rx, ry, rz,
            ]
            self._sync_send_coords(pre, speed, int(self.approach_mode))
            self._log_in_position_debug(pre, 1)

            check = self._coords_reached(pre[:3])
            if check is None:
                rospy.logwarn(
                    "[arm_control] vendor pre-grasp verify skipped (no coords read back)"
                )
            elif not check[0]:
                rospy.logerr(
                    "[arm_control] vendor pre-grasp NOT reached: want=%s got=%s err_mm=%.1f",
                    pre[:3], list(check[2][:3]), check[1],
                )
                self._sync_send_angles(
                    list(self.ready_angles), 50, async_sleep_s=1.5
                )
                self._sync_send_angles(
                    list(self.home_angles), 50, async_sleep_s=1.5
                )
                return False

            grasp = [x_mm, y_mm, z_mm, rx, ry, rz]
            self._sync_send_coords(grasp, speed, 1)
            self._log_in_position_debug(grasp, 1)

            check = self._coords_reached(grasp[:3])
            if check is None:
                rospy.logwarn(
                    "[arm_control] vendor grasp verify skipped (no coords read back)"
                )
            elif not check[0]:
                rospy.logerr(
                    "[arm_control] vendor grasp NOT reached: want=%s got=%s err_mm=%.1f",
                    grasp[:3], list(check[2][:3]), check[1],
                )
                self._sync_send_angles(
                    list(self.ready_angles), 50, async_sleep_s=1.5
                )
                self._sync_send_angles(
                    list(self.home_angles), 50, async_sleep_s=1.5
                )
                return False

            self._mc.set_gripper_state(1, 80)
            time.sleep(1.5)
            self._sync_send_angles(
                list(self.ready_angles), 50, async_sleep_s=1.5
            )
            self._sync_send_angles(
                list(self.home_angles), 50, async_sleep_s=1.5
            )
            return True
        except Exception as e:
            rospy.logerr("[arm_control] pick_vendor_sync error: %s", e)
            return False

    def _select_target_in_base(self):
        """Pick target in ``base_link`` from explicit override only.

        Strict mode: we only accept caller-provided coordinates via
        ``~target_pose_override``. This avoids accidental grabs from stale
        detector stream poses.
        """
        with self._lock:
            override = self._override_target
            self._override_target = None

        if override is not None:
            xyz = self._pose_in_base(override)
            if xyz is not None:
                return xyz, "override"
            rospy.logwarn(
                "[arm_control] override pose could not be transformed to %s; "
                "NOT falling back to stream for this pick.",
                self.base_frame,
            )
            return None, "override_tf_failed"
        return None, "override_required"

    def _pose_in_base(self, pose):
        if pose.header.frame_id == self.base_frame:
            return (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        try:
            T = self.tfbuf.lookup_transform(
                self.base_frame, pose.header.frame_id,
                rospy.Time(0), rospy.Duration(0.3),
            )
            transformed = tf2_geometry_msgs.do_transform_pose(pose, T)
            return (
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z,
            )
        except Exception as e:
            rospy.logwarn("[arm_control] TF target->base failed: %s", e)
            return None


if __name__ == "__main__":
    rospy.init_node("arm_control")
    ArmControl()
    rospy.loginfo("[arm_control] ready")
    rospy.spin()
