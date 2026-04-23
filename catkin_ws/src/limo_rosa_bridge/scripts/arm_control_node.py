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

* ``~pick``      — close gripper on the latest target (override > stream)
* ``~place``     — drop the cube on the left-side tray
* ``~go_home``   — return to the idle home pose
"""

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

        self._mc = self._connect_arm()

        rospy.Service("~pick", Trigger, self._handle_pick)
        rospy.Service("~place", Trigger, self._handle_place)
        rospy.Service("~go_home", Trigger, self._handle_home)

    # ------------------------------------------------------------ params
    def _load_params(self):
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.speed = int(rospy.get_param("~speed", 40))
        self.place_speed = int(rospy.get_param("~place_speed", 40))
        self.place_gripper_open = int(rospy.get_param("~place_gripper_open", 80))
        self.place_angles = rospy.get_param(
            "~place_angles", list(mch.PLACE_ANGLES)
        )
        self.home_angles = rospy.get_param(
            "~home_angles", list(mch.HOME_ANGLES)
        )
        self.coord_lim_mm = float(rospy.get_param("~coord_limit_mm", 280.0))
        self.pre_dy_mm = float(rospy.get_param("~pre_grasp_delta_y_mm", 30.0))
        self.pre_dz_mm = float(rospy.get_param("~pre_grasp_delta_z_mm", 5.0))
        self.grasp_z_adjust_mm = float(
            rospy.get_param("~grasp_z_adjust_mm", -35.0)
        )
        self.grasp_rxryrz = (
            int(rospy.get_param("~grasp_coord_rx", mch.GRASP_RXRYRZ[0])),
            int(rospy.get_param("~grasp_coord_ry", mch.GRASP_RXRYRZ[1])),
            int(rospy.get_param("~grasp_coord_rz", mch.GRASP_RXRYRZ[2])),
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

    def _handle_pick(self, _req):
        if self._mc is None:
            return TriggerResponse(success=False, message="arm not connected")
        target, source = self._select_target_in_base()
        if target is None:
            return TriggerResponse(
                success=False,
                message=(
                    "No target pose available; publish ~target_pose or "
                    "~target_pose_override first."
                ),
            )
        x_m, y_m, z_m = target
        X_arm, Y_arm, Z_arm = mch.base_to_arm_mm(x_m, y_m, z_m)
        # Apply static z-adjust and clamp inside the safe cube.
        Z_arm = Z_arm + float(self.grasp_z_adjust_mm)
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
        ok = self._execute_pick(cx, cy, cz)
        return TriggerResponse(
            success=bool(ok),
            message=(
                f"pick ok at arm_mm=({cx:.0f},{cy:.0f},{cz:.0f}) [src={source}]"
                if ok
                else f"pick failed [src={source}]; see node logs"
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
    def _execute_pick(self, x_mm, y_mm, z_mm):
        rx, ry, rz = self.grasp_rxryrz
        speed = self.speed
        try:
            self._mc.set_gripper_state(0, 80)
            # Pre-grasp above/behind the target.
            self._mc.send_coords(
                [x_mm, y_mm + self.pre_dy_mm, z_mm + self.pre_dz_mm, rx, ry, rz],
                speed,
            )
            time.sleep(2.0)
            # Descend to grasp.
            self._mc.send_coords([x_mm, y_mm, z_mm, rx, ry, rz], speed)
            time.sleep(2.0)
            self._mc.set_gripper_state(1, 80)
            time.sleep(1.5)
            # Lift back to home so base can move safely.
            self._mc.send_angles(list(self.home_angles), 50)
            time.sleep(1.5)
            return True
        except Exception as e:
            rospy.logerr("[arm_control] pick error: %s", e)
            return False

    def _select_target_in_base(self):
        """Pick the pose to grasp at and return it in ``base_link``.

        Override pose wins when set and is consumed (cleared) so the next
        pick falls back to the perception stream. Returns
        ``(xyz_tuple_or_None, source_label)``.
        """
        with self._lock:
            override = self._override_target
            self._override_target = None
            stream = self._latest_target

        if override is not None:
            xyz = self._pose_in_base(override)
            if xyz is not None:
                return xyz, "override"
            rospy.logwarn(
                "[arm_control] override pose could not be transformed to %s; "
                "falling back to stream",
                self.base_frame,
            )

        if stream is None:
            return None, "none"
        xyz = self._pose_in_base(stream)
        return (xyz, "stream") if xyz is not None else (None, "stream")

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
