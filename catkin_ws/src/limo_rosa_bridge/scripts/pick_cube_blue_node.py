#!/usr/bin/env python3
"""
Blue cube finder + front grasp (RGB-D).

- Detects a blue cube via HSV thresholding in the RGB image.
- Uses depth to estimate the 3D position.
- Transforms the point into base_link via TF.
- Maps base_link -> arm coordinates (same mapping as in object_finder.py).
- Executes a simple front approach grasp using pymycobot.

Notes:
- HSV thresholds are a heuristic; tune them for your lighting/camera.
- Requires TF from camera optical frame to base_link.
"""

import os
import time
import numpy as np
import rospy
import message_filters
import tf2_ros
import tf2_geometry_msgs
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from pymycobot import MyCobot280


class BlueCubeGrasper:
    def __init__(self):
        rospy.init_node("blue_cube_grasper")

        # --- Params (defaults mirror the style of object_finder.py) ---
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_depth_optical_frame")

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.info_topic = rospy.get_param("~info_topic", "/camera/color/camera_info")

        self.depth_min = float(rospy.get_param("~depth_min", 0.10))
        self.depth_max = float(rospy.get_param("~depth_max", 2.50))

        self.min_area_px = int(rospy.get_param("~min_area_px", 900))  # reject small blobs
        self.median_patch_px = int(rospy.get_param("~median_patch_px", 9))  # odd preferred
        self.show_debug_window = rospy.get_param("~show_debug_window", False)
        self.debug_window_name = rospy.get_param("~debug_window_name", "blue_cube_debug")

        # HSV thresholds for "blue"
        # Typical blue range (OpenCV H: 0..179). Tune if needed.
        self.h_low = int(rospy.get_param("~h_low", 40))
        self.h_high = int(rospy.get_param("~h_high", 179))
        self.s_low = int(rospy.get_param("~s_low", 50))
        self.v_low = int(rospy.get_param("~v_low", 40))

        # Grasp behavior
        self.require_stable_hits = int(rospy.get_param("~stable_hits", 3))
        self.hit_count = 0
        self.last_target_base = None

        # MyCobot
        self.mc_port = rospy.get_param("~port", "/dev/ttyACM0")
        self.mc_baud = int(rospy.get_param("~baud", 115200))

        # --- Init robot ---
        self.mc = MyCobot280(self.mc_port, self.mc_baud)
        # safe-ish pose (taken from your object_finder init)
        self.mc.send_angles([-90.0, 0.0, -10.0, -90.0, 0.0, 57.0], 50)

        # --- TF ---
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        self.bridge = CvBridge()

        # --- Sync subscribers ---
        s_rgb = message_filters.Subscriber(self.rgb_topic, Image)
        s_depth = message_filters.Subscriber(self.depth_topic, Image)
        s_info = message_filters.Subscriber(self.info_topic, CameraInfo)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [s_rgb, s_depth, s_info],
            queue_size=10,
            slop=0.12,
        )
        self.sync.registerCallback(self.cb)
        if self.show_debug_window:
            rospy.on_shutdown(self._close_debug_window)

        rospy.loginfo("blue_cube_grasper ready")

    def cb(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # Convert images
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cv_bridge error: %s", e)
            return

        H, W = rgb.shape[:2]
        fx, fy, cx, cy = info_msg.K[0], info_msg.K[4], info_msg.K[2], info_msg.K[5]

        # Detect blue blob (largest)
        det = self._detect_blue_blob(rgb)
        dbg = rgb.copy() if self.show_debug_window else None
        if det is None:
            self.hit_count = 0
            self.last_target_base = None
            self._show_debug(dbg, None, None)
            return

        (u, v, area) = det

        # Refine depth around centroid
        Z = self._median_depth_patch(depth, u, v, H, W)
        if Z is None or not (self.depth_min <= Z <= self.depth_max):
            self.hit_count = 0
            self.last_target_base = None
            self._show_debug(dbg, (u, v), None)
            return

        # Project to camera coordinates
        Xc = (u - cx) * Z / fx
        Yc = (v - cy) * Z / fy

        # Transform camera -> base_link
        p_base = self._to_base_link(info_msg, rgb_msg, Xc, Yc, Z)
        if p_base is None:
            self.hit_count = 0
            self.last_target_base = None
            self._show_debug(dbg, (u, v), None)
            return

        # Simple stability gate
        self.hit_count += 1
        self.last_target_base = p_base

        rospy.loginfo_throttle(
            1.0,
            "blue blob area=%d | uv=(%d,%d) Z=%.3f | base=(%.3f,%.3f,%.3f) hits=%d/%d",
            int(area), int(u), int(v), float(Z),
            p_base[0], p_base[1], p_base[2],
            self.hit_count, self.require_stable_hits,
        )
        self._show_debug(dbg, (u, v), int(area))

        if self.hit_count >= self.require_stable_hits:
            # Reset counter to avoid repeated grasps
            self.hit_count = 0

            # Convert base_link meters -> arm mm 
            x_top, y_top, z_top = p_base  # in meters, base_link

            x_mm = x_top * 1000.0
            y_mm = y_top * 1000.0
            z_mm = z_top * 1000.0

            # base_link -> arm frame mapping (the arm is 90Â° rotated compared to the simulation)
            X_arm = y_mm
            Y_arm = -x_mm
            Z_arm = z_mm

            rospy.loginfo(
                "GRASP base_link[m]=(%.3f,%.3f,%.3f) -> arm[mm]=(%.1f,%.1f,%.1f)",
                x_top, y_top, z_top, X_arm, Y_arm, Z_arm
            )

            ok = self.do_grasp(X_arm, Y_arm, Z_arm)
            rospy.loginfo("Grasp result: %s", "SUCCESS" if ok else "FAIL")

            # Return to a safe pose after grasp attempt
            try:
                self.mc.send_angles([-90.0, 0.0, -10.0, -90.0, 0.0, 57.0], 50)
            except Exception:
                pass

    def _detect_blue_blob(self, bgr_img):
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        lower = np.array([self.h_low, self.s_low, self.v_low], dtype=np.uint8)
        upper = np.array([self.h_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # Clean up
        mask = cv2.medianBlur(mask, 7)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None

        # Find largest blob above area threshold
        best = None
        best_area = 0
        for c in cnts:
            area = cv2.contourArea(c)
            if area > best_area:
                best_area = area
                best = c

        if best is None or best_area < self.min_area_px:
            return None

        M = cv2.moments(best)
        if abs(M["m00"]) < 1e-6:
            return None

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        return (u, v, int(best_area))

    def _median_depth_patch(self, depth, u, v, H, W):
        win = self.median_patch_px
        if win % 2 == 0:
            win += 1
        r = win // 2

        x0, x1 = max(0, u - r), min(W, u + r + 1)
        y0, y1 = max(0, v - r), min(H, v + r + 1)
        patch = depth[y0:y1, x0:x1].copy()

        valid = np.isfinite(patch) & (patch > self.depth_min) & (patch < self.depth_max)
        vals = patch[valid]
        if vals.size < 10:
            return None
        return float(np.median(vals))

    def _to_base_link(self, info_msg, rgb_msg, Xc, Yc, Zc):
        try:
            src_frame = info_msg.header.frame_id or self.camera_frame

            p_cam = PointStamped()
            p_cam.header = rgb_msg.header
            p_cam.header.frame_id = src_frame
            p_cam.point.x = float(Xc)
            p_cam.point.y = float(Yc)
            p_cam.point.z = float(Zc)

            T_cb = self._lookup_transform_tolerant(self.base_frame, src_frame, rgb_msg.header.stamp)
            p_cam.header.stamp = T_cb.header.stamp
            p_base = tf2_geometry_msgs.do_transform_point(p_cam, T_cb).point
            return (float(p_base.x), float(p_base.y), float(p_base.z))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF camera->base failed: %s", e)
            return None

    def _lookup_transform_tolerant(self, target_frame, source_frame, stamp, timeout=0.3):
        try:
            return self.tfbuf.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(timeout))
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException):
            return self.tfbuf.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))

    def _show_debug(self, img, uv, area):
        """
        Show live detection debug image on robot display.
        """
        if not self.show_debug_window or img is None:
            return
        try:
            if uv is not None:
                cv2.drawMarker(img, uv, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
            if area is not None:
                cv2.putText(
                    img,
                    f"blue area={int(area)} hits={self.hit_count}/{self.require_stable_hits}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow(self.debug_window_name, img)
            cv2.waitKey(1)
        except Exception:
            pass

    def _close_debug_window(self):
        if not self.show_debug_window:
            return
        try:
            cv2.destroyWindow(self.debug_window_name)
            cv2.waitKey(1)
        except Exception:
            pass

    # ---- Grasp copied/adapted from your object_finder.py ----
    def do_grasp(self, X_arm_mm, Y_arm_mm, Z_arm_mm):
        """
        Front approach grasp sequence using MyCobot.
        Coordinate mapping is assumed identical to object_finder.py.
        """
        try:

            # open gripper
            self.mc.set_gripper_state(0, 80)

            # fixed tool orientation to grasp from front
            rx, ry, rz = -110, 45, 165
            speed = 40

            # offsets fintune if needed
            X_arm_mm = X_arm_mm
            Y_arm_mm = Y_arm_mm + 30
            Z_arm_mm = Z_arm_mm + 10

            # approach from the front (slightly higher and further)
            self.mc.send_coords([X_arm_mm, Y_arm_mm + 30, Z_arm_mm + 10, rx, ry, rz], speed)

            # move into grasp
            self.mc.send_coords([X_arm_mm, Y_arm_mm, Z_arm_mm, rx, ry, rz], speed)

            # close gripper
            self.mc.set_gripper_state(1, 80)

            # lift / retreat
            self.mc.send_angles([-77.0, -50.0, -40.0, 100.0, -5.0, 52.0], 50)

            return True
        except Exception as e:
            rospy.logerr("Grasp failed: %s", e)
            return False


if __name__ == "__main__":
    BlueCubeGrasper()
    rospy.spin()