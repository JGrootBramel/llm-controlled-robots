#!/usr/bin/env python3
import os
import threading
import time
import math
import numpy as np
import rospy
import message_filters
import tf2_ros
import tf2_geometry_msgs
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from pymycobot import MyCobot280

from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerResponse

class BlueCubeGrasper:
    def __init__(self):
        rospy.init_node("blue_cube_grasper")
        
        # --- Frame & Topic Config ---
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_depth_optical_frame")
        self.target_frame = rospy.get_param("~target_frame", "map") 

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.info_topic = rospy.get_param("~info_topic", "/camera/color/camera_info")

        # --- Detection Thresholds (FIXED FOR BLUE) ---
        self.depth_min = float(rospy.get_param("~depth_min", 0.10))
        self.depth_max = float(rospy.get_param("~depth_max", 2.50))
        self.min_area_px = int(rospy.get_param("~min_area_px", 50)) # Lowered to see further
        self.max_area_px = int(rospy.get_param("~max_area_px", 4000))
        self.median_patch_px = int(rospy.get_param("~median_patch_px", 9))

        # Target color: "red" or "blue" (red often better in blue-tinted rooms)
        self.target_color = rospy.get_param("~target_color", "red").strip().lower()
        if self.target_color not in ("red", "blue"):
            self.target_color = "red"
        # HSV for blue: single band; for red: two bands (H wraps 0/180 in OpenCV)
        if self.target_color == "blue":
            self.h_low = int(rospy.get_param("~blue_h_low", 105))
            self.h_high = int(rospy.get_param("~blue_h_high", 135))
            self.s_low = int(rospy.get_param("~blue_s_low", 70))
            self.v_low = int(rospy.get_param("~blue_v_low", 40))
            self._red_low1 = self._red_high1 = self._red_low2 = self._red_high2 = None
        else:
            # Red: (0–10) and (170–180) in H
            red_h1_low = int(rospy.get_param("~red_h1_low", 0))
            red_h1_high = int(rospy.get_param("~red_h1_high", 10))
            red_h2_low = int(rospy.get_param("~red_h2_low", 170))
            red_h2_high = int(rospy.get_param("~red_h2_high", 180))
            # Slightly stricter defaults to suppress dark red background clutter.
            red_s_low = int(rospy.get_param("~red_s_low", 85))
            red_v_low = int(rospy.get_param("~red_v_low", 65))
            self._red_low1 = np.array([red_h1_low, red_s_low, red_v_low], dtype=np.uint8)
            self._red_high1 = np.array([red_h1_high, 255, 255], dtype=np.uint8)
            self._red_low2 = np.array([red_h2_low, red_s_low, red_v_low], dtype=np.uint8)
            self._red_high2 = np.array([red_h2_high, 255, 255], dtype=np.uint8)
            self.h_low = self.h_high = self.s_low = self.v_low = None

        # --- State Tracking ---
        self.require_stable_hits = int(rospy.get_param("~stable_hits", 3))
        self.hit_count = 0
        self.last_p_map = None # Stores the latest coordinate for the grasp
        self.grasp_armed = False 

        # --- Place pose (left of base / tray): tune via params or defaults below ---
        self.place_angles = rospy.get_param(
            "~place_angles",
            [-90.0, -25.0, -50.0, 75.0, 0.0, 50.0],
        )
        self.place_speed = int(rospy.get_param("~place_speed", 40))
        self.place_gripper_open = int(rospy.get_param("~place_gripper_open", 80))
        self.place_home_angles = rospy.get_param(
            "~place_home_angles",
            [-50.0, 0.0, -10.0, -90.0, 0.0, 57.0],
        )
        self.place_home_speed = int(rospy.get_param("~place_home_speed", 50))

        # MyCobot 280 Cartesian limits (mm); API rejects outside ~ ±350 on each axis.
        self._mc_coord_lim = float(rospy.get_param("~coord_limit_mm", 350.0))
        # Pre-grasp offset adds +30 to Y and +10 to Z — keep those moves inside limits too.
        self._mc_pre_dy = float(rospy.get_param("~pre_grasp_delta_y_mm", 30.0))
        self._mc_pre_dz = float(rospy.get_param("~pre_grasp_delta_z_mm", 10.0))
        # Extra Z (mm) added to arm target before clamp (negative = lower tip toward table).
        self._grasp_z_adjust_mm = float(rospy.get_param("~grasp_z_adjust_mm", -35.0))
        # Fine-tune MyCobot mm targets after base_link→arm rotation (per mount calibration).
        self._arm_bias_x_mm = float(rospy.get_param("~arm_bias_x_mm", 0.0))
        self._arm_bias_y_mm = float(rospy.get_param("~arm_bias_y_mm", 0.0))
        self._arm_bias_z_mm = float(rospy.get_param("~arm_bias_z_mm", 0.0))
        self._grasp_refine_iters = int(rospy.get_param("~grasp_refine_iters", 8))
        self._grasp_refine_sleep_s = float(rospy.get_param("~grasp_refine_sleep_s", 0.4))
        self._grasp_refine_speed = int(rospy.get_param("~grasp_refine_speed", 28))
        self._grasp_pixel_tol_px = float(rospy.get_param("~grasp_pixel_tol_px", 14.0))
        self._grasp_rxryrz = (
            int(rospy.get_param("~grasp_coord_rx", -110)),
            int(rospy.get_param("~grasp_coord_ry", 45)),
            int(rospy.get_param("~grasp_coord_rz", 165)),
        )
        # False (default): same-frame grasp from stable-hit pose (reliable). True: extra vision iterations (see ~grasp_refine_*).
        self._use_async_grasp_refine = bool(
            rospy.get_param("~use_async_grasp_refine", False)
        )

        self._grasp_busy = False
        self._last_frame_lock = threading.Lock()
        self._last_rgb = None
        self._last_depth = None
        # Immutable snap of intrinsics + frame_id — never pass live CameraInfo refs into another thread (rospy reuses msgs).
        self._last_snap = None

        # --- MyCobot Hardware Init ---
        self.mc_port = rospy.get_param("~port", "/dev/ttyACM0")
        self.mc_baud = int(rospy.get_param("~baud", 115200))
        self.mc = None
        try:
            # Ensure you ran: sudo chmod 777 /dev/ttyACM0
            self.mc = MyCobot280(self.mc_port, self.mc_baud)
            # Home pose: Looking forward and slightly down at the floor/box
            self.mc.send_angles([-50.0, 0.0, -10.0, -90.0, 0.0, 57.0], 50)
        except Exception as e:
            rospy.logerr(f"MyCobot Connection Failed: {e}")
            rospy.logerr(
                "Grasp/place will not run until the arm is reachable on %s",
                self.mc_port,
            )

        # --- TF & Bridge ---
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)
        self.bridge = CvBridge()

        # --- ROS Interfaces for ROSA Tools ---
        self.cube_pub = rospy.Publisher("~cube_map_pose", PointStamped, queue_size=10)
        # Keep private topic for backwards compatibility, and publish a global topic
        # so RViz configs can subscribe consistently across detector nodes.
        self.marker_pub = rospy.Publisher("~detected_cube_markers", Marker, queue_size=10)
        self.marker_pub_global = rospy.Publisher("/detected_objects_markers", Marker, queue_size=10)
        
        # This is the "Doorbell" the LLM rings after it finishes driving
        self.grasp_srv = rospy.Service("~execute_grasp", Trigger, self.trigger_grasp_cb)

        # --- Synchronized Subscribers ---
        s_rgb = message_filters.Subscriber(self.rgb_topic, Image)
        s_depth = message_filters.Subscriber(self.depth_topic, Image)
        s_info = message_filters.Subscriber(self.info_topic, CameraInfo)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [s_rgb, s_depth, s_info], queue_size=10, slop=0.15
        )
        self.sync.registerCallback(self.cb)
        rospy.loginfo(
            "blue_cube_grasper ready target_color=%s area=[%d, %d] depth=[%.2f, %.2f] topic=%s",
            self.target_color,
            self.min_area_px,
            self.max_area_px,
            self.depth_min,
            self.depth_max,
            self.depth_topic,
        )

    def trigger_grasp_cb(self, req):
        """Arm the grasp; camera callback will perform it using live detection for better accuracy."""
        rospy.loginfo("execute_grasp: arming grasp; will use camera to refine pose when cube is in view.")
        self.grasp_armed = True
        self.hit_count = 0
        return TriggerResponse(success=True, message="Grasp armed; robot will grasp when cube is detected in camera.")

    def _decode_images(self, rgb_msg: Image, depth_msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
            return rgb, depth
        except Exception as e:
            rospy.logwarn_throttle(5.0, "blue_cube_grasper: cv_bridge error: %s", e)
            return None, None
    def _camera_snapshot(self, info_msg: CameraInfo):
        """Copy everything needed from CameraInfo; safe to use from any thread with matching rgb/depth copies."""
        fid = info_msg.header.frame_id if info_msg.header.frame_id else self.camera_frame
        K = tuple(float(info_msg.K[i]) for i in range(9))
        return {"frame_id": str(fid), "K": K}
    def _extract_detection_dict(self, rgb, depth, snap: dict):
        """
        Single-frame blob + depth → (u, v, Z, p_base, p_map) or None.
        `snap` must match this rgb/depth (same _camera_snapshot / frame).
        """
        if snap is None or rgb is None or depth is None:
            return None
        H, W = rgb.shape[:2]
        K = snap["K"]
        fx, fy, cx_i, cy_i = K[0], K[4], K[2], K[5]

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        if self.target_color == "blue":
            lower = np.array([self.h_low, self.s_low, self.v_low], dtype=np.uint8)
            upper = np.array([self.h_high, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
        else:
            mask1 = cv2.inRange(hsv, self._red_low1, self._red_high1)
            mask2 = cv2.inRange(hsv, self._red_low2, self._red_high2)
            mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            rospy.logdebug_throttle(1.0, "blue_cube_grasper: no color contour for target=%s", self.target_color)
            return None
        best_cnt = max(cnts, key=cv2.contourArea)
        best_area = float(cv2.contourArea(best_cnt))
        if best_area < self.min_area_px:
            rospy.loginfo_throttle(
                1.0,
                "blue_cube_grasper: contour too small area=%.1f (< min_area_px=%d)",
                best_area,
                self.min_area_px,
            )
            return None
        if best_area > self.max_area_px:
            rospy.loginfo_throttle(
                1.0,
                "blue_cube_grasper: contour too large area=%.1f (> max_area_px=%d)",
                best_area,
                self.max_area_px,
            )
            return None
        M = cv2.moments(best_cnt)
        u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

        Hd, Wd = depth.shape[:2]
        ud, vd = self._color_to_depth_pixels(u, v, W, H, Wd, Hd)
        Z = self._median_depth_patch(depth, ud, vd, Hd, Wd)
        if Z is None:
            rospy.loginfo_throttle(
                1.0,
                "blue_cube_grasper: invalid depth at uv=(%d,%d)->(%d,%d), depth bounds=[%.2f, %.2f]",
                u,
                v,
                ud,
                vd,
                self.depth_min,
                self.depth_max,
            )
            return None
        Xc, Yc = (u - cx_i) * Z / fx, (v - cy_i) * Z / fy
        frame_id = snap["frame_id"]
        p_base = self._to_frame_optical(frame_id, Xc, Yc, Z, self.base_frame)
        p_map = self._to_frame_optical(frame_id, Xc, Yc, Z, self.target_frame)
        if p_base is None:
            rospy.loginfo_throttle(
                1.0,
                "blue_cube_grasper: TF camera->%s failed (camera frame=%s)",
                self.base_frame,
                frame_id,
            )
            return None
        return (u, v, Z, p_base, p_map)

    @staticmethod
    def _color_to_depth_pixels(u_col, v_col, rgb_w, rgb_h, depth_w, depth_h):
        """
        Map a color-image pixel to depth-image indices when resolutions differ
        (e.g. 640×480 color + 640×400 depth). Using color (u,v) directly on depth was
        sampling the wrong rows → nearly constant Z and identical grasp motions.
        """
        if rgb_w == depth_w and rgb_h == depth_h:
            return u_col, v_col
        ud = int(round(u_col * depth_w / float(rgb_w)))
        vd = int(round(v_col * depth_h / float(rgb_h)))
        ud = max(0, min(depth_w - 1, ud))
        vd = max(0, min(depth_h - 1, vd))
        return ud, vd

    def _base_to_arm_mm(self, p_base):
        x_top, y_top, z_top = p_base
        X_arm = y_top * 1000.0 + self._arm_bias_x_mm
        Y_arm = -x_top * 1000.0 + self._arm_bias_y_mm
        Z_arm = z_top * 1000.0 + self._grasp_z_adjust_mm + self._arm_bias_z_mm
        return X_arm, Y_arm, Z_arm

    def _simple_grasp_job(self, arm_mm):
        """Run pick+place using pose from the stable-hit frame (no extra vision loop)."""
        try:
            X, Y, Z = arm_mm
            rospy.loginfo(
                "grasp direct: stable-hit pose (%.1f, %.1f, %.1f) mm (z_adjust=%.1f)",
                X,
                Y,
                Z,
                self._grasp_z_adjust_mm,
            )
            if self.do_grasp(X, Y, Z):
                self.do_place()
        finally:
            self._grasp_busy = False

    def _grasp_refine_and_execute(self, seed_arm, seed_snap, seed_rgb, seed_depth):
        """
        Optional: iterative pre-grasp moves. Always falls back to seed_arm if vision fails
        (fixes bogus 'abort' when worker couldn't match blobs due to msg reuse — now snapshots safe).
        Iter 0 uses seed_* with no initial sleep so arm hasn't moved yet.
        """
        try:
            if self.mc is None:
                rospy.logerr("grasp refine: MyCobot not connected")
                return
            rx, ry, rz = self._grasp_rxryrz
            try:
                self.mc.set_gripper_state(0, 80)
            except Exception as e:
                rospy.logwarn("grasp refine: gripper open failed: %s", e)

            last_arm = seed_arm

            for i in range(self._grasp_refine_iters):
                if i > 0:
                    rospy.sleep(self._grasp_refine_sleep_s)

                if i == 0 and seed_rgb is not None and seed_snap is not None:
                    rgb = seed_rgb
                    depth = seed_depth
                    snap = seed_snap
                else:
                    with self._last_frame_lock:
                        rgb = None if self._last_rgb is None else self._last_rgb.copy()
                        depth = None if self._last_depth is None else self._last_depth.copy()
                        snap = self._last_snap

                if rgb is None or depth is None or snap is None:
                    rospy.logwarn_throttle(2.0, "grasp refine: frame/snap missing (iter %d)", i)
                    continue

                det = self._extract_detection_dict(rgb, depth, snap)
                if det is None:
                    rospy.logwarn_throttle(2.0, "grasp refine: no blob (iter %d)", i)
                    continue

                u, v, _Z, p_base, _pm = det
                X, Y, Zmm = self._base_to_arm_mm(p_base)
                last_arm = (X, Y, Zmm)

                K = snap["K"]
                px_u, px_v = float(K[2]), float(K[5])
                err_px = math.hypot(float(u) - px_u, float(v) - px_v)

                cxm, cym, czm, clipped = self._clamp_grasp_coords_mm(X, Y, Zmm)
                if clipped:
                    rospy.logwarn_throttle(
                        3.0,
                        "grasp refine: workspace clamp; drive base closer if needed",
                    )

                if err_px <= self._grasp_pixel_tol_px and i >= 2:
                    rospy.loginfo(
                        "grasp refine: centered (%.1f px ≤ %.1f); stop refine",
                        err_px,
                        self._grasp_pixel_tol_px,
                    )
                    break

                is_last = i >= self._grasp_refine_iters - 1
                if not is_last:
                    try:
                        self.mc.send_coords(
                            [
                                cxm,
                                cym + self._mc_pre_dy,
                                czm + self._mc_pre_dz,
                                rx,
                                ry,
                                rz,
                            ],
                            self._grasp_refine_speed,
                        )
                    except Exception as e:
                        rospy.logerr("grasp refine: send_coords failed: %s", e)
                        break

            rospy.sleep(max(0.12, self._grasp_refine_sleep_s * 0.35))
            with self._last_frame_lock:
                rgb = None if self._last_rgb is None else self._last_rgb.copy()
                depth = None if self._last_depth is None else self._last_depth.copy()
                snap = self._last_snap
            if rgb is not None and depth is not None and snap is not None:
                det = self._extract_detection_dict(rgb, depth, snap)
                if det is not None:
                    last_arm = self._base_to_arm_mm(det[3])

            Xf, Yf, Zf = last_arm
            rospy.loginfo(
                "grasp refine: final pick at (%.1f, %.1f, %.1f) mm (fallback used if refine lost blob)",
                Xf,
                Yf,
                Zf,
            )
            if self.do_grasp(Xf, Yf, Zf):
                self.do_place()
        finally:
            self._grasp_busy = False

    def cb(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        rgb, depth = self._decode_images(rgb_msg, depth_msg)
        if rgb is None:
            return

        snap = self._camera_snapshot(info_msg)
        with self._last_frame_lock:
            self._last_rgb = rgb.copy()
            self._last_depth = depth.copy()
            self._last_snap = snap

        det = self._extract_detection_dict(rgb, depth, snap)
        if det is None:
            rospy.logdebug("blue_cube_grasper: no detection this frame")
            self.hit_count = 0
            return

        u, v, Z, p_base, p_map = det

        if p_map is not None:
            # Update 'last known' for the grasp service
            self.last_p_map = p_map 

            # Publish map-frame cube pose for higher-level tools (ROSA / scan_for_blue_cubes)
            cube_msg = PointStamped()
            cube_msg.header.stamp = rgb_msg.header.stamp
            cube_msg.header.frame_id = self.target_frame
            cube_msg.point.x = float(p_map[0])
            cube_msg.point.y = float(p_map[1])
            cube_msg.point.z = float(p_map[2])
            self.cube_pub.publish(cube_msg)
            rospy.loginfo_throttle(
                1.0,
                "blue_cube_grasper: published cube_map_pose (x=%.2f, y=%.2f, z=%.2f)",
                cube_msg.point.x,
                cube_msg.point.y,
                cube_msg.point.z,
            )

            # Simple RViz marker so detections are visible on the map
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.target_frame
            marker.ns = "blue_cubes"
            # Use a coarse grid ID for stable markers
            gx = int(cube_msg.point.x * 10.0)
            gy = int(cube_msg.point.y * 10.0)
            marker.id = gx + gy * 1000
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = cube_msg.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            if self.target_color == "red":
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            self.marker_pub.publish(marker)
            self.marker_pub_global.publish(marker)

        # When grasp is armed, use live camera pose + optional visual refinement (thread)
        self.hit_count += 1
        if (
            not self._grasp_busy
            and self.hit_count >= self.require_stable_hits
            and self.grasp_armed
            and p_base is not None
        ):
            Hrgb, Wrgb = rgb.shape[0], rgb.shape[1]
            Hd, Wd = depth.shape[0], depth.shape[1]
            ud, vd = self._color_to_depth_pixels(u, v, Wrgb, Hrgb, Wd, Hd)
            K = snap["K"]
            fx, fy, cx_i, cy_i = K[0], K[4], K[2], K[5]
            Xc = (u - cx_i) * Z / fx
            Yc = (v - cy_i) * Z / fy
            arm_mm = self._base_to_arm_mm(p_base)
            rospy.loginfo(
                "STABLE HIT trace: rgb %dx%d depth %dx%d | color_uv=(%d,%d)→depth_uv=(%d,%d) | "
                "Z=%.3fm optical XYZ=(%.3f,%.3f,%.3f) | base_link=(%.3f,%.3f,%.3f)m | "
                "arm_mm=(%.1f,%.1f,%.1f)",
                Wrgb,
                Hrgb,
                Wd,
                Hd,
                u,
                v,
                ud,
                vd,
                Z,
                Xc,
                Yc,
                Z,
                p_base[0],
                p_base[1],
                p_base[2],
                arm_mm[0],
                arm_mm[1],
                arm_mm[2],
            )
            if Wrgb != Wd or Hrgb != Hd:
                rospy.logwarn_throttle(
                    15.0,
                    "Color and depth differ in size — depth is sampled at remapped (ud,vd); "
                    "prefer /camera/... depth registered to color (same WxH) if available.",
                )
            seed_rgb = rgb.copy()
            seed_depth = depth.copy()
            self.grasp_armed = False
            self.hit_count = 0
            self._grasp_busy = True
            if self._use_async_grasp_refine:
                rospy.loginfo(
                    "STABLE HIT: async refine + pick (same-frame seed + snapshot K/frame_id)."
                )
                threading.Thread(
                    target=self._grasp_refine_and_execute,
                    args=(arm_mm, snap, seed_rgb, seed_depth),
                    daemon=True,
                ).start()
            else:
                rospy.loginfo("STABLE HIT: direct pick from this frame (recommended).")
                threading.Thread(
                    target=self._simple_grasp_job,
                    args=(arm_mm,),
                    daemon=True,
                ).start()

    def _median_depth_patch(self, depth, u, v, depth_h, depth_w):
        """Median depth in a window around (u,v) in *depth image* coordinates (HxW = depth shape)."""
        win = self.median_patch_px
        if win % 2 == 0: win += 1
        r = win // 2
        x0, x1 = max(0, u - r), min(depth_w, u + r + 1)
        y0, y1 = max(0, v - r), min(depth_h, v + r + 1)
        patch = depth[y0:y1, x0:x1].copy()
        valid = np.isfinite(patch) & (patch > self.depth_min) & (patch < self.depth_max)
        vals = patch[valid]
        if vals.size < 10: return None
        return float(np.median(vals))

    def _to_frame_optical(self, optical_frame_id: str, Xc, Yc, Zc, target_frame):
        """TF: point in camera optical frame → target_frame (string frame_id, no CameraInfo ref)."""
        try:
            src_frame = optical_frame_id or self.camera_frame
            p_cam = PointStamped()
            p_cam.header.frame_id = src_frame
            p_cam.point.x, p_cam.point.y, p_cam.point.z = float(Xc), float(Yc), float(Zc)

            T_cb = self.tfbuf.lookup_transform(
                target_frame, src_frame, rospy.Time(0), rospy.Duration(0.3)
            )
            p_base = tf2_geometry_msgs.do_transform_point(p_cam, T_cb).point
            return (float(p_base.x), float(p_base.y), float(p_base.z))
        except Exception:
            return None

    def _clamp_grasp_coords_mm(self, x_mm: float, y_mm: float, z_mm: float):
        """
        Clamp (X,Y,Z) so final grasp and pre-grasp (Y+dy, Z+dz) stay within mc_coord_lim.
        Returns (x', y', z', clipped) where clipped is True if any axis was changed.
        """
        L = self._mc_coord_lim
        dy, dz = self._mc_pre_dy, self._mc_pre_dz
        # Final pose must be in [-L, L]; pre pose needs y+dy and z+dz in [-L, L].
        y_hi = min(L, L - dy)
        y_lo = max(-L, -L - dy)
        z_hi = min(L, L - dz)
        z_lo = max(-L, -L - dz)
        ox, oy, oz = x_mm, y_mm, z_mm
        cx = max(-L, min(L, x_mm))
        cy = max(y_lo, min(y_hi, y_mm))
        cz = max(z_lo, min(z_hi, z_mm))
        clipped = (cx != ox) or (cy != oy) or (cz != oz)
        return cx, cy, cz, clipped

    def do_grasp(self, X_arm_mm, Y_arm_mm, Z_arm_mm):
        if self.mc is None:
            rospy.logerr_throttle(
                10.0,
                "do_grasp skipped: MyCobot not connected (see startup MyCobot Connection Failed)",
            )
            return False
        cx, cy, cz, clipped = self._clamp_grasp_coords_mm(X_arm_mm, Y_arm_mm, Z_arm_mm)
        if clipped:
            rospy.logwarn(
                "Grasp coords clamped for MyCobot limits (±%.0f mm, incl. pre-offset): "
                "requested (%.1f, %.1f, %.1f) → (%.1f, %.1f, %.1f). Drive closer if grasp misses.",
                self._mc_coord_lim,
                X_arm_mm,
                Y_arm_mm,
                Z_arm_mm,
                cx,
                cy,
                cz,
            )
        try:
            self.mc.set_gripper_state(0, 80)
            rx, ry, rz = self._grasp_rxryrz
            speed = 40
            self.mc.send_coords([cx, cy + self._mc_pre_dy, cz + self._mc_pre_dz, rx, ry, rz], speed)
            time.sleep(2)
            self.mc.send_coords([cx, cy, cz, rx, ry, rz], speed)
            time.sleep(2)
            self.mc.set_gripper_state(1, 80)
            time.sleep(1.5)
            self.mc.send_angles([-77.0, -50.0, -40.0, 100.0, -5.0, 52.0], 50)
            time.sleep(2)
            return True
        except Exception as e:
            rospy.logerr("Grasp failed (MyCobot / coords): %s", e)
            return False

    def do_place(self):
        """Move arm to the configured left-side tray pose and drop the cube."""
        if self.mc is None:
            rospy.logerr_throttle(
                10.0,
                "do_place skipped: MyCobot not connected",
            )
            return
        try:
            self.mc.send_angles(self.place_angles, self.place_speed)
            time.sleep(2.5)
            self.mc.set_gripper_state(0, self.place_gripper_open)
            time.sleep(1.0)
            self.mc.send_angles(self.place_home_angles, self.place_home_speed)
            time.sleep(2.0)
        except Exception as e:
            rospy.logerr("Place failed: %s", e)
            
    def _transform_point(self, pt_tuple, from_frame, to_frame, stamp):
        """
        Helper to transform a (x, y, z) tuple between coordinate frames.
        """
        try:
            p = PointStamped()
            p.header.frame_id = from_frame
            # Use Time(0) to get the latest available transform
            p.header.stamp = rospy.Time(0) 
            p.point.x, p.point.y, p.point.z = pt_tuple
            
            # Look up the transform
            T = self.tfbuf.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.3))
            
            # Perform the transformation
            pt_out = tf2_geometry_msgs.do_transform_point(p, T).point
            return (pt_out.x, pt_out.y, pt_out.z)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"TF Transform failed: {e}")
            return None

if __name__ == "__main__":
    BlueCubeGrasper()
    rospy.spin()