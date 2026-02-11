#!/usr/bin/env python3
"""
Blue cube finder + front grasp (RGB-D)
PyKDL-FREE / tf2_geometry_msgs-FREE
"""

import numpy as np
import rospy
import cv2
import message_filters
import tf2_ros

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix
from pymycobot import MyCobot280


# ---------- PURE NUMPY TF ----------
def transform_point(point_xyz, transform):
    t = transform.transform.translation
    q = transform.transform.rotation

    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z

    p = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0])
    p_out = T @ p
    return p_out[:3]


class BlueCubeGrasper:
    def __init__(self):
        rospy.init_node("blue_cube_grasper")

        # ---- Parameters ----
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_depth_optical_frame"
        )

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.info_topic = rospy.get_param(
            "~info_topic", "/camera/color/camera_info"
        )

        self.depth_min = rospy.get_param("~depth_min", 0.10)
        self.depth_max = rospy.get_param("~depth_max", 2.50)
        self.min_area_px = rospy.get_param("~min_area_px", 900)
        self.stable_hits = rospy.get_param("~stable_hits", 3)

        # HSV blue thresholds (tune if needed)
        self.h_low = rospy.get_param("~h_low", 90)
        self.h_high = rospy.get_param("~h_high", 140)
        self.s_low = rospy.get_param("~s_low", 50)
        self.v_low = rospy.get_param("~v_low", 50)

        # ---- Robot ----
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        self.mc = MyCobot280(port, baud)

        # Safe pose
        self.mc.send_angles([-90, 0, -10, -90, 0, 57], 50)

        # ---- TF ----
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # ---- Vision ----
        self.bridge = CvBridge()
        self.hit_count = 0
        self.last_target = None

        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.info_topic, CameraInfo)

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.1,
        )
        sync.registerCallback(self.callback)

        rospy.loginfo("BlueCubeGrasper ready (PyKDL-free)")

    # ---------- CALLBACK ----------
    def callback(self, rgb_msg, depth_msg, info_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
        except Exception:
            return

        H, W = rgb.shape[:2]
        fx, fy = info_msg.K[0], info_msg.K[4]
        cx, cy = info_msg.K[2], info_msg.K[5]

        det = self.detect_blue(rgb)
        if det is None:
            self.hit_count = 0
            return

        u, v, _ = det
        Z = self.median_depth(depth, u, v)
        if Z is None:
            self.hit_count = 0
            return

        # Camera coords
        Xc = (u - cx) * Z / fx
        Yc = (v - cy) * Z / fy

        try:
            T = self.tfbuf.lookup_transform(
                self.base_frame,
                info_msg.header.frame_id or self.camera_frame,
                rospy.Time(0),
                rospy.Duration(0.3),
            )
            p_base = transform_point((Xc, Yc, Z), T)
        except Exception:
            self.hit_count = 0
            return

        self.hit_count += 1
        self.last_target = p_base

        rospy.loginfo_throttle(
            1.0,
            "Blue cube base_link [m]: x=%.3f y=%.3f z=%.3f (%d/%d)",
            p_base[0], p_base[1], p_base[2],
            self.hit_count, self.stable_hits,
        )

        if self.hit_count >= self.stable_hits:
            self.hit_count = 0
            self.grasp(p_base)

    # ---------- BLUE DETECTION ----------
    def detect_blue(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_low, self.s_low, self.v_low])
        upper = np.array([self.h_high, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not cnts:
            return None

        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.min_area_px:
            return None

        M = cv2.moments(c)
        if M["m00"] == 0:
            return None

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        return u, v, area

    # ---------- DEPTH ----------
    def median_depth(self, depth, u, v, r=4):
        h, w = depth.shape
        x0, x1 = max(0, u - r), min(w, u + r)
        y0, y1 = max(0, v - r), min(h, v + r)
        patch = depth[y0:y1, x0:x1]
        valid = patch[(patch > self.depth_min) & (patch < self.depth_max)]
        if valid.size < 10:
            return None
        return float(np.median(valid))

    # ---------- GRASP ----------
    def grasp(self, p_base):
        x_m, y_m, z_m = p_base
        x_mm = x_m * 1000
        y_mm = y_m * 1000
        z_mm = z_m * 1000

        # base_link → arm frame mapping
        X = y_mm
        Y = -x_mm + 30
        Z = z_mm + 10

        rospy.loginfo(
            "Grasping at arm [mm]: X=%.1f Y=%.1f Z=%.1f", X, Y, Z
        )

        try:
            self.mc.set_gripper_state(0, 80)
            self.mc.send_coords([X, Y + 30, Z, -110, 45, 165], 40)
            self.mc.send_coords([X, Y, Z, -110, 45, 165], 40)
            self.mc.set_gripper_state(1, 80)
            self.mc.send_angles([-77, -50, -40, 100, -5, 52], 50)
        except Exception as e:
            rospy.logerr("Grasp failed: %s", e)


if __name__ == "__main__":
    BlueCubeGrasper()
    rospy.spin()

