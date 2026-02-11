#!/usr/bin/env python3
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

        # Frames
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_depth_optical_frame"
        )

        # Topics
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/camera/color/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/depth/image_raw"
        )
        self.info_topic = rospy.get_param(
            "~info_topic", "/camera/color/camera_info"
        )

        # Depth limits (meters)
        self.depth_min = rospy.get_param("~depth_min", 0.10)
        self.depth_max = rospy.get_param("~depth_max", 2.50)

        # Detection params
        self.min_area_px = rospy.get_param("~min_area_px", 900)
        self.median_patch_px = rospy.get_param("~median_patch_px", 9)

        # HSV blue range (OpenCV)
        self.h_low = rospy.get_param("~h_low", 90)
        self.h_high = rospy.get_param("~h_high", 130)
        self.s_low = rospy.get_param("~s_low", 50)
        self.v_low = rospy.get_param("~v_low", 40)

        # Stability
        self.require_stable_hits = rospy.get_param("~stable_hits", 3)
        self.hit_count = 0

        # MyCobot
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        self.mc = MyCobot280(port, baud)

        # Safe pose
        self.mc.send_angles([-90, 0, -10, -90, 0, 57], 50)

        # TF
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        self.bridge = CvBridge()

        # Sync subscribers
        s_rgb = message_filters.Subscriber(self.rgb_topic, Image)
        s_depth = message_filters.Subscriber(self.depth_topic, Image)
        s_info = message_filters.Subscriber(self.info_topic, CameraInfo)

        sync = message_filters.ApproximateTimeSynchronizer(
            [s_rgb, s_depth, s_info], queue_size=10, slop=0.1
        )
        sync.registerCallback(self.cb)

        rospy.loginfo("BlueCubeGrasper started (NO PyKDL)")

    def cb(self, rgb_msg, depth_msg, info_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return

        H, W = rgb.shape[:2]
        fx, fy, cx, cy = (
            info_msg.K[0],
            info_msg.K[4],
            info_msg.K[2],
            info_msg.K[5],
        )

        det = self.detect_blue(rgb)
        if det is None:
            self.hit_count = 0
            return

        u, v, area = det
        Z = self.median_depth(depth, u, v, H, W)
        if Z is None:
            self.hit_count = 0
            return

        Xc = (u - cx) * Z / fx
        Yc = (v - cy) * Z / fy

        base_pt = self.to_base(rgb_msg, info_msg, Xc, Yc, Z)
        if base_pt is None:
            self.hit_count = 0
            return

        self.hit_count += 1
        rospy.loginfo_throttle(
            1.0,
            "Target base_link [m]: %.3f %.3f %.3f (%d/%d)",
            *base_pt,
            self.hit_count,
            self.require_stable_hits,
        )

        if self.hit_count >= self.require_stable_hits:
            self.hit_count = 0
            self.grasp(base_pt)

    def detect_blue(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (self.h_low, self.s_low, self.v_low),
            (self.h_high, 255, 255),
        )

        mask = cv2.medianBlur(mask, 7)
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
        return u, v, int(area)

    def median_depth(self, depth, u, v, H, W):
        r = self.median_patch_px // 2
        patch = depth[max(0, v - r):min(H, v + r + 1),
                      max(0, u - r):min(W, u + r + 1)]
        vals = patch[np.isfinite(patch)]
        if len(vals) < 10:
            return None
        Z = float(np.median(vals))
        if not (self.depth_min <= Z <= self.depth_max):
            return None
        return Z

    def to_base(self, rgb_msg, info_msg, X, Y, Z):
        try:
            p = PointStamped()
            p.header = rgb_msg.header
            p.header.frame_id = info_msg.header.frame_id
            p.point.x, p.point.y, p.point.z = X, Y, Z

            T = self.tfbuf.lookup_transform(
                self.base_frame,
                p.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.3),
            )
            out = tf2_geometry_msgs.do_transform_point(p, T).point
            return out.x, out.y, out.z
        except Exception as e:
            rospy.logwarn("TF failed: %s", e)
            return None

    def grasp(self, base_pt):
        x, y, z = base_pt
        X = y * 1000.0
        Y = -x * 1000.0
        Z = z * 1000.0

        rospy.loginfo("Grasp arm [mm]: %.1f %.1f %.1f", X, Y, Z)

        try:
            self.mc.set_gripper_state(0, 80)
            self.mc.send_coords([X, Y + 30, Z + 20, -110, 45, 165], 40)
            self.mc.send_coords([X, Y, Z, -110, 45, 165], 40)
            self.mc.set_gripper_state(1, 80)
            self.mc.send_angles([-77, -50, -40, 100, -5, 52], 50)
        except Exception as e:
            rospy.logerr("Grasp failed: %s", e)


if __name__ == "__main__":
    BlueCubeGrasper()
    rospy.spin()
