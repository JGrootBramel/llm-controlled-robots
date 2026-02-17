#!/usr/bin/env python3

import os
import time
import math
import rospy
import numpy as np
import torch
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Point, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from tf.transformations import quaternion_matrix
from nanoowl.owl_predictor import OwlPredictor
from PIL import Image as PILImage
import cv2


def transform_point_manual(p, t, q):
    """Transform a point p using translation t and quaternion q."""
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    p_h = np.array([p[0], p[1], p[2], 1.0])
    p_out = T @ p_h
    return p_out[:3]


class ObjectFinder:
    S_SEARCHING = 0
    S_APPROACHING = 1
    S_ARRIVED = 2
    S_ALIGNED = 3
    S_CLOSING_IN = 4
    S_GRASPED = 5

    def __init__(self):
        rospy.init_node("object_finder")

        self._load_params()
        self._init_state()
        self._init_detector()
        self._init_ros_comm()
        self._init_traj_logging()

        self.pub_ready.publish(Bool(data=True))
        rospy.loginfo("ObjectFinder ready | prompt=%s thr=%.2f", self.prompt, self.threshold)

    # ---------------- Initialization ----------------
    def _load_params(self):
        self.prompt       = rospy.get_param("~prompt", "a water bottle")
        self.threshold    = rospy.get_param("~threshold", 0.15)
        self.engine_path  = rospy.get_param(
            "~image_encoder_engine",
            os.path.expanduser("~/nanoowl/data/owl_image_encoder_patch32.engine")
        )

        self.min_hits     = rospy.get_param("~min_hits", 3)
        self.max_age      = rospy.get_param("~max_age_sec", 2.0)

        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame   = rospy.get_param("~base_frame",   "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_depth_optical_frame")
        self.rgb_topic    = rospy.get_param("~rgb_topic",   "/camera/color/image_raw")
        self.depth_topic  = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.info_topic   = rospy.get_param("~info_topic",  "/camera/color/camera_info")

        self.depth_min        = rospy.get_param("~depth_min", 0.10)
        self.depth_max        = rospy.get_param("~depth_max", 4.50)
        self.near_percentile  = rospy.get_param("~near_percentile", 15.0)
        self.band_width_m     = rospy.get_param("~band_width_m", 0.02)
        self.bbox_shrink_px   = rospy.get_param("~bbox_shrink_px", 6)
        self.median_patch_px  = rospy.get_param("~median_patch_px", 7)

        self.min_standoff   = rospy.get_param("~min_standoff", 0.31)
        self.close_in_speed = rospy.get_param("~close_in_speed", 0.10)

        self.top_percentile   = rospy.get_param("~top_percentile", 90.0)
        self.grasp_z_lift     = rospy.get_param("~grasp_lift_m", 0.02)
        self.publish_debug    = rospy.get_param("~publish_debug", True)

        self.traj_max_duration_sec = rospy.get_param("~traj_max_duration_sec", 6 * 60.0)
        self.traj_log_dir = rospy.get_param(
            "~traj_log_dir", os.path.expanduser("~/traj_logs")
        )

    def _init_state(self):
        self.state = self.S_SEARCHING
        self.query = self.prompt
        self.hits_in_row = 0
        self.last_seen_ts = 0.0
        self.sent_goal_for_this_track = False
        self.approached = False
        self.last_detection = None
        self.start_pose = None

        self.traj_logging_active = True
        self.traj_start_time = rospy.Time.now()
        self.traj_last_xy = None
        self.traj_total_dist = 0.0
        self.traj_log = []

    def _init_detector(self):
        self.predictor = OwlPredictor(
            "google/owlvit-base-patch32",
            image_encoder_engine=self.engine_path,
            device="cpu",
        )
        self.text_encoding = self.predictor.encode_text(self.query)

    def _init_ros_comm(self):
        self.bridge = CvBridge()
        import tf
        self.tf_listener = tf.TransformListener()

        self.pub_found  = rospy.Publisher("/object_found", Bool, queue_size=1, latch=True)
        self.pub_ready  = rospy.Publisher("/object_detection_ready", Bool, queue_size=1, latch=True)
        self.pub_pose   = rospy.Publisher("/object_pose",  PoseStamped, queue_size=1, latch=True)
        self.pub_goal   = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_dbg    = (rospy.Publisher("/object_debug", Image, queue_size=1) if self.publish_debug else None)

        s_rgb   = message_filters.Subscriber(self.rgb_topic, Image)
        s_depth = message_filters.Subscriber(self.depth_topic, Image)
        s_info  = message_filters.Subscriber(self.info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer([s_rgb, s_depth, s_info], queue_size=10, slop=0.12)
        self.sync.registerCallback(self.tick)

        rospy.Subscriber("/object_query", String, self.query_cb, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

    def _init_traj_logging(self):
        os.makedirs(self.traj_log_dir, exist_ok=True)
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.traj_log_path = os.path.join(self.traj_log_dir, f"traj_{run_id}.txt")
        self.traj_timer = rospy.Timer(
            rospy.Duration(0.1),
            self._traj_timer_cb,
        )

    # ---------------- Callbacks ----------------
    def amcl_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose.pose

    def query_cb(self, msg: String):
        self.query = msg.data.strip()
        self.hits_in_row = 0
        self.sent_goal_for_this_track = False
        self.text_encoding = self.predictor.encode_text(self.query)
        rospy.loginfo("Updated prompt: %s", self.query)

    # ---------------- TF Replacement ----------------
    def _lookup_transform_manual(self, target_frame, source_frame, stamp):
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, stamp, rospy.Duration(0.3))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, stamp)
            t = Point(x=trans[0], y=trans[1], z=trans[2])
            q = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
            return t, q
        except Exception:
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            t = Point(x=trans[0], y=trans[1], z=trans[2])
            q = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
            return t, q

    def _transform_point(self, point: PointStamped, target_frame: str):
        t, q = self._lookup_transform_manual(target_frame, point.header.frame_id, point.header.stamp)
        p = [point.point.x, point.point.y, point.point.z]
        x, y, z = transform_point_manual(p, t, q)
        new_point = PointStamped()
        new_point.header.frame_id = target_frame
        new_point.header.stamp = rospy.Time.now()
        new_point.point.x = x
        new_point.point.y = y
        new_point.point.z = z
        return new_point

    # ---------------- Main Callback ----------------
    def tick(self, rgb_msg, depth_msg, info_msg):
        # Convert images
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # Run OWL-ViT detection
        img = PILImage.fromarray(cv_rgb)
        detections = self.predictor.predict(img, self.text_encoding)
        if detections:
            best = detections[0]
            x, y = int(best["bbox"][0]), int(best["bbox"][1])
            z = cv_depth[y, x]
            pt = PointStamped()
            pt.header.frame_id = self.camera_frame
            pt.point.x = (x - info_msg.K[2]) * z / info_msg.K[0]
            pt.point.y = (y - info_msg.K[5]) * z / info_msg.K[4]
            pt.point.z = z
            pt_world = self._transform_point(pt, self.target_frame)

            pose = PoseStamped()
            pose.header.frame_id = self.target_frame
            pose.pose.position = pt_world.point
            pose.pose.orientation = Quaternion(0, 0, 0, 1)
            self.pub_pose.publish(pose)
            self.pub_found.publish(Bool(data=True))
        else:
            self.pub_found.publish(Bool(data=False))


if __name__ == "__main__":
    finder = ObjectFinder()
    rospy.spin()