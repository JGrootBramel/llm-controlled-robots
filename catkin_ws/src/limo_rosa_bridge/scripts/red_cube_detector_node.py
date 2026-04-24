#!/usr/bin/env python3
"""HSV-based red cube detector.

Pure perception node. Subscribes to the Astra Pro RGB-D topics, detects the
largest red blob with :mod:`_helpers.hsv`, back-projects it into the map
frame using :mod:`_helpers.projection`, and publishes:

* ``/red_cubes/latest_pose``   (``geometry_msgs/PoseStamped``, latched)
* ``/red_cubes/found``         (``std_msgs/Bool``, latched)
* ``/red_cubes/markers``       (``visualization_msgs/Marker``)

Services:

* ``~enable``    (``std_srvs/SetBool``)  — suspend/resume publishing
* ``~snapshot``  (``std_srvs/Trigger``)  — immediately publish the latest
                                            buffered detection

No motion, no arm, no move_base. The approach and grasp live in their own
nodes.
"""

import os
import sys
from threading import Lock

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

# Allow ``from _helpers import hsv, projection`` when the script is invoked
# directly (CMakeLists installs the package into the same folder).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _helpers import hsv as hsv_helpers  # noqa: E402
from _helpers import projection as proj_helpers  # noqa: E402


class RedCubeDetector:
    def __init__(self):
        self._load_params()
        self._init_ros_comm()

        self.bridge = CvBridge()
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)
        self._tf_point_fn = proj_helpers.make_tf2_transform_point_fn(self.tfbuf)

        self._lock = Lock()
        self._last_pose = None  # PoseStamped or None
        self._found_pub.publish(Bool(data=False))
        self._enabled = True

    # -------------------------------------------------------------- params
    def _load_params(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_depth_optical_frame"
        )
        self.target_frame = rospy.get_param("~target_frame", "map")

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/depth/image_raw"
        )
        self.info_topic = rospy.get_param(
            "~info_topic", "/camera/color/camera_info"
        )

        self.depth_min = float(rospy.get_param("~depth_min", 0.10))
        self.depth_max = float(rospy.get_param("~depth_max", 2.50))
        self.min_area_px = int(rospy.get_param("~min_area_px", 50))
        self.max_area_px = int(rospy.get_param("~max_area_px", 4000))
        self.median_patch_px = int(rospy.get_param("~median_patch_px", 9))
        self.publish_debug = bool(rospy.get_param("~publish_debug", False))

        self._thresholds = hsv_helpers.RedThresholds(
            h1_low=int(rospy.get_param("~red_h1_low", hsv_helpers.DEFAULT_RED_H1[0])),
            h1_high=int(rospy.get_param("~red_h1_high", hsv_helpers.DEFAULT_RED_H1[1])),
            h2_low=int(rospy.get_param("~red_h2_low", hsv_helpers.DEFAULT_RED_H2[0])),
            h2_high=int(rospy.get_param("~red_h2_high", hsv_helpers.DEFAULT_RED_H2[1])),
            s_low=int(rospy.get_param("~red_s_low", hsv_helpers.DEFAULT_RED_S_LOW)),
            v_low=int(rospy.get_param("~red_v_low", hsv_helpers.DEFAULT_RED_V_LOW)),
        )

    # ------------------------------------------------------------ ros comm
    def _init_ros_comm(self):
        self._pose_pub = rospy.Publisher(
            "/red_cubes/latest_pose", PoseStamped, queue_size=1, latch=True
        )
        self._found_pub = rospy.Publisher(
            "/red_cubes/found", Bool, queue_size=1, latch=True
        )
        self._marker_pub = rospy.Publisher(
            "/red_cubes/markers", Marker, queue_size=10
        )
        self._debug_pub = rospy.Publisher(
            "~debug_image", Image, queue_size=1
        ) if self.publish_debug else None

        rospy.Service("~enable", SetBool, self._handle_enable)
        rospy.Service("~snapshot", Trigger, self._handle_snapshot)

        s_rgb = message_filters.Subscriber(self.rgb_topic, Image)
        s_depth = message_filters.Subscriber(self.depth_topic, Image)
        s_info = message_filters.Subscriber(self.info_topic, CameraInfo)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [s_rgb, s_depth, s_info], queue_size=10, slop=0.15
        )
        self._sync.registerCallback(self._cb)

    # -------------------------------------------------------------- service
    def _handle_enable(self, req):
        self._enabled = bool(req.data)
        if not self._enabled:
            self._found_pub.publish(Bool(data=False))
        msg = "enabled" if self._enabled else "disabled"
        return SetBoolResponse(success=True, message=f"red_cube_detector {msg}")

    def _handle_snapshot(self, _req):
        with self._lock:
            pose = self._last_pose
        if pose is None:
            return TriggerResponse(
                success=False, message="No red cube detected yet."
            )
        self._pose_pub.publish(pose)
        return TriggerResponse(
            success=True,
            message=(
                f"Last detection at ({pose.pose.position.x:.2f}, "
                f"{pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}) "
                f"in {pose.header.frame_id}."
            ),
        )

    # ---------------------------------------------------------------- main
    def _cb(self, rgb_msg, depth_msg, info_msg):
        if not self._enabled:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "red_cube_detector cv_bridge: %s", e)
            return

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = hsv_helpers.red_mask(hsv, self._thresholds)
        blob = hsv_helpers.largest_blob_centroid(
            mask, min_area=self.min_area_px, max_area=self.max_area_px
        )
        if blob is None:
            self._found_pub.publish(Bool(data=False))
            self._publish_debug(rgb, None)
            return
        u_col, v_col, _area = blob

        Hc, Wc = rgb.shape[:2]
        Hd, Wd = depth.shape[:2]
        u_d, v_d = hsv_helpers.color_to_depth_pixels(u_col, v_col, Wc, Hc, Wd, Hd)
        z = hsv_helpers.median_depth_patch(
            depth, u_d, v_d,
            patch_px=self.median_patch_px,
            depth_min=self.depth_min,
            depth_max=self.depth_max,
        )
        if z is None:
            self._found_pub.publish(Bool(data=False))
            self._publish_debug(rgb, (u_col, v_col))
            return

        K = info_msg.K
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        src_frame = info_msg.header.frame_id or self.camera_frame
        stamp = rgb_msg.header.stamp
        p_base = proj_helpers.camera_pixel_to_frame(
            u_d, v_d, z, fx, fy, cx, cy,
            src_frame, self.base_frame, self._tf_point_fn, stamp,
        )
        if p_base is None:
            self._publish_debug(rgb, (u_col, v_col))
            return
        p_map = proj_helpers.camera_pixel_to_frame(
            u_d, v_d, z, fx, fy, cx, cy,
            src_frame, self.target_frame, self._tf_point_fn, stamp,
        )
        if p_map is None:
            self._publish_debug(rgb, (u_col, v_col))
            return

        pose = PoseStamped()
        pose.header.stamp = rgb_msg.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(p_map[0])
        pose.pose.position.y = float(p_map[1])
        # Publish full 3D target coordinates for downstream grasping.
        pose.pose.position.z = float(p_map[2])
        # Orient the pose to face the robot (for downstream approach planners).
        try:
            T = self.tfbuf.lookup_transform(
                self.target_frame, self.base_frame,
                rospy.Time(0), rospy.Duration(0.1),
            )
            yaw = np.arctan2(
                float(p_map[1]) - T.transform.translation.y,
                float(p_map[0]) - T.transform.translation.x,
            )
        except Exception:
            yaw = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        with self._lock:
            self._last_pose = pose

        self._pose_pub.publish(pose)
        self._found_pub.publish(Bool(data=True))
        self._publish_marker(pose)
        self._publish_debug(rgb, (u_col, v_col))

    # -------------------------------------------------------------- helpers
    def _publish_marker(self, pose):
        m = Marker()
        m.header.frame_id = pose.header.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = "red_cubes"
        gx = int(pose.pose.position.x * 10.0)
        gy = int(pose.pose.position.y * 10.0)
        m.id = gx + gy * 1000
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = pose.pose.position.x
        m.pose.position.y = pose.pose.position.y
        m.pose.position.z = 0.025
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        self._marker_pub.publish(m)

    def _publish_debug(self, rgb, pick):
        if self._debug_pub is None:
            return
        try:
            img = rgb.copy()
            if pick is not None:
                cv2.drawMarker(
                    img, (int(pick[0]), int(pick[1])),
                    (0, 255, 255), cv2.MARKER_CROSS, 20, 2,
                )
            self._debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("red_cube_detector")
    RedCubeDetector()
    rospy.loginfo("[red_cube_detector] ready")
    rospy.spin()
