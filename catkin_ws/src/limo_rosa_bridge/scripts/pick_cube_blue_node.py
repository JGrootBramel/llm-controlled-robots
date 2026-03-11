#!/usr/bin/env python3
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
        self.median_patch_px = int(rospy.get_param("~median_patch_px", 9))

        # EXTREME BLUE SETTINGS (Wide Open)
        self.h_low = 90    # Includes Cyan/Teal
        self.h_high = 150  # Includes Deep Purple/Violet
        self.s_low = 30    # See blue even if it's washed out/pale
        self.v_low = 30    # See blue even in deep shadows

        # --- State Tracking ---
        self.require_stable_hits = int(rospy.get_param("~stable_hits", 3))
        self.hit_count = 0
        self.last_p_map = None # Stores the latest coordinate for the grasp
        self.grasp_armed = False 

        # --- MyCobot Hardware Init ---
        self.mc_port = rospy.get_param("~port", "/dev/ttyACM0")
        self.mc_baud = int(rospy.get_param("~baud", 115200))
        try:
            # Ensure you ran: sudo chmod 777 /dev/ttyACM0
            self.mc = MyCobot280(self.mc_port, self.mc_baud)
            # Home pose: Looking forward and slightly down at the floor/box
            self.mc.send_angles([-50.0, 0.0, -10.0, -90.0, 0.0, 57.0], 50)
        except Exception as e:
            rospy.logerr(f"MyCobot Connection Failed: {e}")

        # --- TF & Bridge ---
        self.tfbuf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)
        self.bridge = CvBridge()

        # --- ROS Interfaces for ROSA Tools ---
        self.cube_pub = rospy.Publisher("~cube_map_pose", PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("~detected_cube_markers", Marker, queue_size=10)
        
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
        rospy.loginfo("blue_cube_grasper (Tool Server) ready with Blue HSV Filter")

    def trigger_grasp_cb(self, req):
        """Service callback triggered by LLM to execute the grasp."""
        self.grasp_armed = True
        return TriggerResponse(success=True, message="Looking for cube to grasp...")

    def cb(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # 1. Conversion (keep your existing try/except block)
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)
        except Exception: return

        H, W = rgb.shape[:2]
        fx, fy, cx, cy = info_msg.K[0], info_msg.K[4], info_msg.K[2], info_msg.K[5]

        # 2. Optimized Detection
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_low, self.s_low, self.v_low], dtype=np.uint8)
        upper = np.array([self.h_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # --- REDUCED FILTERING ---
        # We only use a tiny blur so we don't 'erase' the cube
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # SHOW DEBUG WINDOW (This should now show the cube!)
        cv2.imshow("Blue_Filter_Debug", mask)
        cv2.waitKey(1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.hit_count = 0
            return

        best_cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(best_cnt)
        
        if area < self.min_area_px:
            self.hit_count = 0
            return

        M = cv2.moments(best_cnt)
        u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

        # 3. Depth & Transform
        Z = self._median_depth_patch(depth, u, v, H, W)
        if Z is None: return

        Xc, Yc = (u - cx) * Z / fx, (v - cy) * Z / fy
        p_base = self._to_frame(info_msg, rgb_msg, Xc, Yc, Z, self.base_frame)
        p_map = self._to_frame(info_msg, rgb_msg, Xc, Yc, Z, self.target_frame)

        if p_map is not None:
            # Update 'last known' for the grasp service
            self.last_p_map = p_map 
            
            # Marker & Publisher (Keep your existing code here)
            # ... [Existing Marker Logic] ...

        # 4. THE GRASP TRIGGER
        self.hit_count += 1
        if self.hit_count >= self.require_stable_hits and self.grasp_armed and p_base is not None:
            rospy.loginfo("STABLE HIT FOUND! Executing Physical Grasp...")
            self.grasp_armed = False # Reset trigger

            # Map coordinates to Arm Space
            x_top, y_top, z_top = p_base
            X_arm = y_top * 1000.0
            Y_arm = -x_top * 1000.0
            Z_arm = z_top * 1000.0

            if self.do_grasp(X_arm, Y_arm, Z_arm):
                self.do_place()

    def _median_depth_patch(self, depth, u, v, H, W):
        win = self.median_patch_px
        if win % 2 == 0: win += 1
        r = win // 2
        x0, x1 = max(0, u - r), min(W, u + r + 1)
        y0, y1 = max(0, v - r), min(H, v + r + 1)
        patch = depth[y0:y1, x0:x1].copy()
        valid = np.isfinite(patch) & (patch > self.depth_min) & (patch < self.depth_max)
        vals = patch[valid]
        if vals.size < 10: return None
        return float(np.median(vals))

    def _to_frame(self, info_msg, rgb_msg, Xc, Yc, Zc, target_frame):
        """Generalized TF lookup"""
        try:
            src_frame = info_msg.header.frame_id or self.camera_frame
            p_cam = PointStamped()
            p_cam.header.frame_id = src_frame
            p_cam.point.x, p_cam.point.y, p_cam.point.z = float(Xc), float(Yc), float(Zc)

            T_cb = self.tfbuf.lookup_transform(target_frame, src_frame, rospy.Time(0), rospy.Duration(0.3))
            p_base = tf2_geometry_msgs.do_transform_point(p_cam, T_cb).point
            return (float(p_base.x), float(p_base.y), float(p_base.z))
        except Exception:
            return None

    # ... [KEEP YOUR EXISTING do_grasp() HERE] ...
    def do_grasp(self, X_arm_mm, Y_arm_mm, Z_arm_mm):
        try:
            self.mc.set_gripper_state(0, 80)
            rx, ry, rz = -110, 45, 165
            speed = 40
            self.mc.send_coords([X_arm_mm, Y_arm_mm + 30, Z_arm_mm + 10, rx, ry, rz], speed)
            time.sleep(2)
            self.mc.send_coords([X_arm_mm, Y_arm_mm, Z_arm_mm, rx, ry, rz], speed)
            time.sleep(2)
            self.mc.set_gripper_state(1, 80)
            time.sleep(1.5)
            self.mc.send_angles([-77.0, -50.0, -40.0, 100.0, -5.0, 52.0], 50)
            time.sleep(2)
            return True
        except Exception as e:
            return False

    def do_place(self):
        """Rotates the arm to the back and drops the cube on the LIMO tray."""
        try:
            # Note: You may need to tune these joint angles slightly to match your physical tray
            # Joint 1 (Base) rotated ~180 degrees backwards
            self.mc.send_angles([90.0, -30.0, -40.0, 80.0, 0.0, 50.0], 40)
            time.sleep(3.0)
            
            # Open gripper to drop
            self.mc.set_gripper_state(0, 80)
            time.sleep(1.0)
            
            # Return to standard safe pose facing front
            self.mc.send_angles([-90.0, 0.0, -10.0, -90.0, 0.0, 57.0], 50)
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
    def handle_grasp(self, req):
        rospy.loginfo("Grasp service triggered by LLM!")
        # Call your existing arm movement logic here
        self.do_grasp_sequence() 
        return TriggerResponse(success=True, message="Arm sequence complete")
    
    def trigger_grasp_cb(self, req):
        """
        Called by the LLM tool fetch_and_store_cube.
        """
        rospy.loginfo("Service 'execute_grasp' triggered! Starting arm movement...")
        
        if self.last_p_map is None:
            return TriggerResponse(success=False, message="No cube currently in sight to grasp.")

        # Logic to call your do_grasp() function
        success = self.execute_physical_grasp() 
        
        if success:
            return TriggerResponse(success=True, message="Cube picked up and placed on left tray.")
        else:
            return TriggerResponse(success=False, message="Grasp failed: Robot couldn't reach.")

if __name__ == "__main__":
    BlueCubeGrasper()
    rospy.spin()