#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import math

class BoxDetector:
    def __init__(self):
        rospy.init_node("box_detector")

        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/box_detection", String, queue_size=10)

        rospy.Subscriber("/camera/image_raw", Image, self.image_cb)
        rospy.loginfo("Box detector started")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []

        colors = {
            "blue": ((100, 150, 50), (140, 255, 255)),
            "red1": ((0, 150, 50), (10, 255, 255)),
            "red2": ((170, 150, 50), (180, 255, 255)),
        }

        for color in ["blue", "red"]:
            if color == "blue":
                mask = cv2.inRange(hsv, *colors["blue"])
            else:
                mask1 = cv2.inRange(hsv, *colors["red1"])
                mask2 = cv2.inRange(hsv, *colors["red2"])
                mask = mask1 | mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > 1000:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = x + w // 2
                    img_center = frame.shape[1] // 2

                    angle = (cx - img_center) * 0.05  # camera calibration
                    distance = 1.5 / math.sqrt(area) # rough estimate

                    data = {
                        "color": color,
                        "distance": round(distance, 2),
                        "angle": round(angle, 2)
                    }

                    self.pub.publish(json.dumps(data))
                    return

if __name__ == "__main__":
    BoxDetector()
    rospy.spin()
