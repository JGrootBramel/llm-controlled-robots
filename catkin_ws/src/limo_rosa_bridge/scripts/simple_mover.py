#!/usr/bin/env python3
"""
Simple motion executor for LIMO cobot (PyKDL-free).

- Base motion via /cmd_vel
- Arm motion via MoveIt (move_group)
"""

import sys
import math
import rospy
import moveit_commander

from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger, TriggerResponse


class SimpleMover:
    def __init__(self):
        rospy.init_node("simple_mover", anonymous=False)

        # ---------- Base ----------
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10
        )

        # ---------- MoveIt ----------
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # 🔴 CHANGE THIS IF NEEDED
        self.arm_group_name = rospy.get_param("~arm_group", "arm")
        self.arm = moveit_commander.MoveGroupCommander(self.arm_group_name)

        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        rospy.loginfo("SimpleMover initialized")
        rospy.loginfo(f"Planning group: {self.arm_group_name}")
        rospy.loginfo(f"End effector: {self.arm.get_end_effector_link()}")

        # ---------- Services ----------
        rospy.Service("~stop_base", Trigger, self.stop_base_srv)
        rospy.Service("~home_arm", Trigger, self.home_arm_srv)
        rospy.Service("~open_gripper", Trigger, self.open_gripper_srv)
        rospy.Service("~close_gripper", Trigger, self.close_gripper_srv)

    # =============================
    # Base control
    # =============================
    def drive(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        rate = rospy.Rate(10)
        ticks = int(duration * 10)

        for _ in range(ticks):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop_base()

    def stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def stop_base_srv(self, _req):
        self.stop_base()
        return TriggerResponse(True, "Base stopped")

    # =============================
    # Arm control (MoveIt)
    # =============================
    def home_arm_srv(self, _req):
        try:
            self.arm.set_named_target("home")
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            return TriggerResponse(success, "Arm homed")
        except Exception as e:
            return TriggerResponse(False, str(e))

    def move_arm_to_pose(self, pose: PoseStamped):
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    # =============================
    # Gripper (best-effort)
    # =============================
    def open_gripper_srv(self, _req):
        return self._gripper_named("open")

    def close_gripper_srv(self, _req):
        return self._gripper_named("close")

    def _gripper_named(self, name):
        try:
            self.arm.set_named_target(name)
            success = self.arm.go(wait=True)
            self.arm.stop()
            return TriggerResponse(success, f"Gripper {name}")
        except Exception:
            return TriggerResponse(
                False,
                "Gripper named target not defined (safe to ignore)"
            )


if __name__ == "__main__":
    try:
        mover = SimpleMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()

