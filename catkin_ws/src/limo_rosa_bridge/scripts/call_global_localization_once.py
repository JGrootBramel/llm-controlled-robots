#!/usr/bin/env python3
"""Call /global_localization once, then optionally rotate in place."""

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


def main() -> None:
    rospy.init_node("call_global_localization_once", anonymous=False)

    service_name = rospy.get_param("~service_name", "/global_localization")
    timeout_s = float(rospy.get_param("~wait_timeout_s", 20.0))
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    rotate_after_global = bool(rospy.get_param("~rotate_after_global", True))
    rotate_angular_speed = float(rospy.get_param("~rotate_angular_speed", 0.4))
    rotate_duration_s = float(rospy.get_param("~rotate_duration_s", 16.0))
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))

    try:
        rospy.loginfo("Waiting for service %s (timeout %.1fs)...", service_name, timeout_s)
        rospy.wait_for_service(service_name, timeout=timeout_s)
        call_global_localization = rospy.ServiceProxy(service_name, Empty)
        call_global_localization()
        rospy.loginfo("Called %s successfully.", service_name)
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logwarn("Failed to call %s: %s", service_name, exc)
        return

    if not rotate_after_global:
        return

    if rotate_duration_s <= 0.0 or rotate_angular_speed == 0.0:
        rospy.loginfo("Skipping rotate step due to zero duration/speed.")
        return

    cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    rate = rospy.Rate(max(publish_rate_hz, 1.0))
    end_time = rospy.Time.now() + rospy.Duration(rotate_duration_s)
    twist = Twist()
    twist.angular.z = rotate_angular_speed

    rospy.loginfo(
        "Rotating in place for %.1fs at %.3f rad/s on %s.",
        rotate_duration_s,
        rotate_angular_speed,
        cmd_vel_topic,
    )
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        cmd_pub.publish(twist)
        rate.sleep()

    cmd_pub.publish(Twist())
    rospy.loginfo("Rotate step complete; published stop command.")


if __name__ == "__main__":
    main()
