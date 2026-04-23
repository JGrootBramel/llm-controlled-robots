#!/usr/bin/env python3
"""Wait for AMCL global-localization service and call it once."""

import rospy
from std_srvs.srv import Empty


def main() -> None:
    rospy.init_node("call_global_localization_once", anonymous=False)

    service_name = rospy.get_param("~service_name", "/global_localization")
    timeout_s = float(rospy.get_param("~wait_timeout_s", 20.0))

    try:
        rospy.loginfo("Waiting for service %s (timeout %.1fs)...", service_name, timeout_s)
        rospy.wait_for_service(service_name, timeout=timeout_s)
        call_global_localization = rospy.ServiceProxy(service_name, Empty)
        call_global_localization()
        rospy.loginfo("Called %s successfully.", service_name)
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logwarn("Failed to call %s: %s", service_name, exc)


if __name__ == "__main__":
    main()
