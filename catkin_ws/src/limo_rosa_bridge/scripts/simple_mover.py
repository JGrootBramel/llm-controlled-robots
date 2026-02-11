#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import PyKDL

def main():
    rospy.init_node('rosa_executioner', anonymous=True)
    
    pub = rospy.Publisher('/rosa/test', String, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        msg = "Hello from simple_mover"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
