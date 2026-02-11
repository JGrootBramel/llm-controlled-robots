#!/usr/bin/env python3
import rospy
import PyKDL
import kdl_parser_py.urdf as kdl_parser

def main():
    rospy.init_node('rosa_executioner')

    robot = rospy.get_param('/robot_description', None)
    if robot is None:
        rospy.logerr("No /robot_description found!")
        return

    ok, tree = kdl_parser.treeFromUrdfString(robot)
    if ok:
        rospy.loginfo("KDL Tree successfully parsed")
    else:
        rospy.logerr("Failed to parse KDL Tree")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
