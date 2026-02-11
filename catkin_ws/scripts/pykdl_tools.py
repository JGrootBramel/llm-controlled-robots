#!/usr/bin/env python3
import rospy
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

class PyKDLTools:
    def __init__(self):
        rospy.init_node("pykdl_tools")

        rospy.loginfo("Loading URDF from parameter server")
        robot = URDF.from_parameter_server()

        ok, self.tree = treeFromUrdfModel(robot)
        if not ok:
            rospy.logfatal("Failed to build KDL tree")
            return

        rospy.loginfo(
            "PyKDL ready with %d segments",
            self.tree.getNrOfSegments()
        )

        rospy.spin()

if __name__ == "__main__":
    PyKDLTools()
