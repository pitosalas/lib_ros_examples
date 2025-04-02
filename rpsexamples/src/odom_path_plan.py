#!/usr/bin/env python3

from basenode2 import BaseNode
from geometry_msgs.msg import Pose2D
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate



class OdomPathPlan(BaseNode):
    def set_path(self, path):
        self.path = path

if __name__ == "__main__":
    rospy.init_node("OdomPathPlan")
    rn = OdomPathPlan()
    rospy.on_shutdown(rn.shutdown_hook)

    # Desired path as x,y odometry coordinates. For now we ignore theta
    # Path is assumed to start whereever the current odometry is.

    path = [[2, 0], [2, 2], [4, 2], [4, 0]]
    rn.set_path(path)
    rn.run()
