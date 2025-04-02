#!/usr/bin/env python

import rospy
import sys
import signal
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def odom_cb(msg):
    oreuler = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
    ppoint = msg.pose.pose.position
    y = ppoint.y; x = ppoint.x
    print (f'yaw={yaw}, x={x} y={y}')

def shutdown(sig, stackframe):
    print("Exit because of ^c")
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    sys.exit(0)

# Initialize this program as a node
rospy.init_node('mini_odom_demo')
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
signal.signal(signal.SIGINT, shutdown)

rospy.spin()
