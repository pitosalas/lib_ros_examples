#!/usr/bin/env python

import rospy
import sys
import signal
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

state = "aligning"
twist = Twist()
exit_loop = False

def radians_normalize(angle):
    while(angle<0):
        angle += 2*pi
    while(angle>2*pi):
        angle -= 2*pi
    print (angle)
    return angle

def odom_cb(msg):
    global state
    global twist
    global target_y, target_x
    oreuler = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
    yaw = radians_normalize(yaw)
    ppoint = msg.pose.pose.position
    y = ppoint.y; x = ppoint.x
    print ("state=%s yaw=%0.2f x=%0.2f y=%0.2f" % (state, yaw, x, y))
    if state == "aligning":
        twist.angular.z = 0.5
        if (-0.05 < yaw < 0.05):
            state = "outgoing"
            target_y = y
            target_x = x + 1
    elif state == "outgoing":
        delta_y = y - target_y
        delta_x = x - target_x
        print("outgoing delta_y=%0.2f delta_x=%0.2f" % (delta_y, delta_x))
        twist.angular.z = delta_y * -0.5
        twist.linear.x = 0.1
        if (-0.05 < delta_x < 0.05):
            state = "turning"
            target_x = target_x - 1.0
    elif state == "turning":
        twist.angular.z = 0.5
        twist.linear.x = 0
        if pi-0.05 < yaw < pi+0.05:
            state = "returning"
            target_x = x - 1.0
    elif state == "returning":
        twist.angular.z = 0.0
        twist.linear.x = 0.1
        delta_x = x - target_x
        print (twist)
        if -0.05 < delta_x < 0.05:
            state = "complete"
    elif state == "complete":
        twist.angular.z = 0
        twist.linear.x = 0

def shutdown(sig, stackframe):
    print("Exit because of ^c")
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    sys.exit(0)

# Initialize this program as a node
rospy.init_node('outback')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
signal.signal(signal.SIGINT, shutdown)


# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(20)

while not rospy.is_shutdown() and state != "complete":
    cmd_vel_pub.publish(twist)
    rate.sleep()

