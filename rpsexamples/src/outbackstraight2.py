#!/usr/bin/env python

import rospy
import sys
import signal
import pid
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

twist = Twist()
exit_loop = False

pid = pid.PID(-pi/2, pi/2, 0.6, 0.0, 0.1)

def radians_normalize(angle):
    '''Make sure any radian angle is 0 < angle < 2*PI'''
    while(angle<0):
        angle += 2*pi
    while(angle>2*pi):
        angle -= 2*pi
    print angle
    return angle

def kinematics(current_z, speed, turn):
    '''
    speed is in meters/second
    turn is in radians, positive = counter clockwise
    '''
    t = Twist()
    t.linear.x = speed
    t.angular.z = turn - current_z
    return t

def pose(msg):
    ''' extract and convert pose components we need '''
    opoint = msg.pose.pose.position
    y = opoint.y; 
    x = opoint.x
    oreuler = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
    return x, y, yaw

def odom_cb(msg):
    ''' keep on line y=0 '''
    global twist
    x, y, yaw = pose(msg)
    pid_turn = pid.compute(0, y)
    twist = kinematics(yaw, 0.8, pid_turn)
    print("turn: %f odom x: %0.2f y: %00.2f cmdvel x: %0.2f z: %0.2f" % 
            (pid_turn, x, y, twist.linear.x, twist.angular.z))

def shutdown(sig, stackframe):
    print("Exit because of ^c")
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    sys.exit(0)

# Initialize this program as a node
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
rospy.init_node('outback')
signal.signal(signal.SIGINT, shutdown)

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    cmd_vel_pub.publish(twist)
    rate.sleep()

twist.angular.z = 0
twist.linear.x = 0
cmd_vel_pub.publish(twist)

