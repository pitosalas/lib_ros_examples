#!/usr/bin/env python3

import rospy
import sys
from rpsexamples.msg import Sensor
import signal
import pid
from math import pi, sqrt, atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import arange
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


FORWARD_SPEED = 0.5
SLOW_FORWARD = 0.3
ROTATION = 0.4
DEBUG = True
GOAL_DIST = 0.3
GOAL_BEARING = 90

class Roamer:
    def __init__(self):
        # Initialize this program as a node
        rospy.init_node("roamer")
        signal.signal(signal.SIGINT, self.shutdown)
        self.path = Path()
        self.twist = Twist()
        self.twist.linear.x = FORWARD_SPEED
        self.pid = pid.PID(-0.8, 0.5, 0.5, 0.0, 0.0)
        self.state = "start"
        self.initial_yaw = None
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.smart_lidar_sub = rospy.Subscriber("/sensor", Sensor, self.smart_lidar_cb)
        if DEBUG:
            print(f"type, short bear, short dist, delta bear, delta dist")


    def radians_norm(self, angle):
        while angle < -pi:
            angle = 2*pi + angle
        while angle > pi:
            angle = angle - 2*pi
        return angle

    def degrees_norm(self, angle):
        while angle < -180:
            angle = 360 + angle
        while angle > 180:
            angle = angle - 360
        return angle

    def distance_from(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def pose(self, msg):
        """extract and convert pose components we need"""
        opoint = msg.pose.pose.position
        y = opoint.y
        x = opoint.x
        oquat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oquat.x, oquat.y, oquat.z, oquat.w])
        # yaw = atan2(2.0 * (oquat.w * oquat.z + oquat.x * oquat.y), oquat.w * oquat.w + oquat.x * oquat.x - oquat.y * oquat.y - oquat.z * oquat.z);
        return x, y, yaw

    def print_debug(self, arg):
        if DEBUG:
            print(
                f"{arg}{self.shortest_bearing:2},{self.shortest:2.3},{self.delta_bearing:2},{self.delta_distance:2.3}"
            )

    def smart_lidar_cb(self, msg):
        self.shortest_bearing = self.degrees_norm(msg.shortest_bearing)
        self.shortest = msg.shortest
        self.delta_distance = msg.shortest - GOAL_DIST
        self.delta_bearing = self.degrees_norm(msg.shortest_bearing - GOAL_BEARING)

        # if self.delta_distance > 0.4 and -5 < self.shortest_bearing < 5:
        if self.delta_distance >= 0.4 and -25 < self.shortest_bearing < 25:
            self.print_debug("far ok,  ")
            self.twist = self.make_twist(0.3, 0)
        elif self.delta_distance >= 0.4:
            self.print_debug("far,     ")
            #self.twist = self.make_twist(0, 0.4)
            self.twist = self.make_twist(0, 0.6)
        # elif self.delta_distance < 0.5  and -25 < self.delta_bearing < 25:
        elif self.delta_distance < 0.4  and -25 < self.delta_bearing < 25:
            self.print_debug("close ok,")
            self.twist = self.make_twist(0.3, self.delta_distance)
        elif self.delta_distance < 0.4:
        # elif self.delta_distance < 0.5:
            self.print_debug("close,   ")
            #rotation = self.delta_bearing * 0.01
            rotation = self.delta_bearing * 0.006
            self.twist = self.make_twist(0.1, rotation)
        else:
            self.print_debug("error,   ")

    def full_stop(self):
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

    def make_twist(self, forward_speed, rotation_speed):
        t = Twist()
        t.linear.x = forward_speed
        t.angular.z = rotation_speed
        return t

    def get_twist(self, t):
        return t.linear.x, t.angular.z

    def shutdown(self, sig, stackframe):
        print("Exit because of ^c")
        self.full_stop()
        sys.exit(0)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist)
            # print(self.get_twist(self.twist))
            rate.sleep()
        self.full_stop()


# rate object gets a sleep() method which will sleep 1/10 seconds
r = Roamer()
r.run()

# for f in arange(-6*pi, 6*pi, pi/4.0):
#     print(f"{f/2.0:2.4}, {r.radians_norm(f/2.0):1.4}")
