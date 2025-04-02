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


FORWARD_SPEED = 0.2
SLOW_FORWARD = 0.2
SLOW_ROTATION = 0.3
DEBUG = True
GOAL_DIST = 0.5
GOAL_BEARING = 90
WALL_AHEAD_DIST = 1.0


class Roamer:
    def __init__(self):
        # Initialize this program as a node
        print("init")
        rospy.init_node("roamer")
        signal.signal(signal.SIGINT, self.shutdown)
        # self.path = Path()
        self.twist = Twist()
        # self.twist.linear.x = FORWARD_SPEED
        self.pid = pid.PID(-0.3, 0.3, -0.19, -0.00, 0.00)
        self.state = "start"
        self.initial_yaw = None
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)
        self.smart_lidar_sub = rospy.Subscriber("/sensor", Sensor, self.smart_lidar_cb)

    def radians_norm(self, angle):
        while angle < -pi:
            angle = 2 * pi + angle
        while angle > pi:
            angle = angle - 2 * pi
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

    def facing_wall(self):
        return abs(self.shortest_bearing) < 15
    def facing_away_from_wall(self):
        return abs(self.shortest_bearing) > 150
    def danger_zone(self):
        return self.shortest <= 0.4
    def working_zone(self):
        return 0.4 <= self.shortest <= 1.0
    def far_zone(self):
        return self.shortest > 1.0
    def parallel_wall(self):
        return GOAL_BEARING - 30 < self.shortest_bearing < GOAL_BEARING + 30
    def too_close(self):
        return self.shortest < 0.4
    def too_far(self):
        return self.shortest > 1.2
    def little_close(self):
        return 0.4 < self.shortest < 0.6
    def little_far(self):
        return 0.6 < self.shortest < 1.1
    def stop(self):
        self.twist = self.make_twist(0.0,0.0)
    def go_towards_left(self):
        self.twist = self.make_twist(0.3,-0.4)
    def go_towards_right(self):
        self.twist = self.make_twist(0.3,0.4)

    def process_rules(self, rules):
        print("\nRULES:")
        for rule in rules:
            trigger = eval(rule[1])
            print(rule[0], trigger)
            if (trigger):
                self.print_debug(rule[0])
                eval(rule[2])
                return

    def print_debug(self, arg):
        if DEBUG:
            print(
                f"{arg} ({self.shortest_bearing:2}, {self.shortest:2.3}, {self.forward:2.3}, {self.left:2.3}), ({self.twist.linear.x:1.2}, {self.twist.angular.z:1.2})"
            )


    def smart_lidar_cb(self, msg):
        self.shortest_bearing = self.degrees_norm(msg.shortest_bearing)
        self.left = msg.left
        self.forward = msg.forward
        self.shortest = msg.shortest
        if (self.forward < WALL_AHEAD_DIST):
            self.print_debug("turn")
            self.set_twist(0.0, -SLOW_ROTATION)
        else:
            self.delta_distance = msg.left - GOAL_DIST
            self.delta_bearing = GOAL_BEARING - msg.shortest_bearing
            self.print_debug("follow")
            self.pid_turn = self.pid.compute(GOAL_DIST, self.delta_distance)
            self.set_twist(SLOW_FORWARD, self.pid_turn)

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
    def set_twist(self, forw, rot):
        self.twist = self.make_twist(forw, rot)

    def shutdown(self, sig, stackframe):
        print("Exit because of ^c")
        self.full_stop()
        sys.exit(0)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
        self.full_stop()


# rate object gets a sleep() method which will sleep 1/10 seconds
r = Roamer()
r.run()

# for f in arange(-6*pi, 6*pi, pi/4.0):
#     print(f"{f/2.0:2.4}, {r.radians_norm(f/2.0):1.4}")
