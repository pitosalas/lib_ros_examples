#!/usr/bin/env python

import rospy
from rpsexamples.msg import Sensor
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from math import pi
from rvizmarkerarray import MarkerArrayUtils
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
import bru_utils as bu


"""Analyze information coming from lidar (/scan) and calculate some higher level metrics

Note:   all angles are in radians, 0 < angle < pi
        left angles are positive
        right angles are negative
        forward is zero
        rear is pi
"""

DEBUG = 1

# Bearing names in radians
FRONT_BEAR = 0
LEFT_BEAR = pi / 2
RIGHT_BEAR = -pi / 2
REAR_BEAR = pi

# Bearing names in degrees
FRONT_BEAR_A = 0
LEFT_BEAR_A = 90
RIGHT_BEAR_A = 270
REAR_BEAR_A = 180

MIN_LIDAR_DISTANCE = 0.1  # ignore Lidar reported distances less than this
INVALID_DISTANCE = np.NAN  # use this value instead of an invalid or ignored distance
AVE_SLICE = 5 # Each reading is an average of readings +/- AVE_SLICE/2 degrees

GREEN = ColorRGBA(0, 1, 0, 1)
RED = ColorRGBA(1, 0, 0, 1)
BLUE = ColorRGBA(0, 0, 1, 1)
GREY = ColorRGBA(0.6, 0.6, 0.6, 1)
YELLOW = ColorRGBA(255 / 256.0, 240 / 256.0, 0 / 256.0, 1.0)

FRONT_COLOR = RED
LEFT_COLOR = GREEN
REAR_COLOR = BLUE
RIGHT_COLOR = GREEN
NEAR_COLOR = YELLOW


class SmartLidar:
    def __init__(self):
        self.hertz = 20
        self.shutdown_requested = False
        self.rate = rospy.Rate(self.hertz)

    def shutdown_hook(self):
        self.shutdown_requested = True
        print("\n**** Shutdown Requested ****")

    def marker_array_pub(self):
        mu = MarkerArrayUtils()
        mu.add_marker(1, FRONT_COLOR, FRONT_BEAR, self.front_dist)
        mu.add_marker(2, LEFT_COLOR, LEFT_BEAR, self.left_dist)
        mu.add_marker(3, RIGHT_COLOR, RIGHT_BEAR, self.right_dist)
        mu.add_marker(4, REAR_COLOR, REAR_BEAR, self.rear_dist)
        mu.add_marker(5, NEAR_COLOR, self.near_bear, self.near_dist)
        mu.publish()

    def filter_and_summarize(self, a, i):
        """Given an array a and an index i, replace each entry in the array with the average
        of 'AVE_SLICE' adjacent cells, while filtering out values that are less than MIN_LIDAR_DISTANCE or np.inf"""
        take_indeces = range(int(i - AVE_SLICE / 2), int(i + AVE_SLICE / 2))
        x = np.take(a, take_indeces, mode="wrap")
        y = np.logical_and(x > MIN_LIDAR_DISTANCE, x != np.inf)
        slice = x[y]
        if slice.size == 0:
            print(".",end="")
            return np.NAN
        else:
            return np.average(x[y])

    def scan_callback(self, msg):
        """Return the bearing and distance to the nearest obstacle as well as the distance
        to obstacle front, rear, left and right, in radians"""
        ar = np.array(msg.ranges)
        filter_and_average = [self.filter_and_summarize(ar, x) for x in range(0, ar.size)]
        # Our lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two,
        # gazebo sends back 360 numbers so it doesn't have to be divided by two. The calculation is to
        # make sure the resultant lidar_div is an integer.
        lidar_div = 2 if ar.size == 719 else 1
        try:
            self.near_bear = np.nanargmin(np.around(filter_and_average, decimals=2))
            self.near_dist = filter_and_average[self.near_bear]
            self.near_bear = math.radians(self.near_bear / lidar_div)
        except ValueError:
            print("+",end="")
            self.near_dist = INVALID_DISTANCE
            self.near_bear = INVALID_DISTANCE

        # For Platform we map things because the lidar is mounted backwards

        self.front_dist = filter_and_average[FRONT_BEAR_A * lidar_div]
        self.right_dist = filter_and_average[RIGHT_BEAR_A * lidar_div]
        self.left_dist = filter_and_average[LEFT_BEAR_A * lidar_div]
        self.rear_dist = filter_and_average[REAR_BEAR_A * lidar_div]
        self.near_bear = bu.normalize_angle(self.near_bear)

        # self.front_dist = filter_and_average[REAR_BEAR_A * lidar_div]
        # self.right_dist = filter_and_average[LEFT_BEAR_A * lidar_div]
        # self.left_dist = filter_and_average[RIGHT_BEAR_A * lidar_div]
        # self.rear_dist = filter_and_average[FRONT_BEAR_A * lidar_div]
        # self.near_bear = bu.invert_angle(self.near_bear)
        if DEBUG:
            print(
                "smartlidar nearest: %.2f bearing: %.3f front: %.3f left: %.3f rear: %.3f right: %.3f "
                % (
                    self.near_dist,
                    self.near_bear,
                    self.front_dist,
                    self.left_dist,
                    self.rear_dist,
                    self.right_dist,
                )
            )

    def pre_loop(self):
        self.sensor_pub = rospy.Publisher("/sensor", Sensor, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        bu.wait_for_simulator()
        self.time_now = rospy.Time()
        self.forward_cmd = 0
        self.turn_cmd = 0
        self.near_dist = self.front_dist = self.rear_dist = self.left_dist = self.right_dist = np.NAN
        self.near_bear = math.radians(0)
        self.state_bear = math.radians(0)

    def loop(self):
        sensor_msg = Sensor(
            self.front_dist,
            self.left_dist,
            self.right_dist,
            self.rear_dist,
            self.near_dist,
            math.degrees(self.near_bear),
        )
        self.sensor_pub.publish(sensor_msg)
        self.marker_array_pub()
        self.time_now = rospy.Time.now()

    def run(self):
        self.rate = rospy.Rate(self.hertz)
        self.pre_loop()
        while not rospy.is_shutdown() and not self.shutdown_requested:
            self.loop()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("smartlidar", log_level=rospy.DEBUG)
    ss = SmartLidar()
    rospy.on_shutdown(ss.shutdown_hook)
    ss.pre_loop()
    ss.run()
