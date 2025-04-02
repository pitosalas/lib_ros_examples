#!/usr/bin/env python

import rospy
import basenode
from rpsexamples.msg import Sensor
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from strollerkalman import kalman_predict, kalman_update
from math import pi
from rvizmarkerarray import MarkerArrayUtils
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist

class SmartSensor(basenode.BaseNode):

    FRONT_BEAR=0
    RIGHT_BEAR=270
    REAR_BEAR=180
    LEFT_BEAR=90

    GREEN = ColorRGBA(0, 1, 0, 1)
    RED = ColorRGBA(1, 0, 0, 1)
    GREY = ColorRGBA(0.6, 0.6, 0.6, 1)

    def invert_angle(self,angle):
        return (angle + pi) % (2 * pi)

    def marker_array_pub(self):
        mu = MarkerArrayUtils()
        # mu.add_marker(1, grey, invert_angle(radians(FRONT_BEAR)), forward)
        mu.add_marker(2, self.GREY, self.invert_angle(math.radians(self.LEFT_BEAR)), self.left_dist)
        # mu.add_marker(3, grey, invert_angle(radians(RIGHT_BEAR)), right)
        # mu.add_marker(4, grey, invert_angle(radians(REAR_BEAR)), rear)
        mu.add_marker(5, self.RED, self.invert_angle(self.near_bear), self.near_dist)
        mu.publish()

    def filter(self, a, i):
        """Given an array a and an index i, replace each entry in the array with the average
        of three adjacent cells, while filtering out values that are less than 0.05 or np.inf"""
        x = np.take(a, [i, i-1, i+1], mode='wrap')
        y = np.logical_and(x>0.05, x!=np.inf)
        slice = x[y]
        if slice.size == 0:
            return np.NAN
        else:
            return np.average(x[y])

    def scan_callback(self, msg):
        """Return the bearing and distance to the nearest obstacle as well as the distance
        to obstacle front, rear, left and right, in radians"""
        ar = np.array(msg.ranges)
        filter_and_average = [self.filter(ar,x) for x in range(0, ar.size)]
        self.near_bear = np.nanargmin(np.around(filter_and_average, decimals=2))
        self.near_dist = filter_and_average[self.near_bear]
    # minirover's lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two, 
    # gazebo sends back 360 numbers so it doesn't have to be divided by two. The calculation is to
    # make sure the resultant lidar_div is an integer.
        lidar_div = int(ar.size/360.0 + 0.5)
        self.near_bear = math.radians(self.near_bear/lidar_div)
        self.front_dist = filter_and_average[self.FRONT_BEAR*lidar_div]
        self.right_dist = filter_and_average[self.RIGHT_BEAR*lidar_div]
        self.left_dist = filter_and_average[self.LEFT_BEAR*lidar_div]
        self.rear_dist = filter_and_average[self.REAR_BEAR*lidar_div]
        #print("smartsensor nearest: %.2f bearing: %.3f front: %.3f left: %.3f rear: %.3f right: %.3f " % (self.near_dist, math.degrees(self.near_bear), self.front_dist, self.left_dist, self.rear_dist, self.right_dist))

    def cmd_vel_callback(self,msg):
        """Whenever there's a new command for motion this will be incorporated, using the Kalman FIlter
        into the estimated nearest bearing and distance"""
        self.forward_cmd = msg.linear.x
        self.turn_cmd = msg.angular.z

    def pre_loop(self):
        self.sensor_pub = rospy.Publisher('/sensor', Sensor, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.wait_for_simulator()
        self.time_now = rospy.Time()
        self.forward_cmd = 0
        self.turn_cmd = 0
        self.state_dist = self.near_dist = self.front_dist = self.rear_dist = self.left_dist = self.right_dist = 0.0
        self.near_bear = math.radians(0)
        self.state_bear = math.radians(0)


    def loop(self):
        elapsed = rospy.Time.now().to_sec() - self.time_now.to_sec()
        control_motion = self.forward_cmd * elapsed
        state_dist_temp, state_bear_temp = kalman_predict(self.state_dist, self.state_bear, control_motion)
        self.state_dist, self.state_bear = kalman_update(0.4, state_dist_temp, state_bear_temp, self.near_dist, self.near_bear)
        sensor_msg = Sensor(self.front_dist, self.left_dist, self.right_dist, self.rear_dist, self.state_dist, self.state_bear)
        
        self.sensor_pub.publish(sensor_msg)
        self.marker_array_pub()
        self.time_now = rospy.Time.now()
    
if __name__ == '__main__':
    rospy.init_node('prrsensor', log_level=rospy.DEBUG)
    ss = SmartSensor()
    rospy.on_shutdown(ss.shutdown_hook)
    ss.run()

