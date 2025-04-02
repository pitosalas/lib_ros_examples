#!/usr/bin/env python

#August Soderberg
#asoderberg@brandeis.edu   
#PA: ROS Roomba
#Autonomous Robotics
#9/21/2019

#This node is responsible for telling the robot where to drive

import rospy

from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from constants import *
import random
import math


angle_distance_tolerance = 16

#The state is decided and published by the scan_values_handler.
#This function just makes the published value a global variable
def cb_state(msg):
    global state
    state = msg.data

#The direction which the robot should turn is decided and published by the scan_values_handler.
#This function just makes the published value a global variable
def cb_direction(msg):
    global direction
    direction = msg.data


def odom_cb(msg):
    global pose
    pose = msg.pose
    global goal_angle
    global distance_to_home
    if start_pose: 
        #print((start_pose.pose.position.y - pose.pose.position.y)/(start_pose.pose.position.x - pose.pose.position.x))
        goal_angle = PI + math.atan((start_pose.pose.position.y - pose.pose.position.y)/(start_pose.pose.position.x - pose.pose.position.x))
        # if goal_angle > PI: goal_angle = -PI + (goal_angle - PI)
        print("GOAL ANGLE: " + str(goal_angle))
        orientation = pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        current_angle = yaw
        print("CURRENT ANGLE: " + str(current_angle))
        distance_to_home = ((start_pose.pose.position.x - pose.pose.position.x) ** 2 + (start_pose.pose.position.y - pose.pose.position.y) ** 2) ** 0.5
        print("DISTANCE: " + str(distance_to_home))

def close_angles(current_angle, goal_angle):
    if goal_angle > 2 * PI - PI/angle_distance_tolerance and current_angle < -2 * PI + PI /angle_distance_tolerance: return "IN RANGE"
    if current_angle > 2 * PI - PI/angle_distance_tolerance and goal_angle < -2 * PI + PI /angle_distance_tolerance: return "IN RANGE"
    if abs(goal_angle - current_angle) < PI / angle_distance_tolerance: return "IN RANGE"
    return "NOT IN RANGE"
    # if current_angle < goal_angle + PI/angle_distance_tolerance and current_angle > goal_angle - PI/angle_distance_tolerance: return "IN RANGE"
    # if current_angle > goal_angle: return "LEFT"
    # return "RIGHT"

def choose_direction_to_turn(current_angle, goal_angle):
    direction = 0.5
    if goal_angle > current_angle: 
        direction = 0.5
    else:
        direction = -0.5
    if abs(goal_angle - current_angle) > PI: direction *= -1
    return direction


#Set up node and pubs/subs
rospy.init_node('pilot')
sub_state = rospy.Subscriber('state', Int16, cb_state)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
sub_direction = rospy.Subscriber('turn_direction', Bool, cb_direction)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Create a twist and rate object
t = Twist()
rate = rospy.Rate(2)

#Starting states, they will quickly be overwritten
state = WAITING
direction = TURN_RIGHT
pose = None
comeback_state = 0
goal_angle = 0
start_pose = None
while rospy.Time.now().nsecs == 0: continue
start_time = rospy.Time.now()

while pose == None: continue
start_pose = pose

driving_back = False
distance_to_home = None

#This entire while loop just says if we should be driving, drive forward, if we should be turning,
#turn in the direction we received from scan_values_handler, if we haven't gotten any info yet
#just wait, and anything else print an error.
while not rospy.is_shutdown():
    if rospy.Time.now() - start_time < rospy.Duration(secs=10):
        if state == DRIVING:
            t.angular.z = 0
            t.linear.x = LINEAR_SPEED
            pub.publish(t)
        elif state == SPINNING:
            if direction == TURN_RIGHT:
                t.angular.z = ANGULAR_SPEED*RIGHT
            else:
                t.angular.z = ANGULAR_SPEED*LEFT
            t.linear.x = 0
            while (state != DRIVING):
                pub.publish(t)
            starting_time = rospy.Time.now().to_sec()
            end_time = random.random()*PI / 2 / ANGULAR_SPEED
            while (rospy.Time.now().to_sec() - starting_time < end_time):
                pub.publish(t)
        elif state == WAITING:
            rate.sleep()
        else:
            print("ERROR: State Not Recognized")
            rospy.spin()
    else:
        orientation = pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        current_angle = yaw
        print("GOAL ANGLE: " + str(goal_angle))
        print("CURRENT ANGLE: " + str(current_angle))
        result = close_angles(current_angle, goal_angle)
        direction = choose_direction_to_turn(current_angle, goal_angle)
        # if result == "LEFT":
        #     t = Twist()
        #     t.angular.z = -0.5
        #     pub.publish(t)
        #     print("TURNING L")
        # if result == "RIGHT":
        #     t = Twist()
        #     t.angular.z = 0.5
        #     pub.publish(t)
        #     print("TURNING R")
        if distance_to_home < 0.25:
            pub.publish(Twist())
        elif result == "IN RANGE":
            t = Twist()
            t.linear.x = 0.5
            pub.publish(t)
            print("DRIVING")
            driving_back = True
        else:
            t = Twist()
            t.angular.z = direction
            pub.publish(t)
            print("TURNING")