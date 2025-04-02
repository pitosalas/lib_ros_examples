#!/usr/bin/env python3

import rospy
import tf2_ros
import signal
import copy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isclose
from scipy.spatial import distance

initial_pose = None
current_pose = Pose2D()
dist = 0.0

def calc_distance(pa, pb):
    return distance.euclidean([pa.x, pa.y], [pb.x, pb.y])

def invert_angle(angle):
    return (angle + pi) % (2 * pi)

def odom_cb(msg):
    global current_pose
    global dist
    global initial_pose

    current_pose.x, current_pose.y = msg.pose.pose.position.x, msg.pose.pose.position.y
    roll, pitch, current_pose.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    if initial_pose is None:
        initial_pose = copy.copy(current_pose)
        print(f"initial:{initial_pose.x:1.2},{initial_pose.y:1.2}")
    dist = calc_distance(current_pose, initial_pose)
    pose_pub.publish(current_pose)

# Initialize this program as a node
rospy.loginfo("init")
rospy.init_node("line")
#signal.signal(signal.SIGINT, shutdown)
odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
rate = rospy.Rate(10)
rospy.loginfo("move forward")
start = rospy.Time.now()
while not rospy.is_shutdown() and dist < 0.5:
    move = Twist()
    move.linear.x = 0.2
    move.angular.z = 0
    movement_pub.publish(move)
    rospy.loginfo(f"dist:{dist:2.2}, x:{current_pose.x:2.2}, y:{current_pose.y:2.2}, t:{current_pose.theta:2.2}")
    rate.sleep()
rospy.loginfo("turn right")
target_theta = invert_angle(current_pose.theta)
print(f"target theta: {target_theta:1.3}")
while not rospy.is_shutdown() and not isclose(target_theta, current_pose.theta, abs_tol=0.2):
    move = Twist()
    move.linear.x = 0
    move.angular.z = 0.5
    movement_pub.publish(move)
    rospy.loginfo(f"dist:{dist:2.2}, x:{current_pose.x:2.2}, y:{current_pose.y:2.2}, t:{current_pose.theta:2.2}")
    rate.sleep()
print("Returning")
while not rospy.is_shutdown() and not isclose(dist, 0, abs_tol=0.2):
    move = Twist()
    move.linear.x = 0.2
    move.angular.z = 0
    movement_pub.publish(move)
    rospy.loginfo(f"dist:{dist:2.2}, x:{current_pose.x:2.2}, y:{current_pose.y:2.2}, t:{current_pose.theta:2.2}")
    rate.sleep()
print("back home")
move.linear.x = 0
move.angular.z = 0
movement_pub.publish(move)
