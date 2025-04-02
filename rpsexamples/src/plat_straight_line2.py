#!/usr/bin/env python3

import signal
import rospy
import copy
from typing import Callable
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt, atan2, isclose
from basenode2 import BaseNode
import bru_utils as bu


class StraightLine(BaseNode):
    def __init__(self):
        super().__init__()
        self.initial_pose: Pose2D
        self.current_pose: Pose2D
        self.dist: float = 0.0
        self.odom_sub: rospy.Subscriber
        self.movement_pub: rospy.Publisher = None
        self.pose_pub: rospy.Publisher = None
        self.start: rospy.Time
        self.state: str
        self.log_throttle : int = 0

    def log(self, label: str):
        bu.info(f"{label}: {bu.pose_to_str(self.current_pose)}")

    def log_throttled(self, label: str):
        self.log_throttle += 1
        if self.log_throttle == 10:
            self.log(label)
            self.log_throttle = 0

    def odom_cb(self, msg):
        self.current_pose.x, self.current_pose.y = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        _, _, self.current_pose.theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        if self.initial_pose is None:
            self.initial_pose = copy.copy(self.current_pose)
        self.dist = bu.calc_distance(self.current_pose, self.initial_pose)
        self.pose_pub.publish(self.current_pose)

    def initial_setup(self):
        super().initial_setup()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
        self.rate = rospy.Rate(10)
        self.start = rospy.Time.now()
        self.dist = 0.0
        self.current_pose = Pose2D()
        self.initial_pose = None
        while self.initial_pose is None:
            self.rate.sleep()

    def drive_straight_line(self, arrived) -> None:
        bu.info("straight driving")
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
        self.rate.sleep()
        while not arrived(self.dist)  and not self.shutdown_requested:
            self.log("fwd")
            self.movement_pub.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0.0
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def turn_by_radians(self, arrived: float) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        while not arrived(self.current_pose.theta) and not self.shutdown_requested:
            self.log_throttled("[tbr]")
            self.movement_pub.publish(twist)
            self.rate.sleep()
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def drive_to(self, goal_pose: Pose2D):
        starting_pose : Pose2D = self.current_pose
        self.log(f"starting: {bu.pose_to_str(starting_pose)} goal: {bu.pose_to_str(goal_pose)}")
        while not self.shutdown_requested:
            linear_distance = bu.calc_distance(starting_pose, goal_pose)
            angular_offset = goal_pose.theta - self.current_pose.theta
            twist = Twist()
            twist.linear.x = pid_linear.compute(0, linear_distance)
            twist.angular.z = pid_angular.compute(0, angular_offset)
            if isclose(linear_distance, 0, abs_tol=0.25):
                twist.linear.x = 0.0
                print(".")
            if isclose(angular_offset, 0, abs_tol=0.25):
                twist.angular.z = 0.0
                print("*")
            self.log_throttled(f"[dt] ld:{linear_distance:1.3} ao:{angular_offset:1.2} tlx:{twist.linear.x:1.2} taz:{twist.angular.z:1.2}")
            if isclose(linear_distance, 0, abs_tol=0.25) and isclose(angular_offset, 0, abs_tol=0.25):
                break
            self.movement_pub.publish(twist)
            self.rate.sleep()


    def drive_to_0(self, goal_pose: Pose2D):
        pid_linear = bu.PID(-0.5, 0.5, -0.4, 0.0, 0)
        pid_angular = bu.PID(-0.5, 0.5, -0.4, 0, 0)
        starting_pose : Pose2D = self.current_pose
        self.log(f"starting: {bu.pose_to_str(starting_pose)} goal: {bu.pose_to_str(goal_pose)}")
        while not self.shutdown_requested:
            linear_distance = bu.calc_distance(starting_pose, goal_pose)
            angular_offset = goal_pose.theta - self.current_pose.theta
            twist = Twist()
            twist.linear.x = pid_linear.compute(0, linear_distance)
            twist.angular.z = pid_angular.compute(0, angular_offset)
            if isclose(linear_distance, 0, abs_tol=0.25):
                twist.linear.x = 0.0
                print(".")
            if isclose(angular_offset, 0, abs_tol=0.25):
                twist.angular.z = 0.0
                print("*")
            self.log_throttled(f"[dt] ld:{linear_distance:1.3} ao:{angular_offset:1.2} tlx:{twist.linear.x:1.2} taz:{twist.angular.z:1.2}")
            if isclose(linear_distance, 0, abs_tol=0.25) and isclose(angular_offset, 0, abs_tol=0.25):
                break
            self.movement_pub.publish(twist)
            self.rate.sleep()

    def stop(self) -> None:
        super().stop()
        twist = Twist()
        self.movement_pub.publish(twist)

    def loop(self):
        super().loop()
        self.drive_to(bu.offset_point(self.current_pose, 2.0))
        target_theta = bu.invert_angle(self.current_pose.theta)
        self.turn_by_radians(lambda d: isclose(target_theta, self.current_pose.theta, abs_tol=0.4))
        self.stop()


if __name__ == "__main__":
    rospy.init_node("plat_straight_line2")
    rn = StraightLine()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
