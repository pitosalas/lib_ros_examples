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
        if self.log_throttle >= 1:
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
        self.rate = rospy.Rate(20)
        self.start = rospy.Time.now()
        self.dist = 0.0
        self.current_pose = Pose2D()
        self.initial_pose = None
        while self.initial_pose is None:
            self.rate.sleep()
        self.state = "straight"
        self.goal_pose = bu.offset_point(self.initial_pose, 10.0)
        self.pid_linear = bu.PID(-1.0, 1.0, -0.3, 0.0, 0.0)
        self.pid_angular = bu.PID(-1.0, 1.0, 0.8, 0.0, 0.0)


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

    def turn_by_radians(self, arrived) -> None:
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
        linear_distance = bu.calc_distance(self.current_pose, goal_pose)
        angular_offset = bu.normalize_angle(self.current_pose.theta - self.goal_pose.theta)
        twist = Twist()
        twist.linear.x = self.pid_linear.compute(0, linear_distance)
        twist.angular.z = self.pid_angular.compute(0, angular_offset)
        self.log_throttled(f"ldist: {linear_distance:1.2} angoff: {angular_offset:1.2} goal: {bu.pose_to_str(goal_pose)}, twistX:{twist.linear.x:1.2} twistA:{twist.angular.z:1.2}")
        self.movement_pub.publish(twist)
        self.rate.sleep()


    def drive_to_0(self, goal_pose: Pose2D):
        pid_linear = bu.PID(-0.5, 0.5, -3.0, 0.0, 0)
        pid_angular = bu.PID(-0.5, 0.5, 3.0, 0, 0)
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

    def stop(self) -> None:
        super().stop()
        twist = Twist()
        self.movement_pub.publish(twist)

    def update_state(self):
        if (self.state == "straight" and bu.poses_close(self.goal_pose, self.current_pose)):
            self.state = "done"

    def loop(self):
        super().loop()
        self.update_state()
        if (self.state == "straight"):
            self.drive_to(self.goal_pose)
        else:
            self.shutdown_requested = True

if __name__ == "__main__":
    rospy.init_node("plat_straight_line2")
    rn = StraightLine()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
