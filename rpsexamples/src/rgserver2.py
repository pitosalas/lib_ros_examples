#!/usr/bin/env python3
"""This module puts up a command prompt and gives the user control by commands of the robot"""

from rgparser2 import Parser
import rospy
from bru_utils import (
    turn_to_target,
    wait_for_simulator,
    calc_distance,
    normalize_angle,
    sigmoid,
)

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rpsexamples.msg import Mon
from math import degrees


class RoboGym:
    """Contains all the actions for the rg command set."""

    def __init__(self):
        rospy.init_node("robogym")
        wait_for_simulator()
        self.init_vars()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.mon_pub = rospy.Publisher("/monitor", Mon, queue_size=1)
        self.target = Pose2D(1, 1, 0)
        self.odom_pose = Pose2D(0, 0, 0)
        self.required_turn_angle = 0
        self.twist = None
        self.distance = None
        self.exit = False

    def odom_cb(self, msg: str):
        """ROS callback for Odometry Message"""
        oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(
            self.odom_pose.theta,
            self.odom_pose.x,
            self.odom_pose.y,
            self.target.x,
            self.target.y,
        )

    def init_vars(self):
        """Variables present at startup"""
        self.symbol_table = {
            "log": 1,  # 1 means verbose logging
            "max_ang": 0.75,  # maximum angular velcoicy (imposed by safe_publish)
            "max_lin": 0.5,  # maximum linear velocoty
            "target_ang": 0.75,  # angular velocity for normal speed
            "target_lin": 0.5,  # linear velocity for normal speed
            "r1": [[0, 0], [1, 1], [0, 0]],  # A sample navigational route
            "arrival_delta": 0.05,  # How close counts as the same point for goto command
        }

    def log(self, msg: str):
        """print out log messages if the log variable is set to 1"""
        if self.symbol_table["log"] == 1:
            print(msg)

    def safe_publish_cmd_vel(self):
        """Publish cmd_vel from self.twist, after applying maximum speed and rotation constraints.
        Note that this will probably move to a new safe_cmd_vel node"""
        safe = Twist()
        safe.linear.x = min(self.twist.linear.x, rg.symbol_table["max_lin"])
        safe.angular.z = min(self.twist.angular.z, rg.symbol_table["max_ang"])
        self.log(
            f"target: {self.target.x:2.2f}, {self.target.y:2.2f}"
            + f" curr: {self.odom_pose.x:2.2f} {self.odom_pose.y:2.2f})"
            + f", dist: {self.distance:2.2f}, turn: {degrees(normalize_angle(self.required_turn_angle)):2.2f}"
            + f", safe lin: {safe.linear.x:2.2f}, ang: {safe.angular.z:2.2f}"
            + f", req lin: {self.twist.linear.x:2.2f}, ang: {self.twist.angular.z:2.2f}"
        )
        self.cmd_vel_pub.publish(safe)

    def command_table(self):
        """Simply returns the command table"""
        return {
            "exit": {
                "args": [""],
                "nargs": 0,
                "description": "Exit Now",
                "handler": self.exit_now,
                "usage": "Invalid command. Usage: exit",
            },
            "stop": {
                "args": [],
                "nargs": 0,
                "description": "Stop the robot",
                "handler": self.stop,
            },
            "goto": {
                "args": ["<x>", "<y>"],
                "description": "Go to given odometry coordinate",
                "handler": self.goto,
                "usage": "Invalid command. Usage: goto <x> <y>",
                "nargs": 2,
            },
            "route": {
                "args": ["<route>"],
                "nargs": 1,
                "description": "Specific list of points. List can be zero or more points",
                "handler": self.route,
                "usage": "Invalid command. Usage: route <array of points>",
            },
        }

    def exit_now(self, args):
        """Execute "exit" command, to exit program"""
        print("Exiting now...")
        exit(0)

    def stop(self, args):
        """Execute the "stop" command, to Stop the robot by setting the cmd_vel to all zeros"""
        self.twist = Twist()
        self.distance = 0
        self.mon_pub.publish(Mon("state", "Stop immediately"))
        self.safe_publish_cmd_vel()
        print("Immediate stop")

    def goto(self, args):
        """Make the robot move to a specific x y"""
        print(args)
        self.twist = Twist()
        self.target = Pose2D(args[0], args[1], 0)
        rate = rospy.Rate(10)
        self.mon_pub.publish(
            Mon("state", f"Goto odometry x,y {self.target.x} {self.target.y}")
        )
        while not rospy.is_shutdown():
            self.distance = calc_distance(self.odom_pose, self.target)
            abs_required_turn = abs(normalize_angle(self.required_turn_angle))
            if self.distance > rg.symbol_table["arrival_delta"]:
                sig_turn = sigmoid(
                    abs_required_turn, 0.5, rg.symbol_table["target_ang"]
                )
                self.twist.linear.x = (
                    0
                    if abs_required_turn > 0.3
                    else sigmoid(self.distance, 0.5, rg.symbol_table["target_lin"])
                )
                self.twist.angular.z = (
                    sig_turn
                    if normalize_angle(self.required_turn_angle) > 0
                    else -sig_turn
                )
                self.safe_publish_cmd_vel()
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            rate.sleep()

    def is_valid_route(self, var):
        """type check that this is a valid route"""
        print(var)
        return isinstance(var, list) and all(
            isinstance(elem, list) and len(elem) == 2 for elem in var
        )

    def route(self, args):
        """Move the robot along a route defined by a series of points"""
        assert self.is_valid_route(args[0])
        for point in args[0]:
            self.goto(point)


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rg = RoboGym()
    cp = Parser(rg.symbol_table, rg.command_table())
    while True:
        try:
            cp.cli()
        except KeyboardInterrupt:
            rg.stop([])
            print("Stopping Robot. Type Exit to leave")
