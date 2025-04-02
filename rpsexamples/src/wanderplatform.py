#!/usr/bin/env python3

import rospy
from basenode2 import BaseNode
from rpsexamples.msg import Sensor
import bru_utils as bu
from geometry_msgs.msg import Twist
from math import pi, radians

FORWARD_LINEAR_SPEED = 0.2
TURN_LINEAR_SPEED = -0.02
TURN_ANGULAR_SPEED = 0.15


class WanderPlatform(BaseNode):
    def initial_setup(self):
        super().initial_setup()
        self.state = "forward"
        self.shortest = None
        self.smart_lidar_sub = rospy.Subscriber("/sensor", Sensor, self.smart_lidar_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        while self.shortest is None:
            self.rate.sleep()

    def log(self, label: str):
        bu.info(f"{label}")

    def smart_lidar_cb(self, msg):
        self.shortest_bearing = bu.normalize_angle(radians(msg.shortest_bearing))
        self.shortest = msg.shortest

    def forward(self):
        twist = Twist()
        twist.linear.x = FORWARD_LINEAR_SPEED
        twist.angular.z = 0.0
        self.movement_pub.publish(twist)
    
    def turn(self):
        twist = Twist()
        twist.linear.x = TURN_LINEAR_SPEED
        twist.angular.z = TURN_ANGULAR_SPEED
        self.movement_pub.publish(twist)
    
    def update_state(self):
        self.log(f"STATE: {self.state}: short: {self.shortest:1.2} bearing: {self.shortest_bearing:1.2}")
    # no obstacles nearby
        if self.shortest > 0.6:
            self.state = "forward"
    # obstacles "behind"
        elif self.shortest < 0.4 and abs(self.shortest_bearing) > (pi / 2):
            self.state = "forward"
    # obstacles "ahead"
        elif self.shortest < 0.4 and abs(self.shortest_bearing) <= (pi / 2):
            self.state = "turn"
    # obstacles behind and to the left
        elif self.shortest < 0.4 and pi/2.0 < self.shortest_bearing < pi:
            self.state = "slow forward right"
    # obstacles behind and to the right
        elif self.shortest < 0.4 and -pi < self.shortest_bearing < - pi / 2.0:
            self.state = "slow forward left"

    # obstacle ahead and to the left
    # onstacle ahead and to the right
    
    # case 4: "impossible"
        else:
            self.state = "error"

    def loop(self):
        super().loop()
        self.update_state()
        if self.state == "forward":
            self.forward()
        elif self.state == "turn":
            self.turn()
        else:
            self.shutdown_requested = True


if __name__ == "__main__":
    rospy.init_node("wanderplatform")
    rn = WanderPlatform()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
