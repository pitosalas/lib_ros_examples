#!/usr/bin/env python3

import rospy
from basenode2 import BaseNode
from rpsexamples.msg import Sensor
import bru_utils as bu
from geometry_msgs.msg import Twist
from math import pi, radians, isnan

WEDGE_1_ANGLE = pi/3.0
FWD_RING1 = 1.0
REAR_RING1 = 1.0
RING2 = 2.0
RING0 = 0.5


FAST_LINEAR_SPEED = 0.3
SLOW_LINEAR_SPEED = 0.05
FAST_ANGULAR_SPEED = 0.2
SLOW_ANGULAR_SPEED = 0.15


class WanderPlatform(BaseNode):
    def initial_setup(self):
        super().initial_setup()
        self.set_hertz(2)

        self.state = "FAR"
        self.shortest = None
        self.smart_lidar_sub = rospy.Subscriber(
            "/sensor", Sensor, self.smart_lidar_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        while self.shortest is None:
            self.rate.sleep()

    def log(self, label: str):
        bu.info(f"{label}")

    def smart_lidar_cb(self, msg):
        self.shortest_bearing = bu.normalize_angle(
            radians(msg.shortest_bearing))
        self.shortest = msg.shortest

    def FAR(self):
        return self.shortest > RING2

    def forward_wedge(self):
        return abs(self.shortest_bearing) < WEDGE_1_ANGLE

    def MID(self):
        if self.forward_wedge() and RING2 > self.shortest > FWD_RING1:
            return True
        elif not self.forward_wedge() and RING2 > self.shortest > REAR_RING1:
            return True
        elif isnan(self.shortest_bearing):
            return True
        else:
            return False

    def CLOSE_FL(self):
        return 0 <= self.shortest_bearing < WEDGE_1_ANGLE and RING0 <= self.shortest < FWD_RING1

    def CLOSE_FR(self):
        return 0 >= self.shortest_bearing > -WEDGE_1_ANGLE and RING0 <= self.shortest < FWD_RING1

    def CLOSE_RR(self):
        return -pi <  self.shortest_bearing < -WEDGE_1_ANGLE and RING0 <= self.shortest < REAR_RING1

    def CLOSE_RL(self):
        return WEDGE_1_ANGLE < self.shortest_bearing < pi and RING0 <= self.shortest < REAR_RING1

    def DANGER(self):
        return RING0 > self.shortest

    def update_state(self, twist: Twist):
        self.log(
            f"STATE: {self.state}: short: {self.shortest:1.2} bearing: {self.shortest_bearing:1.2}")
        if self.CLOSE_RR():
            self.state = "CLOSE_RR"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = -SLOW_ANGULAR_SPEED
        elif self.CLOSE_RL():
            self.state = "CLOSE_RL"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = SLOW_ANGULAR_SPEED
        elif self.CLOSE_FL():
            self.state = "CLOSE_FL"
            twist.linear.x = -SLOW_LINEAR_SPEED
            twist.angular.z = -FAST_ANGULAR_SPEED
        elif self.CLOSE_FR():
            self.state = "CLOSE_FR"
            twist.linear.x = -SLOW_LINEAR_SPEED
            twist.angular.z = FAST_ANGULAR_SPEED
        elif self.MID():
            self.state = "MID"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = 0
        elif self.FAR():
            state = "FAR"
            twist.linear.x = FAST_LINEAR_SPEED
            twist.angular.z = 0
        elif self.DANGER():
            self.state = "DANGER"
            # self.shutdown_requested = True
        else:
            self.state = "error"
            # self.shutdown_requested = True

    def loop(self):
        super().loop()
        twist = Twist()
        self.update_state(twist)
        self.movement_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("wanderplatform")
    rn = WanderPlatform()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
