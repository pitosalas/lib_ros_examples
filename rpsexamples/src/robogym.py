#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from prrexamples.msg import Robogym

# robogym.py and rg.py work together. rg.py accepts commands from the keyboard and generates Robogym messages which are very
# very simple parsing of the commands. robogym.py subscribes to these messages and executes them. Neither needs to run `onboard`.
#
# The purpose is to help callibrate a robot by sending commands that can be performed by the robot and then measured because
# they are accurate.
#
# The commands are:
#
# reset = stops the robot motion
# move <linear-speed> <angular-speed> <rate> 
#   which will send a constant stream of cmd_vels of the stated linear and angular at the stated rate
# time <linear-speed> <angular-speed> <rate> <seconds>
#   which will send a stream of cmd_vels of the stated linear and angular at the stated rate for specified seconds
# count <linear-speed> <angular-speed> <rate> <count>
#   which will send a stream of exactly <count> cmd_vels of the stated linear and angular at the stated rate
# distance <linear-speed> <angular-speed> <rate> <distance>
#   which will send a stream of <count> cmd_vels of the stated linear and angular at the stated rate until the <distance> has been covered.
#

class RoboGym:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_cli = rospy.Subscriber('cli', Robogym, self.cli_callback)
        self.reset()

    def reset(self):
        self.speed_trans = 0
        self.mode = "reset"
        self.countdown = 0
        self.speed_ang = 0
        self.speed_linear = 0
        self.pub_rate = 0.5
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_cli = rospy.Subscriber('cli', Robogym, self.cli_callback)
        self.last_twist = Twist()
        self.once_flag = False

# Commands:
# move lin, rot, rate e.g. move 0.5 0 5
# count lin, rot, rate, count
# reset
# distance lin, rot, rate, meters (NYI)
# time lin, rot, rate, seconds (NYI)

    def check_params(self, msg):
        if msg.command != "reset" and msg.rate <= 0:
            return False
        if msg.command == "count" and msg.lim <= 0:
            return False
        return True

    def cli_callback(self, msg):
        if not self.check_params(msg):
            print("!! Bad command")
            return
        self.last_twist = Twist()
        self.last_twist.linear.x = msg.lin
        self.last_twist.angular.z = msg.ang
        self.pub_rate = msg.rate
        if (msg.command == "reset"):
            self.mode = "reset"
        if (msg.command == "move"):
            self.mode = "move"
        elif msg.command == "count":
            self.mode = "count"
            self.countdown = msg.lim
        elif msg.command == "time":
            self.mode = "time"
            now = rospy.get_rostime()
            self.start_time = now.secs
            self.end_time = msg.lim + now.secs

    def step(self):
        if self.pub_rate == 0:
            print("step with pub rate 0 is a bug")
        else:
            self.pub_cmd_vel.publish(self.last_twist)
            rospy.sleep(1.0/self.pub_rate)
            self.once_flag = False

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == "count":
                self.countdown -= 1
                if self.countdown == 0:
                    self.mode = "reset"
                    self.once_flag = True
                else:
                    print(f"counter {self.countdown}")
                    self.step()
            elif self.mode == "time":
                togo = self.end_time - rospy.get_rostime().secs
                if togo <= 0:
                    self.mode = "reset"
                    self.once_flag = True
                else:
                    print(f"timer {togo}")
                    self.step()
            elif self.mode == "reset":
                if self.once_flag:
                    t = Twist()
                    self.pub_cmd_vel.publish(t)
                    rospy.sleep(1)
                    self.once_flag = False
            elif self.mode == "move":
                self.step()


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    try:
        RoboGym().run()
    except rospy.ROSInterruptException:
        pass
