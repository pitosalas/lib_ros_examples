#!/usr/bin/env pythongit sta3

"""
Monitor the state of the robot. 
Note that this node requires that the sound_play node is also running. 
rosrun sound_pay sound_play
"""

import rospy
from sound_play.libsoundplay import SoundClient
from rpsexamples.msg import Mon
from nav_msgs.msg import Odometry

class Monitor:
    """Eventually will be the Robot's brain stem keeping track that things
    are going ok. For now it just reports state changes"""
    def __init__(self):
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0
        self.pose = None
        self.sub_monitor = rospy.Subscriber('monitor', Mon, self.monitor_callback)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def monitor_callback(self, msg):
        """Callback when requests are made to monitor"""
        self.say(msg.argument)

    def odom_callback(self, msg):
        """ROS calback on new Odom reading"""
        self.pose = msg.pose

    def say(self, message):
        """Text to speech the string over the speaker"""
        print(f"Status monitor: {message}")
        self.soundhandle.say(message, self.voice, self.volume)

if __name__ == '__main__':
    rospy.init_node("monitor")
    mon = Monitor()
    rate = rospy.Rate(1/60.0)
    while not rospy.is_shutdown():
        mon.say("Robot Status OK")
        rate.sleep()
    mon.say("Control Program exiting")
    


