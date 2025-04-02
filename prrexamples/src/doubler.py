#!/usr/bin/env python

#  rosrun prrexamples doubler
#  rostopic echo doubled
#  rostopic pub /number std_msgs/Int32 "data: 12"  

import rospy
from std_msgs.msg import Int32

rospy.init_node('doubler')

def callback(msg):
    doubled = Int32()
    doubled.data = msg.data * 2
    pub.publish(doubled)

sub = rospy.Subscriber('number', Int32, callback)
pub = rospy.Publisher('doubled', Int32, queue_size=10)
rospy.spin()