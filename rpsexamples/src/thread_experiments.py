#!/usr/bin/env python
import rospy
import sys
import signal
import threading
import random
import time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

global_flag = 0
x = 0

def kill_time(power):
    dumb_array = [random.randint(1,1000) for x in range(10**power)].sort

def xodom_cb(msg):
    global global_flag
    local_flag = global_flag+1
    # kill_time(1)
    time.sleep(0.2)
    print(f'odom_cb: L/G Flag: {local_flag}/{global_flag}, thread: {threading.get_ident()}')
    global_flag = local_flag-1
    time.sleep(0.2)
    # kill_time(1)

def xclock_cb(msg):
    global global_flag
    local_flag = global_flag+1
    time.sleep(1.0)
    # kill_time(6)
    print(f'clockcb: L/G Flag: {local_flag}/{global_flag}, thread: {threading.get_ident()}')
    global_flag = local_flag-1
    time.sleep(1.0)
    # kill_time(1)

def clock_cb(msg):
    global x
    for i in range(1000000):
        x += 1
    for i in range(1000000):
        x -= 1    
    print(f'clockcb: x: {x}, thread: {threading.get_ident()}')
  

def odom_cb(msg):
    global x
    for i in range(1000000):
        x += 1
    for i in range(1000000):
        x -= 1      
    print(f'odom_cb: x: {x}, thread: {threading.get_ident()}')

# Initialize this program as a node
rospy.init_node('mini_odom_demo')
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
#clock_sub = rospy.Subscriber('/clock', Clock, clock_cb)

rospy.spin()
