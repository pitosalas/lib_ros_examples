#!/usr/bin/env python
import rospy
from rpsexamples.msg import Sensor
from std_msgs.msg import ColorRGBA
from math import pi, radians
from marker_array_utils import MarkerArrayUtils
from geometry_msgs.msg import Twist
import stroller_bt

def sensor_callback(msg):
    global target_bearing, target_distance
    target_bearing = msg.shortest_bearing
    target_distance = msg.shortest

def stop():
    global command_vel_pub
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    command_vel_pub.publish(twist)

def shutdown_hook():
    stop()
    print("\n*** Shutdown Requested ***")

rospy.init_node('strollercontrol')
sensor_sub = rospy.Subscriber('/sensor', Sensor,  sensor_callback)
command_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
target_distance = target_bearing = 0
rate = rospy.Rate(2)
rospy.on_shutdown(shutdown_hook)
twist = Twist()
sbc = stroller_bt.StrollerBt()
sbc.create_tree()


# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

while not (rospy.is_shutdown()):
    try:
        sbc.print_status()
        sbc.set_sensor_data(target_distance, target_bearing)
        sbc.tick_once()
        move, turn = sbc.get_desired_motion()
        if (move != twist.linear.x or turn != twist.angular.z):
            twist.linear.x = move
            twist.angular.z = turn
            command_vel_pub.publish(twist)
            #print("fwd: %.2f turn: %.2f " % (twist.linear.x, twist.angular.z))
        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        break
