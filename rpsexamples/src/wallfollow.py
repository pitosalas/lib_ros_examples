#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi, isnan
import sys
from rpsexamples.msg import Sensor
import smartpid

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.hz = 20
        self.distance_from_left_wall = 0
        self.shutdown_requested = False

        #Setting the PID
        self.controller=smartpid.PID(-1,-1.2, 0.00001,0.1)
        self.update_params()

        #Creating cmd_pub Publisher that will publish a Twist msg to cmd_vel
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
        
        self.msg = Twist()
        self.msg.linear.x = self.forward_speed

        #Publish the msg
        self.cmd_pub.publish(self, self.msg)

        # Listen to smartsensor messages        
        self.sensor_sub = rospy.Subscriber('/sensor', Sensor,  self.sensor_callback)

    def sensor_callback(self, msg):
        if isnan(msg.left):
            return
        self.update_params()
        self.distance_from_left_wall = msg.left
        print("left: %.2f dfl: %.2f ddfw: %.2f" % (msg.left, self.distance_from_left_wall, self.desired_distance_from_wall))

       	#Calculating the cross track error
       	cross_track_error = self.desired_distance_from_wall - self.distance_from_left_wall
        
       	#Updating the controller and publishing the cross track error
       	self.controller.update_control(cross_track_error)

       	cmd = Twist()
        cmd.linear.x = min(self.forward_speed, self.speed_limit)

  		#Getting the new cmd.angular.z
       	cmd.angular.z = self.controller.get_control()
       	
       	#Publishing the cmd.linear.x
        print "lin: %.2f ang: %.2f" % (cmd.linear.x, cmd.angular.z)
       	self.cmd_pub.publish(cmd)
   
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown() and not self.shutdown_requested:
            rate.sleep()
        self.stop()
        
    # Update latest params
    def update_params(self):
        self.forward_speed = rospy.get_param("wallfollow/forward_speed")
        self.desired_distance_from_wall = rospy.get_param("wallfollow/desired_distance_from_wall")
        self.speed_limit = rospy.get_param("wallfollow/speed_limit")

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def shutdown_hook(self):
        self.shutdown_requested = True
        print("\n**** Shutdown Requested ****")
        self.stop()


if __name__ == '__main__':
    wfh = WallFollower()
    rospy.on_shutdown(wfh.shutdown_hook)
    wfh.run()

