#!/usr/bin/env python
import rospy

mode = rospy.get_param("mode")
if mode == "sim":
  print("*** setting params for sim mode ***")
  # Set wallfollow parameters
  rospy.set_param("wallfollow/forward_speed", 0.1)
  rospy.set_param("wallfollow/desired_distance_from_wall", 0.5)
  rospy.set_param("wallfollow/speed_limit", 0.2)

  # set smartpid parameters
  rospy.set_param("smartpid/Kp", -0.44)
  rospy.set_param("smartpid/Td", -0.4)
  rospy.set_param("smartpid/Ti", 0)
  rospy.set_param("smartpid/dt",0.1)
elif mode == "real":
  print("*** setting params for real mode ***")

# Set wallfollow parameters
  rospy.set_param("wallfollow/forward_speed", 0.1)
  rospy.set_param("wallfollow/desired_distance_from_wall", 0.15)
  rospy.set_param("wallfollow/speed_limit", 0.1)

# set smartpid parameters
  rospy.set_param("smartpid/Kp", -0.7)
  rospy.set_param("smartpid/Ti", 0)
  rospy.set_param("smartpid/Td", -0.0)
  rospy.set_param("smartpid/dt",0.1)


