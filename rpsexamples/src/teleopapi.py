import rospy
from geometry_msgs.msg import Twist

class TeleopApi():
    def __init__(self, start_node):
        if start_node:
            rospy.init_node('TeleopApi')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def cmd_vel(self, linear, angular, seconds):
        if not (-0.2 < linear < 0.2 and -0.8 < angular < 0.8):
            print "speed needs to be between -0.2 and 0.2"
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        rate = rospy.Rate(10)
        end_time = rospy.Time.now() + rospy.Duration(seconds)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        twist.linear.x = twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)


    def move(self, speed=0, seconds=0):
        self.cmd_vel(speed, 0, seconds)

    def turn(self, speed=0, seconds=0):
        self.cmd_vel(0, speed, seconds)

if __name__ == '__main__':
    toap = TeleopApi(True)
    toap.turn(0.2,1) # move at 0.1 m/s for 1 second.
