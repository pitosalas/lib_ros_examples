import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class SquareDriveNode(Node):
    def __init__(self):
        super().__init__('square_drive_node')
        self.declare_parameter('square_size', 1.0)
        self.declare_parameter('circle_radius', 1.0)

        self.square_size = self.get_parameter('square_size').value
        self.circle_radius = self.get_parameter('circle_radius').value

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10)
        )

        self.current_position = None
        self.current_orientation = None

        self.get_logger().info('SquareDriveNode has been started.')

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def drive_square(self):
        if self.current_position is None:
            self.get_logger().warn('Waiting for odometry data...')
            return

        self.get_logger().info(f'Driving in a square of size {self.square_size} meters.')
        for _ in range(4):
            self.drive_straight(self.square_size)
            self.turn_90_degrees()

    def drive_circle(self):
        if self.current_position is None:
            self.get_logger().warn('Waiting for odometry data...')
            return

        self.get_logger().info(f'Driving in a circle of radius {self.circle_radius} meters.')
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = twist.linear.x / self.circle_radius
        self.cmd_vel_publisher.publish(twist)

    def drive_straight(self, distance):
        # Placeholder for driving straight logic
        self.get_logger().info(f'Driving straight for {distance} meters.')

    def turn_90_degrees(self):
        # Placeholder for turning logic
        self.get_logger().info('Turning 90 degrees.')

    def run(self):
        try:
            while rclpy.ok():
                self.drive_square()
                self.drive_circle()
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.get_logger().info('Node interrupted by user, shutting down.')
        finally:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriveNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()