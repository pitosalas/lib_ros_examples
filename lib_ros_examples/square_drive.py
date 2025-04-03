import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion
import lib_ros_examples.bru_utils
import math


class SquareDriveNode(Node):
    def __init__(self):
        super().__init__("square_drive_node")
        self.declare_parameter("square_size", 0.5)
        self.declare_parameter("circle_radius", 1.0)

        self.square_size = self.get_parameter("square_size").value
        self.circle_radius = self.get_parameter("circle_radius").value

        self.subscription = self.create_subscription(
            Odometry,
            "/odom",  # Replace with your actual topic name if different
            self.odom_callback,
            QoSProfile(depth=10),
        )
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.counter = 0

        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/cmd_vel", QoSProfile(depth=10)
        )
        self.get_logger().info("SquareDriveNode has been started.")

    def odom_callback(self, msg):
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Get orientation and convert to yaw
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.current_yaw = euler_from_quaternion(quaternion)

    def publish_velocity(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher.publish(msg)
        self.counter += 1
        if self.counter > 10:
            self.counter = 0
            self.get_logger().info(
                f"st: {self.state} Pos: ({self.current_x:.2f}, {self.current_y:.2f}), Yaw: {self.current_yaw:.2f} rad"
            )

    def save_starting_odom(self):
        # Wait until the first odometry message is received
        self.get_logger().info("Waiting for initial odometry data...")
        while rclpy.ok() and (
            self.current_x == 0.0 and self.current_y == 0.0 and self.current_yaw == 0.0
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        # Record the current odometry as the starting position
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.get_logger().info(
            f"Starting odometry recorded: x={self.start_x:.2f}, y={self.start_y:.2f}, yaw={self.start_yaw:.2f} rad"
        )

    def calculate_cmd_vel_straight(self):
        distance = math.sqrt(
            (self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2
        )
        if distance < self.square_size:
            self.linear_x = 0.5
            self.angular = 0.0
        else:
            self.linear_x = 0.0
            self.angular = 0.0
            self.state = "turn"
            self.start_yaw = self.current_yaw
            self.get_logger().info("Reached square side, turning.")

    def calculate_cmd_vel_turn(self):
        # angle = math.atan2(self.current_y - self.start_y, self.current_x - self.start_x)
        goal_angle = self.start_yaw + math.pi/2.0
        angle_diff = lib_ros_examples.bru_utils.normalize_angle(self.current_yaw - goal_angle)
        print(
            f"yaw start={math.degrees(self.start_yaw):.2f}, "
            f"curr={math.degrees(self.current_yaw):.2f}, "
            f"diff={math.degrees(angle_diff):.2f}, "
            f"goal={math.degrees(goal_angle):.2f}, "
        )
        if abs(angle_diff) > 0.2:
            self.linear_x = 0.0
            self.angular = 0.8
        else:
            self.linear_x = 0.0
            self.angular = 0.0
            self.state = "straight"
            self.save_starting_odom()
            self.get_logger().info("Turn completed, moving to next side.")

    def run(self):
        self.state = "straight"
        self.save_starting_odom()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.state == "straight":
                    self.calculate_cmd_vel_straight()
                elif self.state == "turn":
                    self.calculate_cmd_vel_turn()
                self.publish_velocity(self.linear_x, self.angular)
        except KeyboardInterrupt:
            self.get_logger().info("Node interrupted by user, shutting down.")
        finally:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriveNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
