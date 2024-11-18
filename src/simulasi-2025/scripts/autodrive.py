#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2, sqrt, asin


class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('autodrive_node')

        # Target coordinates (adjust these as needed)
        self.target_x = self.declare_parameter('target_x', 5.0).value
        self.target_y = self.declare_parameter('target_y', 5.0).value

        # Publisher to send velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LaserScan data to detect obstacles
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscriber for Odometry data to get robot's current position
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer to send commands at regular intervals
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.is_obstacle_ahead = False

        # Robot's current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def scan_callback(self, msg):
        # Check if there is an obstacle in front of the robot
        min_distance = min(msg.ranges)
        self.is_obstacle_ahead = min_distance < 1.0  # Example threshold of 1 meter

    def odom_callback(self, msg):
        # Get the robot's position and orientation from Odometry data
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(quaternion)

    def euler_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angle (yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def timer_callback(self):
        twist = Twist()

        # Calculate the distance and angle to the target
        distance_to_target = sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        angle_to_target = atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        # Log current position and distance
        self.get_logger().info(f"Current Position: x={self.current_x}, y={self.current_y}, Distance to Target: {distance_to_target:.2f}")

        # If the robot is close enough to the target, stop
        if distance_to_target < 0.2:  # Adjust threshold if needed
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Target reached!")
        else:
            # If there is an obstacle ahead, rotate to avoid it
            if self.is_obstacle_ahead:
                twist.angular.z = 0.5  # Turn when obstacle is detected
                self.get_logger().info("Obstacle detected, turning!")
            else:
                # Move straight toward the target
                twist.linear.x = 0.5  # Move at a constant speed (adjust if needed)
                twist.angular.z = 0.0  # No rotation unless there's an obstacle

        # Publish velocity command
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
