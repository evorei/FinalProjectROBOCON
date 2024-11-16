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

        # Mendapatkan parameter target (X, Y) dari launch file
        self.target_x = self.declare_parameter('target_x', 5.0).value
        self.target_y = self.declare_parameter('target_y', 5.0).value

        # Parameter PID
        self.kp_linear = 0.5  # Gain P untuk kecepatan linear
        self.ki_linear = 0.01  # Gain I untuk kecepatan linear
        self.kd_linear = 0.1  # Gain D untuk kecepatan linear

        self.kp_angular = 1.0  # Gain P untuk rotasi
        self.ki_angular = 0.1  # Gain I untuk rotasi
        self.kd_angular = 0.5  # Gain D untuk rotasi

        # Publisher untuk mengirim perintah kecepatan
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber untuk data LaserScan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscriber untuk data Odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer untuk mengirim perintah secara berkala
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.is_obstacle_ahead = False

        # Posisi robot saat ini
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Error PID
        self.error_integral_linear = 0.0
        self.error_previous_linear = 0.0

        self.error_integral_angular = 0.0
        self.error_previous_angular = 0.0

    def scan_callback(self, msg):
        # Periksa jarak terdekat pada data laser
        min_distance = min(msg.ranges)
        self.is_obstacle_ahead = min_distance < 1.0  # Contoh threshold 1 meter

    def odom_callback(self, msg):
        # Mendapatkan posisi dan orientasi robot dari data odometri
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Menghitung orientasi robot dari quaternion menjadi sudut yaw (z-axis)
        quaternion = msg.pose.pose.orientation
        _, _, self.current_theta = self.euler_from_quaternion(quaternion)

    def euler_from_quaternion(self, quaternion):
        # Mengubah quaternion menjadi sudut Euler (yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Roll, pitch, yaw
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

    def pid_control_linear(self, distance):
        # Proportional, Integral, Derivative untuk linear velocity
        error = distance
        self.error_integral_linear += error
        error_derivative = error - self.error_previous_linear

        # PID formula
        pid_output = (self.kp_linear * error) + (self.ki_linear * self.error_integral_linear) + (self.kd_linear * error_derivative)
        
        # Simpan error sebelumnya untuk digunakan pada iterasi berikutnya
        self.error_previous_linear = error

        return pid_output

    def pid_control_angular(self, angle_diff):
        # Proportional, Integral, Derivative untuk angular velocity
        error = angle_diff
        self.error_integral_angular += error
        error_derivative = error - self.error_previous_angular

        # PID formula
        pid_output = (self.kp_angular * error) + (self.ki_angular * self.error_integral_angular) + (self.kd_angular * error_derivative)
        
        # Simpan error sebelumnya untuk digunakan pada iterasi berikutnya
        self.error_previous_angular = error

        return pid_output

    def timer_callback(self):
        twist = Twist()

        # Hitung jarak dan sudut menuju target
        distance_to_target = sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        angle_to_target = atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        # PID kontrol untuk linear dan angular velocity
        linear_velocity = self.pid_control_linear(distance_to_target)
        angular_velocity = self.pid_control_angular(angle_to_target - self.current_theta)

        # Jika robot sudah cukup dekat dengan target, berhenti
        if distance_to_target < 0.2:  # Threshold jarak ke target
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Jika ada rintangan, putar
            if self.is_obstacle_ahead:
                twist.angular.z = 0.5  # Putar saat ada rintangan
            else:
                # Set kecepatan berdasarkan PID controller
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity

        # Kirim perintah ke robot
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

