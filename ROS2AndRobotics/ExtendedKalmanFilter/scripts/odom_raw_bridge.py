#!/usr/bin/env python3
"""Converts /odom/odom_raw (Float32MultiArray) into a standard /odom (nav_msgs/Odometry) message.
It assumes the input array contains [left_wheel_speed, right_wheel_speed] in m/s 
and translates them into linear and angular velocity (Twist).
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class OdomRawBridge(Node):
    def __init__(self):
        super().__init__('odom_raw_bridge')
        self.sub = self.create_subscription(Float32MultiArray, '/odom/odom_raw', self.raw_cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom_unfiltered', 10)
        
        # Estimate of the distance between left and right wheels in meters.
        # You can pass a more precise value via environment variables.
        self.track_width = float(os.environ.get('TRACK_WIDTH', '0.175'))
        self.wheel_radius = float(os.environ.get('WHEEL_RADIUS', '0.04'))
        # If the raw data is in radians instead of linear meters, set this to '1'
        self.is_radians = os.environ.get('IS_RADIANS', '0') == '1'

        self.last_left = None
        self.last_right = None
        self.last_time = None

        self.get_logger().info(f"Odom Raw Bridge starting. Assuming track_width: {self.track_width}m")

    def raw_cb(self, msg):
        if len(msg.data) < 2:
            return

        left_dist = msg.data[0]
        right_dist = msg.data[1]

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_left = left_dist
            self.last_right = right_dist
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        d_left = left_dist - self.last_left
        d_right = right_dist - self.last_right

        self.last_left = left_dist
        self.last_right = right_dist
        self.last_time = current_time

        left_speed = d_left / dt
        right_speed = d_right / dt

        # If the input is in radians, convert to linear distance (m/s)
        if self.is_radians:
            left_speed *= self.wheel_radius
            right_speed *= self.wheel_radius

        # Standard differential drive kinematics for Twist 
        linear_x = (left_speed + right_speed) / 2.0
        angular_z = (right_speed - left_speed) / self.track_width

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Populate Twist (Velocity) constraints. 
        # (We skip position integration here, and let the EKF_node handle the math)
        odom.twist.twist.linear.x = float(linear_x)
        odom.twist.twist.angular.z = float(angular_z)

        # Baseline covariance for velocity data
        odom.twist.covariance[0] = 1e-3   # linear.x
        odom.twist.covariance[35] = 1e-3  # angular.z

        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRawBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()