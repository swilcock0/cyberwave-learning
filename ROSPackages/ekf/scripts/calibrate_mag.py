"""Magnetometer basic hard-iron calibration tool.

Usage:
    dros2 run -- python3 calibrate_mag.py [--topic /imu/mag]

Rotate the robot slowly in multiple full 360-degree circles and figure-8s 
on the ground while this runs. Press Ctrl+C when done to see the offsets.
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

class MagCalibNode(Node):
    def __init__(self, topic: str):
        super().__init__("mag_calibration")
        self.get_logger().info(f"Listening to {topic}. Rotate the robot in 360° circles! Press Ctrl+C when done.")
        self._sub = self.create_subscription(MagneticField, topic, self._cb, 10)
        self.x_vals, self.y_vals, self.z_vals = [], [], []

    def _cb(self, msg: MagneticField):
        self.x_vals.append(msg.magnetic_field.x)
        self.y_vals.append(msg.magnetic_field.y)
        self.z_vals.append(msg.magnetic_field.z)
        
        if len(self.x_vals) % 50 == 0:
            self.get_logger().info(f"Collected {len(self.x_vals)} samples...")

    def report(self):
        if not self.x_vals:
            self.get_logger().warn("No data collected.")
            return

        # Hard-iron offset is the center of the min/max extents
        offset_x = (max(self.x_vals) + min(self.x_vals)) / 2.0
        offset_y = (max(self.y_vals) + min(self.y_vals)) / 2.0
        offset_z = (max(self.z_vals) + min(self.z_vals)) / 2.0

        print("\n" + "─" * 60)
        print("  MAGNETOMETER HARD-IRON CALIBRATION RESULTS")
        print("─" * 60)
        print(f"  Samples: {len(self.x_vals)}")
        print(f"  Max X: {max(self.x_vals):.6e} | Min X: {min(self.x_vals):.6e}")
        print(f"  Max Y: {max(self.y_vals):.6e} | Min Y: {min(self.y_vals):.6e}")
        print(f"  Max Z: {max(self.z_vals):.6e} | Min Z: {min(self.z_vals):.6e}")
        print("\n  Hard Iron Offsets (Subtract these from raw measurements):")
        print(f"    OFFSET_X = {offset_x:.6e}")
        print(f"    OFFSET_Y = {offset_y:.6e}")
        print(f"    OFFSET_Z = {offset_z:.6e}")
        print("─" * 60 + "\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/imu/mag")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MagCalibNode(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()