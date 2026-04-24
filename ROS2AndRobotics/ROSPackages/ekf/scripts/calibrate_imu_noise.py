"""IMU noise calibration tool.

Place the robot completely still, then run:
    dros2 run -- python3 /ekf/calibrate_imu_noise.py --samples 5000 --topic /imu/data_raw

It collects samples from /imu/data_raw and prints the variance of each axis.
Paste the output values into imu_covariance_relay.py (or the env vars in
start_ekf.sh) to give the EKF realistic measurement noise covariances.

Usage:
    python3 calibrate_imu_noise.py [--samples 5000] [--topic /imu/data_raw]
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCalibNode(Node):
    def __init__(self, topic: str, n_samples: int) -> None:
        super().__init__("imu_calibration")
        self._n = n_samples
        self._gyro: list[tuple[float, float, float]] = []
        self._accel: list[tuple[float, float, float]] = []
        self._sub = self.create_subscription(Imu, topic, self._cb, 100)
        self.get_logger().info(
            f"Collecting {n_samples} samples from {topic} — keep the robot STILL ..."
        )

    def _cb(self, msg: Imu) -> None:
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self._gyro.append((g.x, g.y, g.z))
        self._accel.append((a.x, a.y, a.z))

        collected = len(self._gyro)
        if collected % 50 == 0:
            self.get_logger().info(f"  {collected}/{self._n} samples ...")
        if collected >= self._n:
            self._report()
            rclpy.shutdown()

    @staticmethod
    def _variance(values: list[float]) -> float:
        n = len(values)
        mean = sum(values) / n
        return sum((v - mean) ** 2 for v in values) / (n - 1)

    def _report(self) -> None:
        gx_var = self._variance([s[0] for s in self._gyro])
        gy_var = self._variance([s[1] for s in self._gyro])
        gz_var = self._variance([s[2] for s in self._gyro])
        ax_var = self._variance([s[0] for s in self._accel])
        ay_var = self._variance([s[1] for s in self._accel])
        az_var = self._variance([s[2] for s in self._accel])

        sep = "─" * 60
        print(f"\n{sep}")
        print("  IMU NOISE CALIBRATION RESULTS")
        print(sep)
        print(f"  Samples collected : {len(self._gyro)}")
        print()
        print("  Angular velocity variance  (rad²/s²):")
        print(f"    x: {gx_var:.6e}")
        print(f"    y: {gy_var:.6e}")
        print(f"    z: {gz_var:.6e}")
        print()
        print("  Linear acceleration variance  (m²/s⁴):")
        print(f"    x: {ax_var:.6e}")
        print(f"    y: {ay_var:.6e}")
        print(f"    z: {az_var:.6e}")
        print(sep)
        print()
        print("  Copy these values into start_ekf.sh or imu_covariance_relay.py:")
        print()
        print(f"    IMU_GYRO_VAR_X={gx_var:.6e}")
        print(f"    IMU_GYRO_VAR_Y={gy_var:.6e}")
        print(f"    IMU_GYRO_VAR_Z={gz_var:.6e}")
        print(f"    IMU_ACCEL_VAR_X={ax_var:.6e}")
        print(f"    IMU_ACCEL_VAR_Y={ay_var:.6e}")
        print(f"    IMU_ACCEL_VAR_Z={az_var:.6e}")
        print(sep)
        print()
        print("  NEXT STEPS:")
        print("  1. Set env vars above in start_ekf.sh (docker run -e ...) or")
        print("     hardcode into imu_covariance_relay.py defaults.")
        print("  2. Restart: /home/ws/ekf/start_ekf.sh")
        print(sep + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--topic", default="/imu/data_raw")
    # rclpy args come after --ros-args; strip them before argparse
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ImuCalibNode(args.topic, args.samples)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
