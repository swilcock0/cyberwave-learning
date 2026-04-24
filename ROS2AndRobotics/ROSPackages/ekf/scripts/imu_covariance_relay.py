"""Republishes /imu/data_raw as /imu/data with real measurement covariances.

The IMU driver on this robot sets all covariances to zero, which causes the
EKF to treat every measurement as perfectly precise. This relay node injects
measured noise variances so the EKF can weight the IMU correctly.

Set covariance values from calibrate_imu_noise.py output via env vars, or
accept the conservative defaults (typical low-cost MEMS IMU).

Env vars (all optional — defaults are conservative MEMS estimates):
    IMU_GYRO_VAR_X/Y/Z    angular velocity variance  (rad²/s²)
    IMU_ACCEL_VAR_X/Y/Z   linear acceleration variance (m²/s⁴)
    IMU_IN_TOPIC          source topic  (default: /imu/data_raw)
    IMU_OUT_TOPIC         output topic  (default: /imu/data)
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu


def _fenv(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, default))
    except ValueError:
        return default


# Conservative defaults for a typical low-cost MEMS IMU.
# Replace with output from calibrate_imu_noise.py.
_DEFAULT_GYRO_VAR  = 1.0e-4   # rad²/s²
_DEFAULT_ACCEL_VAR = 1.0e-2   # m²/s⁴


class ImuCovarianceRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_covariance_relay")

        gx = _fenv("IMU_GYRO_VAR_X",  _DEFAULT_GYRO_VAR)
        gy = _fenv("IMU_GYRO_VAR_Y",  _DEFAULT_GYRO_VAR)
        gz = _fenv("IMU_GYRO_VAR_Z",  _DEFAULT_GYRO_VAR)
        ax = _fenv("IMU_ACCEL_VAR_X", _DEFAULT_ACCEL_VAR)
        ay = _fenv("IMU_ACCEL_VAR_Y", _DEFAULT_ACCEL_VAR)
        az = _fenv("IMU_ACCEL_VAR_Z", _DEFAULT_ACCEL_VAR)

        # Row-major 3×3 covariance matrices (off-diagonals = 0)
        self._gyro_cov  = [gx, 0.0, 0.0,
                           0.0, gy, 0.0,
                           0.0, 0.0, gz]
        self._accel_cov = [ax, 0.0, 0.0,
                           0.0, ay, 0.0,
                           0.0, 0.0, az]
        # orientation_covariance = 0 allows the madgwick filter to populate it natively
        self._orient_cov = [0.0] * 9

        in_topic  = os.environ.get("IMU_IN_TOPIC",  "/imu/data_raw")
        out_topic = os.environ.get("IMU_OUT_TOPIC", "/imu/data_raw_cov")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Imu, out_topic, sensor_qos)
        self._sub = self.create_subscription(Imu, in_topic, self._relay, sensor_qos)

        self.get_logger().info(
            f"IMU relay: {in_topic} -> {out_topic}  "
            f"gyro_var=({gx:.2e},{gy:.2e},{gz:.2e})  "
            f"accel_var=({ax:.2e},{ay:.2e},{az:.2e})"
        )

    def _relay(self, msg: Imu) -> None:
        out = msg  # mutate in-place; no other node shares this message
        out.angular_velocity_covariance    = self._gyro_cov
        out.linear_acceleration_covariance = self._accel_cov
        out.orientation_covariance         = self._orient_cov
        self._pub.publish(out)

def main() -> None:
    rclpy.init()
    node = ImuCovarianceRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
