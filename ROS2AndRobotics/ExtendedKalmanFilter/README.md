# EKF Sensor Fusion — odom + IMU + Magnetometer → MQTT

Fuses `/odom`, `/imu/data_raw`, and `/imu/mag` using the ROS 2 `robot_localization` Extended
Kalman Filter alongside the `imu_filter_madgwick` algorithm for precise 3D orientation. It then forwards the filtered odometry to an MQTT broker.

> **Note:** The `cyberwave-driver` naturally publishes the robot's map `pose` automatically to drive the 3D Digital Twin simulation from the ROS2 `odom` topic. This `odom` topic does not however get populated fully in the Cyberwave driver so we fill it by fusing data sources. The secondary work done here publishing the EKF's `/odom` stream natively to `odometry_filtered` over a sidecar MQTT payload is built simply for educational/demonstrative purposes on publishing ROS->MQTT from a container.

Runs in a dedicated Docker container (`ros2_ekf`), sharing the robot's DDS network via `--network host`.

---

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | Docker image definition (`ros:humble-ekf`) |
| `ekf_params.yaml` | EKF node configuration (sensor inputs, process noise) |
| `ekf_launch.py` | ROS 2 launch file (alternative to `ros2 run`) |
| `imu_covariance_relay.py` | Republishes `/imu/data_raw` → `/imu/data` with real covariances |
| `odometry_filtered_bridge.py` | Forwards `/odometry/filtered` → MQTT |
| `pose_refined_bridge.py` | Forwards `/pose_refined` → MQTT (separate use-case) |
| `calibrate_imu_noise.py` | Standstill measurement tool — run once to get IMU variances |
| `calibrate_mag.py` | Magnetometer calibration tool, run to get Mag bias |
| `start_ekf.sh` | Orchestration script — builds image, starts all processes |

---

## Quick Start

```bash
# First time only — builds the Docker image (takes ~2 min)
/home/ws/ekf/start_ekf.sh
```

### Advanced Usage (Command Line Arguments)

The `start_ekf.sh` script supports additional flags for tuning sensor fusion behavior:

| Flag | Effect |
|---|---|
| `--use-mag` | Enables the magnetometer. Not so useful **indoors** or in environments with high magnetic interference (e.g., near large metal structures or electrical panels) where the compass heading is unreliable. |
| `--trust-imu` | Increases the EKF's trust in the IMU orientation. Technically, it reduces the Yaw process noise covariance from `0.06` to `0.01` to make the heading less jittery but potentially more prone to drift over very long durations. |

**Example (Outdoor high-precision mode):**
```bash
/home/ws/ekf/start_ekf.sh --use-mag --trust-imu
```

On subsequent runs the image is reused and the container is started or restarted
automatically.

To **rebuild the image** after editing `Dockerfile`:

```bash
sudo docker rmi ros:humble-ekf
sudo docker rm -f ros2_ekf
/home/ws/ekf/start_ekf.sh
```

---

## MQTT Credentials

The `start_ekf.sh` script automatically extracts the active `CYBERWAVE_TWIN_UUID` and `CYBERWAVE_API_KEY` directly from the running `cyberwave-driver` Docker container's environment variables. 

If extraction fails or you need manual control, credentials can be overridden via environment variables before running the script:

```bash
CYBERWAVE_TWIN_UUID="my-uuid" \
CYBERWAVE_TOKEN="my-token" \
/home/ws/ekf/start_ekf.sh
```

**MQTT output topic:** `cyberwave/twin/<robot_id>/odometry_filtered`

**MQTT payload:**
```json
{
  "frame_id": "odom",
  "child_frame_id": "base_footprint",
  "stamp": { "sec": 1234, "nanosec": 567 },
  "position":         { "x": 1.2, "y": 0.3, "z": 0.0 },
  "orientation":      { "x": 0.0, "y": 0.0, "z": 0.12, "w": 0.99 },
  "linear_velocity":  { "x": 0.4, "y": 0.0, "z": 0.0 },
  "angular_velocity": { "x": 0.0, "y": 0.0, "z": 0.05 }
}
```

`position.x/y` and the yaw component of `orientation` are the robot's
displacement **relative to its starting position** (where the EKF initialised).

To extract yaw from the quaternion:

```python
import math
yaw = math.atan2(
    2.0 * (q['w'] * q['z'] + q['x'] * q['y']),
    1.0 - 2.0 * (q['y']**2 + q['z']**2)
)  # radians, 0 = forward at startup
```

---

## IMU Calibration

The IMU driver publishes all-zero covariances, which prevents the EKF from
weighting the IMU correctly. Run the calibration tool once to measure the real
sensor noise, then restart with those values.

### Step 1 — Measure noise (robot must be completely still, ~1 minute)

```bash
sudo docker exec -it ros2_ekf bash -c \
  "source /opt/ros/humble/setup.bash && python3 /ekf/scripts/calibrate_imu_noise.py --samples 500"
```

Sample output:
```
────────────────────────────────────────────────────────────
  IMU NOISE CALIBRATION RESULTS
────────────────────────────────────────────────────────────
  Angular velocity variance  (rad²/s²):
    x: 3.214600e-05
    y: 2.987300e-05
    z: 4.102100e-05

  Linear acceleration variance  (m²/s⁴):
    x: 8.734200e-03
    y: 9.215600e-03
    z: 1.124300e-02

  IMU_GYRO_VAR_X=3.214600e-05
  IMU_GYRO_VAR_Y=2.987300e-05
  IMU_GYRO_VAR_Z=4.102100e-05
  IMU_ACCEL_VAR_X=8.734200e-03
  IMU_ACCEL_VAR_Y=9.215600e-03
  IMU_ACCEL_VAR_Z=1.124300e-02
```

### Step 2 — Restart with measured values

```bash
IMU_GYRO_VAR_X=3.214600e-05 \
IMU_GYRO_VAR_Y=2.987300e-05 \
IMU_GYRO_VAR_Z=4.102100e-05 \
IMU_ACCEL_VAR_X=8.734200e-03 \
IMU_ACCEL_VAR_Y=9.215600e-03 \
IMU_ACCEL_VAR_Z=1.124300e-02 \
/home/ws/ekf/start_ekf.sh
```

These are passed as environment variables into the container and picked up by
`imu_covariance_relay.py`, which injects them into the IMU messages before the
EKF sees them.

Alternatively, you can skip exporting them every time by permanently hardcoding your measured variances as the defaults inside `start_ekf.sh` (e.g., `IMU_GYRO_VAR_X="\${IMU_GYRO_VAR_X:-3.214600e-05}"`).

---

## Magnetometer Calibration (Hard-Iron)

Magnetometers must be calibrated to offset magnetic hard-iron interference. 

### Step 1 — Measure Offsets

Execute `calibrate_mag.py` while slowly rotating the robot in 360-degree circles to get the X, Y, and Z offsets:

```bash
sudo docker exec -it ros2_ekf bash -c \
  "source /opt/ros/humble/setup.bash && python3 /ekf/scripts/calibrate_mag.py"
```

### Step 2 — Apply Offsets

These offsets are fed natively into the C++ `imu_filter_madgwick` node at launch to prevent timing jitters. You can pass them as environment variables:

```bash
MAG_OFFSET_X=-124.125 \
MAG_OFFSET_Y=81.975 \
MAG_OFFSET_Z=164.1 \
/home/ws/ekf/start_ekf.sh
```

Or you can permanently hardcode your results as the defaults inside `start_ekf.sh`:
```bash
-p mag_bias_x:=${MAG_OFFSET_X:--124.125} \
-p mag_bias_y:=${MAG_OFFSET_Y:-81.975} \
-p mag_bias_z:=${MAG_OFFSET_Z:-164.1} \
```

---

## Tuning the Process Noise (`ekf_params.yaml`)

`process_noise_covariance` controls how much the EKF trusts its own motion model
vs the sensor measurements between updates. The relevant diagonal entries are:

| Index | State | Effect of increasing |
|---|---|---|
| [0,0] | x | Position drifts faster; measurements dominate |
| [1,1] | y | Same for lateral position |
| [5,5] | yaw | Heading drifts faster |
| [6,6] | vx | Forward velocity trusts sensors more |
| [11,11] | vyaw | Yaw rate trusts sensors more |

**Pose drifts too fast** → decrease `[0,0]`, `[1,1]`, `[5,5]`  
**Pose is slow/sluggish to respond** → increase those same entries  
**Velocity estimates are jittery** → decrease `[6,6]`, `[11,11]`

After editing `ekf_params.yaml`, restart with `/home/ws/ekf/start_ekf.sh`
(no image rebuild needed — the file is copied into the container at startup).

---

## Diagnostics

```bash
# Check all three processes are running
sudo docker exec ros2_ekf pgrep -a "ekf_node\|imu_covariance\|odometry_filtered"

# Check EKF publish rate
dros2 topic hz /odometry/filtered

# Check EKF health
dros2 topic echo /diagnostics

# Live filtered odometry
dros2 topic echo /odometry/filtered

# Re-run IMU calibration
sudo docker exec -it ros2_ekf bash -c \
  "source /opt/ros/humble/setup.bash && python3 /ekf/scripts/calibrate_imu_noise.py"
```

---

## Architecture

```
/imu/data_raw  ──►  imu_covariance_relay  ──►  /imu/data  ──┐
                                                              │
/imu/data_raw  ──┐                                            │
                 ├─►  imu_filter_madgwick ──►  /imu/data      │
/imu/mag       ──┘    (Updates Orientation)                   ▼
                                                              │
/odom  ────────────────────────────────────────────►  ekf_filter_node
                                                              │
                                                              ▼
                                                   /odometry/filtered
                                                              │
                                                              ▼
                                                 odometry_filtered_bridge
                                                              │
                                                              ▼
                                                    MQTT broker
                             cyberwave/twin/<robot_id>/odometry_filtered
```

All three processes run inside the `ros2_ekf` container. The container uses
`--network host` and `--ipc host` so it shares the same DDS domain as the rest
of the robot software.
