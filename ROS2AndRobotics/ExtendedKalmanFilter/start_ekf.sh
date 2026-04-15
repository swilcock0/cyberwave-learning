#!/bin/bash
# Starts the robot_localization EKF node + MQTT bridge in ros2_ekf container.
# Fuses /odom and /imu/data_raw -> /odometry/filtered -> MQTT.
#
# First-time setup (or after editing Dockerfile):
#   /home/ws/ekf/start_ekf.sh          # builds the image automatically
#
# To rebuild image from scratch:
#   sudo docker rmi ros:humble-ekf && sudo docker rm -f ros2_ekf
#   /home/ws/ekf/start_ekf.sh
#
# MQTT env vars (pass via shell or prefix command):
#   CYBERWAVE_MQTT_HOST, MQTT_PORT, CYBERWAVE_TWIN_UUID, CYBERWAVE_TOKEN
#
set -e

# ── Argument Parsing ──────────────────────────────────────────────────────────
USE_MAG="false"
YAW_PROCESS_NOISE="0.06"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --use-mag) USE_MAG="true"; shift ;;
        --trust-imu) YAW_PROCESS_NOISE="0.01"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
done

IMAGE="ros:humble-ekf"
CONTAINER="ros2_ekf"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Build image if it doesn't exist ─────────────────────────────────────────
if ! sudo docker image inspect "$IMAGE" &>/dev/null; then
    echo "Building EKF image '${IMAGE}' from Dockerfile..."
    sudo docker build -t "$IMAGE" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"
fi

# ── Fetch Cyberwave Credentials (Fallback to defaults) ───────────────────────
CW_DRIVER=$(sudo docker ps --format '{{.Names}}' | grep "^cyberwave-driver-" | head -n 1 || true)
if [ -n "$CW_DRIVER" ]; then
    echo "Extracting credentials from running container: $CW_DRIVER..."
    LIVE_UUID=$(sudo docker exec "$CW_DRIVER" env | grep "^CYBERWAVE_TWIN_UUID=" | cut -d= -f2- || true)
    LIVE_TOKEN=$(sudo docker exec "$CW_DRIVER" env | grep "^CYBERWAVE_API_KEY=" | cut -d= -f2- || true)
    
    CYBERWAVE_TWIN_UUID="${CYBERWAVE_TWIN_UUID:-$LIVE_UUID}"
    CYBERWAVE_TOKEN="${CYBERWAVE_TOKEN:-$LIVE_TOKEN}"
fi

# ── Start container if not already running ───────────────────────────────────
if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Container '${CONTAINER}' already running."
elif sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Starting existing container '${CONTAINER}'..."
    sudo docker start "$CONTAINER"
else
    echo "Creating container '${CONTAINER}'..."
    sudo docker run -d \
        --name "$CONTAINER" \
        --network host \
        --ipc host \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v /home/ws/fastdds_udp.xml:/fastdds_udp.xml \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml \
        -e CYBERWAVE_MQTT_HOST="${CYBERWAVE_MQTT_HOST:-mqtt.cyberwave.com}" \
        -e MQTT_PORT="${MQTT_PORT:-8883}" \
        -e MQTT_TLS="${MQTT_TLS:-1}" \
        -e CYBERWAVE_TWIN_UUID="${CYBERWAVE_TWIN_UUID:-}" \
        -e CYBERWAVE_TOKEN="${CYBERWAVE_TOKEN:-}" \
        -e MQTT_USERNAME="${MQTT_USERNAME:-}" \
        -e MQTT_PASSWORD="${MQTT_PASSWORD:-}" \
        -e PUBLISH_RATE_HZ="${PUBLISH_RATE_HZ:-5.0}" \
        -e IMU_GYRO_VAR_X="${IMU_GYRO_VAR_X:-8.813078e-06}" \
        -e IMU_GYRO_VAR_Y="${IMU_GYRO_VAR_Y:-8.038429e-06}" \
        -e IMU_GYRO_VAR_Z="${IMU_GYRO_VAR_Z:-6.518847e-06}" \
        -e IMU_ACCEL_VAR_X="${IMU_ACCEL_VAR_X:-9.014193e-04}" \
        -e IMU_ACCEL_VAR_Y="${IMU_ACCEL_VAR_Y:-8.526505e-04}" \
        -e IMU_ACCEL_VAR_Z="${IMU_ACCEL_VAR_Z:-8.757489e-04}" \
        -e MAG_OFFSET_X="${MAG_OFFSET_X:-}" \
        -e MAG_OFFSET_Y="${MAG_OFFSET_Y:-}" \
        -e MAG_OFFSET_Z="${MAG_OFFSET_Z:-}" \
        -e ODOM_ROS_TOPIC="/odom" \
        "$IMAGE" \
        sleep infinity
fi

# ── Copy config & scripts into container ─────────────────────────────────────
sudo docker exec "$CONTAINER" mkdir -p /ekf/launch /ekf/config /ekf/scripts
sudo docker cp "${SCRIPT_DIR}/config/ekf_params.yaml"              "${CONTAINER}:/ekf/config/ekf_params.yaml"

# Dynamically update Yaw trust if requested
if [ "$YAW_PROCESS_NOISE" != "0.06" ]; then
    echo "Updating EKF params with high IMU trust (Yaw noise: $YAW_PROCESS_NOISE)..."
    sudo docker exec "$CONTAINER" sed -i "s/0.0,   0.0,   0.0,   0.0,   0.0,   0.06,/0.0,   0.0,   0.0,   0.0,   0.0,   $YAW_PROCESS_NOISE,/" /ekf/config/ekf_params.yaml
fi

sudo docker cp "${SCRIPT_DIR}/launch/ekf.launch.py"                "${CONTAINER}:/ekf/launch/ekf.launch.py"
sudo docker cp "${SCRIPT_DIR}/scripts/odometry_filtered_bridge.py" "${CONTAINER}:/ekf/scripts/odometry_filtered_bridge.py"
sudo docker cp "${SCRIPT_DIR}/scripts/odom_raw_bridge.py"          "${CONTAINER}:/ekf/scripts/odom_raw_bridge.py"
sudo docker cp "${SCRIPT_DIR}/scripts/imu_covariance_relay.py"     "${CONTAINER}:/ekf/scripts/imu_covariance_relay.py"
sudo docker cp "${SCRIPT_DIR}/scripts/calibrate_imu_noise.py"      "${CONTAINER}:/ekf/scripts/calibrate_imu_noise.py"
sudo docker cp "${SCRIPT_DIR}/scripts/calibrate_mag.py"            "${CONTAINER}:/ekf/scripts/calibrate_mag.py"

# ── Kill any previously running EKF/bridge processes ─────────────────────────
sudo docker exec "$CONTAINER" pkill -f ekf_node 2>/dev/null || true
sudo docker exec "$CONTAINER" pkill -f odometry_filtered_bridge 2>/dev/null || true
sudo docker exec "$CONTAINER" pkill -f odom_raw_bridge 2>/dev/null || true
sudo docker exec "$CONTAINER" pkill -f imu_covariance_relay 2>/dev/null || true
sudo docker exec "$CONTAINER" pkill -f imu_filter_madgwick 2>/dev/null || true
sleep 1

# ── Start IMU covariance relay in background ─────────────────────────────────
echo "Starting IMU covariance relay (/imu/data_raw -> /imu/data_raw_cov) ..."
sudo docker exec -d "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  python3 /ekf/scripts/imu_covariance_relay.py"
sleep 1

# ── Start IMU Madgwick filter in background ──────────────────────────────────
echo "Starting IMU Madgwick Filter (/imu/data_raw_cov + /imu/mag -> /imu/data) [use_mag: $USE_MAG]..."
sudo docker exec -d "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  ros2 run imu_filter_madgwick imu_filter_madgwick_node \
                      --ros-args -p use_mag:=$USE_MAG \
                      -p publish_tf:=false \
                      -p gain:=0.5 \
                      -p mag_bias_x:=\${MAG_OFFSET_X:--124.125} \
                      -p mag_bias_y:=\${MAG_OFFSET_Y:-81.975} \
                      -p mag_bias_z:=\${MAG_OFFSET_Z:-164.1} \
                      -r imu/data_raw:=/imu/data_raw_cov \
                      -r imu/mag:=/imu/mag \
                      -r imu/data:=/imu/data"
sleep 1

# ── Start Odom Raw Bridge in background ──────────────────────────────────────
echo "Starting Odom Raw Bridge (/odom/odom_raw -> /odom_unfiltered Twist) ..."
sudo docker exec -d "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  python3 /ekf/scripts/odom_raw_bridge.py"
sleep 1

# ── Start EKF node in background ─────────────────────────────────────────────
echo "Starting EKF node (odom_unfiltered + IMU → /odom) ..."
sudo docker exec -d "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  ros2 run robot_localization ekf_node \
                      --ros-args --params-file /ekf/config/ekf_params.yaml \
                      --remap odometry/filtered:=/odom"

# ── Start MQTT bridge in foreground (Ctrl-C stops both) ──────────────────────
cleanup() {
    echo ""
    echo "Stopping EKF node and MQTT bridge..."
    sudo docker exec "$CONTAINER" pkill -f ekf_node 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f odometry_filtered_bridge 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f odom_raw_bridge 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f imu_covariance_relay 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f imu_filter_madgwick 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting MQTT bridge (/odom → MQTT) ..."
echo "Press Ctrl-C to stop both."
echo

sudo docker exec -it "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  python3 /ekf/scripts/odometry_filtered_bridge.py"
