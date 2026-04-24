#!/bin/bash
# Starts the Nav2 stack in ros2_nav2 container.
# Uses /odom (from EKF) and /scan (from Lidar) to provide navigation.
#
# First-time setup (or after editing Dockerfile):
#   /home/ws/nav2/start_nav2.sh          # builds the image automatically
#
# To rebuild image from scratch:
#   sudo docker rmi ros:humble-nav2 && sudo docker rm -f ros2_nav2
#   /home/ws/nav2/start_nav2.sh
#
set -e

IMAGE="ros:humble-nav2"
CONTAINER="ros2_nav2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Build image if it doesn't exist ─────────────────────────────────────────
if ! sudo docker image inspect "$IMAGE" &>/dev/null; then
    echo "Building Nav2 image '${IMAGE}' from Dockerfile..."
    sudo docker build -t "$IMAGE" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"
fi

# ── Start container if not already running ───────────────────────────────────
if sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Container '${CONTAINER}' already running."
elif sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Starting existing container '${CONTAINER}'..."
    sudo docker start "$CONTAINER"
else
    echo "Creating container '${CONTAINER}'..."
    sudo docker rm -f "$CONTAINER" 2>/dev/null || true
    sudo docker run -d \
        --name "$CONTAINER" \
        --network host \
        --ipc host \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -v /home/ws/fastdds_udp.xml:/fastdds_udp.xml \
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml \
        "$IMAGE" \
        sleep infinity
fi

# ── Copy config & launch files into container ────────────────────────────────
sudo docker exec "$CONTAINER" mkdir -p /nav2/launch /nav2/config
sudo docker cp "${SCRIPT_DIR}/config/nav2_params.yaml"  "${CONTAINER}:/nav2/config/nav2_params.yaml"
sudo docker cp "${SCRIPT_DIR}/config/blank_map.yaml"    "${CONTAINER}:/nav2/config/blank_map.yaml"
sudo docker cp "${SCRIPT_DIR}/config/blank_map.pgm"     "${CONTAINER}:/nav2/config/blank_map.pgm"
# sudo docker cp "${SCRIPT_DIR}/launch/nav2.launch.py"   "${CONTAINER}:/nav2/launch/nav2.launch.py" # (If using a custom launch file)

# ── Kill any previously running Nav2 processes ───────────────────────────────
sudo docker exec "$CONTAINER" pkill -f nav2 2>/dev/null || true
sleep 1

# ── Start Nav2 stack (using standard navigation launch) ──────────────────────
echo "Starting Nav2 Navigation Stack (using EKF's /odom) ..."

echo "Press Ctrl-C to stop."
echo

cleanup() {
    echo ""
    echo "Stopping Nav2 stack..."
    sudo docker exec "$CONTAINER" pkill -f static_transform_publisher 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f nav2 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f amcl 2>/dev/null || true
    sudo docker exec "$CONTAINER" pkill -f map_server 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Publish a static map→odom identity transform so the global costmap can
# activate even when AMCL has not yet received a /scan.  AMCL will override
# this transform once laser scans are available.
sudo docker exec "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  nohup ros2 run tf2_ros static_transform_publisher \
                      --frame-id base_link --child-frame-id lidar_optical_link \
                      --x 0.0398145505519817 --y 0 --z 0.04 --roll 0 --pitch 0 --yaw 1.57079632679 \
                      >/tmp/static_tf.log 2>&1 &"
                      
sudo docker exec -it "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  ros2 launch nav2_bringup bringup_launch.py \
                      use_sim_time:=false \
                      map:=/nav2/config/blank_map.yaml \
                      params_file:=/nav2/config/nav2_params.yaml"
