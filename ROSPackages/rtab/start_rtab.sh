#!/bin/bash
# Starts the RTAB-Map SLAM in ros2_rtab container.
#
# First-time setup (or after editing Dockerfile):
#   /home/ws/dev_sam/rtab/start_rtab.sh          # builds the image automatically
#
set -e

IMAGE="ros:humble-rtab"
CONTAINER="ros2_rtab"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Build image if it doesn't exist ─────────────────────────────────────────
if ! sudo docker image inspect "$IMAGE" &>/dev/null; then
    echo "Building RTAB-Map image '${IMAGE}' from Dockerfile..."
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
sudo docker exec "$CONTAINER" mkdir -p /rtab/launch /rtab/config
sudo docker cp "${SCRIPT_DIR}/config/rtab_params.yaml"  "${CONTAINER}:/rtab/config/rtab_params.yaml"
# sudo docker cp "${SCRIPT_DIR}/config/nav2_params.yaml"  "${CONTAINER}:/rtab/config/nav2_params.yaml"
# sudo docker cp "${SCRIPT_DIR}/config/blank_map.pgm"     "${CONTAINER}:/rtab/config/blank_map.pgm"

# ── Kill any previously running RTAB processes ──────────────────────────────
sudo docker exec "$CONTAINER" pkill -f rtabmap 2>/dev/null || true
sleep 1

# ── Start RTAB-Map SLAM ─────────────────────────────────────────────────────
echo "Starting RTAB-Map SLAM..."

echo "Press Ctrl-C to stop."
echo

cleanup() {
    echo ""
    echo "Stopping RTAB-Map..."
    sudo docker exec "$CONTAINER" pkill -f rtabmap 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sudo docker exec -it "$CONTAINER" \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  ros2 run rtabmap_slam rtabmap --delete_db_on_start \
                      --ros-args \
                      --params-file /rtab/config/rtab_params.yaml \
                      --remap odom:=/odom \
                      --remap scan:=/scan \
                      --remap map:=/map"    