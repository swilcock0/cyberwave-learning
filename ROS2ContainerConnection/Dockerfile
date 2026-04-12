FROM ros:humble-ros-core

# No extra DDS explicitly installed - uses default FastRTPS (FastDDS)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml

# Entrypoint: source ROS, start the ros2 CLI daemon, then exec provided command
RUN printf '%s\n' "#!/bin/bash" \
              "source /opt/ros/humble/setup.bash" \
              "ros2 daemon start" \
              "if [ \$# -eq 0 ]; then" \
              "  exec tail -f /dev/null" \
              "else" \
              "  exec \"\$@\"" \
              "fi" \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
