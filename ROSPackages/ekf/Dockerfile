FROM ros:humble-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# Install EKF sensor fusion (avoid recommended packages, clean caches)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    python3-paho-mqtt \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/* /tmp/* /var/tmp/* /usr/share/doc/* /usr/share/man/*

# Source ROS 2 automatically for all interactive bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
