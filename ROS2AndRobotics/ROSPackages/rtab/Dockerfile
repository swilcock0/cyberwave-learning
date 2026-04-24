FROM ros:humble-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# Install RTAB-Map SLAM (avoid recommended packages, clean caches)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rtabmap-launch \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-util \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/* /tmp/* /var/tmp/* /usr/share/doc/* /usr/share/man/*

# Source ROS 2 automatically for all interactive bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc