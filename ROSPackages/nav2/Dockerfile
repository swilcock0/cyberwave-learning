FROM ros:humble-ros-core
ENV DEBIAN_FRONTEND=noninteractive

# Install Nav2 stack (avoid recommended packages, clean caches)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-bringup \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /var/cache/apt/* /tmp/* /var/tmp/* /usr/share/doc/* /usr/share/man/*

# Source ROS 2 automatically for all interactive bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
