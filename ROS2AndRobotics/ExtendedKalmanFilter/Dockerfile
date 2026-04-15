FROM ros:humble-ros-core

# Install CycloneDDS middleware and EKF sensor fusion
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    python3-paho-mqtt \
    && rm -rf /var/lib/apt/lists/*

# Set the default middleware to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS 2 automatically for bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
