FROM ros:humble-ros-core

# Install CycloneDDS middleware
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set the default middleware to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optional: Source ROS 2 automatically for bash sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc