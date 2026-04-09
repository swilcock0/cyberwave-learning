# Introspection

If you are used to ROS2, an easy way to get introspection into the robot ROS2 topics is to add an execution function to the .bashrc profile. 

First build the Dockerfile given here
```bash
ws@ugvrpi:~ sudo docker build -t ros:humble-cyclone .

[+] Building 0.9s (11/11) FINISHED                                                     docker:default
...
 => => unpacking to docker.io/library/ros:humble-cyclone 
```

Now add the function into the ~/.bashrc profile 
```bash
# --- ROS 2 Background Daemon Configuration ---
ROS_CONTAINER_NAME="ros2_daemon"

# Function to start the daemon if it's not already running
start_ros_daemon() {
    if ! sudo docker ps --format '{{.Names}}' | grep -q "^${ROS_CONTAINER_NAME}$"; then
        echo "Starting ROS 2 Background Daemon (${ROS_CONTAINER_NAME})..."
        # Start in detached mode (-d), using host networking and IPC
        sudo docker run -d \
            --name "$ROS_CONTAINER_NAME" \
            --network host \
            --ipc host \
            -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            ros:humble-cyclone \
            sleep infinity
    fi
}

# The dros2 command: Executes a command inside the existing daemon
dros2() {
    # Ensure the daemon is up before exec-ing
    start_ros_daemon
    
    # Use -t for colors/formatting, but -i only if stdin is a terminal
    sudo docker exec -it "$ROS_CONTAINER_NAME" \
        /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 $*"
}

# Auto-start check when you open a new shell
start_ros_daemon
```


Then you can use it as, for example
```bash
ws@ugvrpi:~ $ dros2 topic info /rosout

Type: rcl_interfaces/msg/Log
Publisher count: 10
Subscription count: 0
```

Additionally you can step fully into the docker container,
```bash
sudo docker run -it --rm --network host --ipc host ros:humble-cyclone
```
which will allow further interaction.




#### TODO
Mounting folders, volumes etc. Installing additional dependencies.