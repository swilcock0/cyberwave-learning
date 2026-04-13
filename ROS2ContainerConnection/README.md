# Introspection

If you are used to ROS2, an easy way to get introspection into the robot ROS2 topics is to add an execution function to the .bashrc profile. 

First build the Dockerfile given here
```bash
ws@ugvrpi:~ sudo docker build -t ros:humble-cyclone .

[+] Building 0.9s (11/11) FINISHED                                                     docker:default
...
 => => unpacking to docker.io/library/ros:humble-cyclone 
```

### Configuration

For the DDS communication to work properly across containers and the host, you must place the `fastdds_udp.xml` file in a location accessible by the Docker daemon. 

1. **Move the file** to `/home/ws/` on your robot (or update the path in the script below):
   ```bash
   cp ROS2ContainerConnection/fastdds_udp.xml /home/ws/fastdds_udp.xml
   ```

2. **Add the function** into the `~/.bashrc` profile. Notice the volume mount `-v /home/ws/fastdds_udp.xml:/fastdds_udp.xml` which makes the configuration available inside the container.

```bash
# --- ROS 2 Background Daemon Configuration ---
ROS_CONTAINER_NAME="ros2_daemon"

# Function to start the daemon if it's not already running
start_ros_daemon() {
    if sudo docker ps --format '{{.Names}}' | grep -q "^${ROS_CONTAINER_NAME}$"; then
        # Already running, nothing to do
        return
    elif sudo docker ps -a --format '{{.Names}}' | grep -q "^${ROS_CONTAINER_NAME}$"; then
        echo "Starting ROS 2 Background Daemon (${ROS_CONTAINER_NAME})..."
        sudo docker start "$ROS_CONTAINER_NAME"
    else
        echo "Starting ROS 2 Background Daemon (${ROS_CONTAINER_NAME})..."
        # Start in detached mode (-d), using host networking and IPC
        sudo docker run -d \
            --name "$ROS_CONTAINER_NAME" \
            --network host \
            --ipc host \
            --privileged \
            -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            -v /home/ws/fastdds_udp.xml:/fastdds_udp.xml \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml \
            ros:humble-cyclone
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

_dros2_complete() {
    local cur=${COMP_WORDS[COMP_CWORD]}
    local line="${COMP_LINE}"
    
    # 1. Translate host command 'dros2' to container command 'ros2'
    local ros2_line="ros2 ${line#*dros2 }"
    # Ensure we calculate the point correctly if dros2 isn't the only word
    local ros2_point=$((${#ros2_line}))

    # 2. Grab suggestions from the container
    # We use 'tr' to convert the Vertical Tab (\v) to a Newline (\n)
    # and remove carriage returns (\r)
    local suggestions=$(sudo docker exec "$ROS_CONTAINER_NAME" \
        /bin/bash -c "source /opt/ros/humble/setup.bash && \
        _ARGCOMPLETE=1 \
        COMP_LINE='$ros2_line' \
        COMP_POINT='$ros2_point' \
        ros2 8>&1 9>&2 1>/dev/null 2>/dev/null" | tr '\v' '\n' | tr -d '\r')

    # 3. Fill the completion array, splitting by newline
    local IFS=$'\n'
    COMPREPLY=( $(compgen -W "$suggestions" -- "$cur") )
}

complete -F _dros2_complete dros2

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