# Cyberwave Learning

[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)


Welcome to the **Cyberwave Learning** repository. This workspace contains a collection of tutorials, experiments, and configuration examples designed to help you integrate and extend the Cyberwave platform, specifically focusing on ROS 2 integration, sensor fusion, and Python-based digital twin interactions. This has all just come from my own experiments as part of the first [Cyberwave Builder's Cohort](https://cyberwave.com/built-with-cyberwave), and will likely become out of date gradually as the [Cyberwave SDK](https://docs.cyberwave.com/get-started) develops further.

I've chosen to build my robot based on a containerised architecture, so that all the main components are within their own Docker containers. This does introduce a small amount of additional overhead storage space wise (I've upgraded the SD card from the original 32Gb). The reasons for this are twofold; it fits in with the Cyberwave system quite nicely, and honestly its been a few years since I've worked with Docker so its a nice opportunity to brush up my skills :P 


## Project brief and links
Team buildPARITY

Autonomous monitoring in construction environments requires a difficult balance between high-level semantic reasoning and low-latency physical safety. Current solutions lack the intelligence to differentiate between structural components and safety hazards, while cloud-only VLA (Vision-Language-Action) models introduce dangerous latency for real-time navigation. The challenge is to fuse ROS2-based sensor data (Lidar/Odometry) with high-compute VLA models via a tiered architecture that maintains safety-critical overrides on a resource-constrained Raspberry Pi 5 while leveraging cloud-side intelligence for mission-level decision-making and hazard identification.

Tech Stack: Cyberwave Edge Core, Cyberwave Cloud, Cyberwave Workflows and Drivers, ROS 2 (Humble), Python/C++, and ros_navigation (Nav2) utilizing Kalman filters for sensor fusion, rtab-map. Hardware includes a Raspberry Pi 5 and a D500 Lidar.

Control: Mission dispatching via a remote Control PC that dissects AI responses into discrete ROS2 motion tasks and telemetry requests.
Integration: The system utilizes the Cyberwave SDK as a bridge between the Edge (Pi 5) and the Cloud, handling telemetry routing, MJPEG video streaming, and MQTT-based command execution. Future expansion includes coordination with a Fanuc CRX10iA/L arm for assembly operations through the Cyberwave cloud and behaviour trees for advanced vla driven behaviours.


Resources:
GitHub Repo: [https://github.com/swilcock0/cyberwave-learning](https://github.com/swilcock0/cyberwave-learning)

Project Planning Doc: https://github.com/swilcock0/cyberwave-learning/blob/main/HighLevelPlan.pdf

D500 Lidar Driver Integration: https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver

Cyberwave SDK PR: https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/pull/1

MOAR UGV ADJUSTMENTS: https://github.com/swilcock0/cyberwave-edge-ros-ugv/
Video: https://www.youtube.com/watch?v=A-wZtpicR24

Blog: https://www.samwilcock.xyz/posts/2026-04-15-Cyberwave-Lidar/ 


## 📂 Repository Structure

The repository is organized into several modules, each targeting a specific aspect of the Cyberwave ecosystem:

### 🤖 ROS2 & Robotics
- **[ROS2 Container Connection](./ROS2AndRobotics/ROS2ContainerConnection/)**: Configuration and Docker setup for establishing DDS connectivity between ROS 2 containers using FastDDS. This one is great to follow first, as you'll end up with a ros introspection tool you can call for running ros (but in a container)
- **[Extended Kalman Filter (EKF)](./ROS2AndRobotics/ExtendedKalmanFilter/)**: A comprehensive guide for fusing Odometry, IMU, and Magnetometer data using the ROS 2 `robot_localization` package. This module includes:
  - Dockerized environment for EKF.
  - Calibration scripts for IMU and Magnetometer.
  - Demonstration of MQTT sidecar bridges for forwarding new topics (not necessarily following the original design pattern/schema/intent - this was an early experiment! See [Driver Creation](./Drivers/DriverCreation/) for an example that actually uses the Cybewave SDK for that).
- **[Hardware Deployments](./ROS2AndRobotics/ROSPackages/)**: Collection of containerized ROS 2 packages used on the robot, including Nav2, RTAB-Map, and EKF configurations. NOTE if you switch to the [swilcock0/ugv-driver:dev](https://hub.docker.com/r/swilcock0/ugv-driver/tags) UGV driver, you will NOT need to run an EKF since I fixed the orientation issues.
<!-- - **[dev_sam](./dev_sam/)**: Development workspace containing experimental Docker setups for:
  - `dros2`: Base ROS 2 environments.
  - `ekf`: Experimental EKF configurations.
  - `nav2`: Navigation 2 stack integration.
  - `rtab`: RTAB-Map SLAM configs. -->

### 🛠️ Drivers & Utilities
- **[Modifying Driver In Place](./Drivers/ModifyingDriverInPlace/)**: Resources and backups for modifying Cyberwave edge drivers directly on the target hardware using Docker volume mounts.
- **[Driver Creation](./Drivers/DriverCreation/)**: End-to-end guide to creating a hardware edge driver, creating and uploading a new asset, connecting them together, and connecting to a parent twin.

### 🧠 High-Level Intelligence
- **[Map & Navigation](./Intelligence/Map/)**: Interactive operator map tool with live pose, Lidar projection, and zone masking.
- **[Workflow Automation](./Intelligence/Workflows/)**: Scripts to automate complex VLA (Vision-Language Action) workflows via the Cyberwave API.

### 🏗️ Projects & Behaviors
- **[AI Site Foreman](./Project/)**: The primary integrated project combining AprilTag absolute localization, the dashboard system, and automated site interrogation.
- **[Behavior Tree Tests](./pytrees_test/)**: Experiments with `py_trees` to implement sophisticated logic like proportional tracking with dropout compensation and safety overrides.

### 🎮 Tutorials
- **[CameraStream](./Tutorials/CameraStream/)**: A Python tutorial demonstrating how to stream live camera feeds from a UGV digital twin using the Cyberwave API and OpenCV.

## 🚀 Getting Started

Most modules in this repository come with their own `README.md` and setup scripts. 

1. **Python API**: Ensure you have the `cyberwave-python` package installed in your Python environment.
2. **Docker**: ROS2 components are containerised.
3. **Hardware**: These tutorials are primarily tested with the **UGV Beast** platform.

## 🔗 Related Repositories
- **[cyberwave-python](https://github.com/cyberwave-os/cyberwave-python)**: The official Python SDK for Cyberwave.
- **[cyberwave-edge-ros-ugv](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv)**: ROS 2 bridge and adapter for UGV edge devices.

---
## Progress
![Progress](resources/progress.jpg)

---
*Created for educational and demonstrative purposes at Politecnico di Milano.*
