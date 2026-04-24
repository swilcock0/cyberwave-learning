# ROS 2 Nav2 Stack Container

This module runs the full ROS 2 Nav2 navigation stack (Planners, Controllers, Behavior Trees, etc.) in a dedicated, isolated Docker container (`ros2_nav2`).

It leverages the `/odom` topic and `odom -> base_footprint` TF transforms published by the neighboring `ros2_ekf` container to provide standard autonomous navigation capabilities.

## Architecture

```
                                [ ros2_ekf ]
/imu/data_raw ──► EKF ──► /odom (odom -> base_footprint)
                            │
                            ▼
/scan ───────►         [ ros2_nav2 ]
                 Nav2 Navigation Stack (Planner, Controller, BT)
                            │
                            ▼
                       /cmd_vel (Robot Velocity Commands)
```

## Quick Start

```bash
# First time only — builds the Docker image and runs the stack
/home/ws/nav2/start_nav2.sh
```

## Files & Organization

| File | Purpose |
|---|---|
| `Dockerfile` | Image definition (`ros:humble-nav2`) pulling ROS 2 Nav2 and CycloneDDS |
| `start_nav2.sh` | Orchestration script - builds the image and brings up the container network |
| `config/nav2_params.yaml`| Nav2 configuration (Costmaps, controllers, frame IDs, AMCL overrides) |
| `launch/` | (Optional) Store custom Nav2 python launch files |

## Configuration

By default, the stack uses the provided `config/nav2_params.yaml`. Because the `ekf` stack natively outputs to the `base_footprint` frame, ensure that the Nav2 `base_frame_id` inside the `nav2_params.yaml` configs natively matches `base_footprint` rather than `base_link` if using AMCL or map servers.

For full localization against a pre-recorded Map, switch the launch file in `start_nav2.sh` from `navigation_launch.py` to `bringup_launch.py` and supply a `map:=/path/to/my_map.yaml` argument.