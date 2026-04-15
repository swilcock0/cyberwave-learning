# Tutorial 1: Creating a Cyberwave Driver

Welcome to the first step in extending the Cyberwave ecosystem. This tutorial will guide you through the process of creating a brand-new driver for a hardware device. For reference, I have created a driver for the Waveshare D500 Lidar kit (a.k.a. the LDRobot LD19 DTOF sensor):

Example driver [cyberwave-edge-d500-lidar-driver](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver)
Original LDRobot SDK [ldlidar_stl_sdk](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk)

For further information, see the original documentation I've followed on [Cyberwave Docs - Writing Compatible Drivers](https://docs.cyberwave.com/edge/drivers/writing-compatible-drivers)

## Overview

A Cyberwave driver acts as a bridge between physical hardware (sensors, actuators, motors) and the Cyberwave Digital Twin platform. It is typically a containerized application running on the **Cyberwave Edge**.

### Key Concepts

1.  **Digital Twin Connection**: The driver uses the `cyberwave` Python SDK to communicate with the platform.
2.  **Hardware Abstraction**: A dedicated `hardware.py` or similar layer handles low-level communication (Serial, I2C, SPI, C++ SDKs).
3.  **Telemetry**: High-frequency data (like Lidar scans or IMU readings) is published via MQTT or Zenoh.
4.  **Edge Configs**: Metadata in the Digital Twin that allows you to configure the driver (e.g., serial port) without changing the code.

## Folder Structure

A standard driver project should follow this structure:

```text
cyberwave-edge-[device]-driver/
├── .github/workflows/      # CI/CD for Docker builds
├── cyberwave_edge_.../     # Main Python package
│   ├── __init__.py
│   ├── driver.py          # Orchestration and SDK logic
│   ├── hardware.py        # Low-level hardware client
│   └── main.py            # Entry point
├── Dockerfile              # Container definition
├── pyproject.toml          # Build and dependency metadata
└── README.md
```

## Step-by-Step Guide

### 1. Use the driver-skill/scaffold.py!
There's a really handy tool for getting the boilerplate in place for the projects at [cyberwave-os/driver-skill](https://github.com/cyberwave-os/driver-skill). I don't have Claude code to use the skill, but the scaffold.py does a great job at setting up the templates for you. Check that repo for further instructions.

### 2. Implement the Hardware Client
Create a `HardwareClient` class ([hardware.py](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver/blob/main/cyberwave_edge_d500_lidar_driver/hardware.py)) that encapsulates the interaction with your device. This should be an abstraction layer so the rest of the driver doesn't need to know about registers, serial protocols, or specific SDK calls.

**Templates vs. Real Implementation (Serial Lidar):**

| Feature | `driver-skill` Template | `d500-lidar-driver` (Real Example) |
|---|---|---|
| **Initialization** | Stores `config` dict from metadata. | Resolves `serial_port` and `product_name` from config or env vars. |
| **SDK Integration** | Bare Python stub. | Uses `ctypes` to load `liblidar_wrapper.so` and defines C-structs for `Point` and `ScanResult`. |
| **Connection** | `logger.info(...)` placeholder. | **`connect()`**: Encodes strings to UTF-8 and calls `lidar_start(product, port)` via the C-library. Raises errors if it fails. |
| **Data Retrieval** | `read_state()` returning an empty dict. | **`read_state()`**: Polls `lidar_get_scan()`, unpacks C-pointers into a Python list of dictionaries, and **critically** calls `lidar_free_scan()` in a `finally` block to prevent memory leaks. |
| **Disconnection** | `pass` | **`disconnect()`**: Calls `lidar_stop()` to release the serial port and stop the DTOF spin. |

**Key takeaway**: When mapping `read_state()`, ensure any memory allocated by a C-library is explicitly freed. Use `ctypes.POINTER` and indexing (e.g., `res.points[i]`) to iterate through hardware buffers.

### 3. Build the Driver Logic
The `Driver` class ([driver.py](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver/blob/main/cyberwave_edge_d500_lidar_driver/driver.py)) coordinates the flow between the hardware and the Cyberwave platform.

**Evolution from Template to Production:**

1.  **Orchestration**:
    *   *Template*: Simple loop reading state and updating the local `twin.json` file.
    *   *Production (Lidar)*: High-frequency polling loop (`0.005s`) that publishes raw sensor data over MQTT/Zenoh for real-time visualization.
2.  **Platform Integration**:
    *   Import the `Cyberwave` SDK.
    *   **Health Checks**: Initialize `EdgeHealthCheck` to prevent the platform from marking the device as "stale".
    *   **Data Publishing**: Use `cw.mqtt.publish` for telemetry. For the Lidar driver, we use the `.../scan` topic to ensure the payload (matching a ROS `LaserScan`) is correctly routed to the digital twin's live visualization.
3.  **Advanced Features**: 
    *   In the Lidar driver, we also added **Native ROS 2 Publishing** using `rclpy`, allowing the same hardware data to be consumed by local ROS nodes while simultaneously feeding the Cyberwave Twin via MQTT.

### 4. Containerize
Write a `Dockerfile` that installs the necessary SDKs and sets up the entry point. For the D500 Lidar, this involved:
*   Starting from a ROS 2 base image.
*   Installing build tools to compile the C++ wrapper.
*   Copying the `.so` library to `/usr/local/lib`.
*   Setting `LD_LIBRARY_PATH`.

### 5. Local Testing with Docker Compose
Before deploying to the platform, you should test your driver locally using `docker-compose.yml` ([example](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver/blob/main/docker-compose.yml)). This allows you to simulate the Cyberwave Edge environment.

*   **`.env` Files**: Store your `CYBERWAVE_API_KEY` and `CYBERWAVE_TWIN_UUID` in a local `.env` file. The `docker-compose.yml` will load these as environment variables.
*   **Device Mapping**: Use the `devices` section in your compose file to map physical hardware (e.g., `/dev/ttyUSB0`) into the container.
*   **Mocking**: You can use a local `twin.json` file to mimic the one the Edge Core would normally provide.

I tend to use the following command when testing (which lets me view the logs)
```bash
sudo docker compose up --build; sleep 5; sudo docker compose logs -f```

**Note**: This setup is strictly for local development and testing. Once deployed to a real Cyberwave Edge node, the platform handles container orchestration, environment variables, and metadata syncing automatically.

### 6. Deploy
Push the Docker image to a registry and configure your Digital Twin in the Cyberwave Dashboard (I've done this with a [GitLab registry action](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver/blob/main/.github/workflows/push_to_gitlab.yaml) - ensure you're using Git secrets and not pushing API keys!)

N.B.: There also seems to be a way to push drivers to the Cyberwave registry, see [Docker Registry](https://docs.cyberwave.com/use-cyberwave/docker-registry).

## Next Steps

- [2. Creating a catalogue asset](/Drivers/DriverCreation/2_CreatingACatalogueAsset/README.md)
- [Modifying an Existing Driver](/ModifyingDriverInPlace/README.md)
- [Docker Registry](https://docs.cyberwave.com/use-cyberwave/docker-registry).
- [Cyberwave SDK Documentation](https://docs.cyberwave.com)
