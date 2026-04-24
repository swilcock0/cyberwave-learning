# Project: AI Site Foreman

This directory contains the final integrated project components for the **AI Site Foreman** robot.

## Components

- **[apriltag_localizer.py](./apriltag_localizer.py)**: Translates AprilTag detections and known tag definitions into absolute robot poses in the map/API coordinate frame.
- **[display_system.py](./display_system.py)**: The main integrated dashboard built with `customtkinter`. It combines the map navigator, AprilTag localization, and automated VLA workflows for hazard detection and site interrogation.

## Key Features
- **Absolute Localization**: Uses `solvePnP` + coordinate transforms to align the robot with the environment map.
- **VLA Integration**: Automatically triggers Vision-Language models to inspect the environment (e.g., checking for hazards like trailing cables).
- **Interactive Missions**: Returns to a home position `[-0.145, -0.25, 0.0]` between site inspection tasks.
