# Behavior Tree Tests

This directory contains experiments using the `py_trees` library to manage complex robot behaviors.

## Files

- **[behaviors.py](./behaviors.py)**: Implementations of custom behavior leaves, including PID tracking for AprilTags and safety monitors.
- **[main.py](./main.py)**: A mock simulation of the "AI Site Foreman" behavior tree, demonstrating safety overrides, mission logic, and emergency handling.
- **[real_main.py](./real_main.py)**: A scaffold for running the behavior tree on real hardware, following the integration roadmap.
- **[Hardware_Integration_Guide.md](./Hardware_Integration_Guide.md)**: A 5-phase plan to wire the behaviors to the physical UGV.

## Behavior Architecture

The "Foreman" tree is built with a parallel root:
1. **Safety Monitor**: Continuous checks for obstacles, battery level, and drop-offs.
2. **Mission Logic**: Handles the primary investigation tasks, path following, and VLA querying.
