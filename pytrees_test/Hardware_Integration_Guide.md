# Hardware Integration Guide: From Mockup to Physical UGV

This guide outlines the concrete next steps to take the asynchronous `py_trees` mockup (`real_main.py` and `behaviors.py`) and wire it up to your physical Cyberwave UGV twin, utilizing the robust computer vision tools we developed earlier.

## Phase 1: The Vision Thread (Camera Integration)
Currently, the `camera_worker()` in `real_main.py` is commented out. We need to merge the logic from `Experiments/ApriltagQuick.py` into this worker thread.

**Action Items:**
1. **Move the OpenCV Pipeline:** Copy the `cv2.VideoCapture()`, Undistortion mathematical matrices (`camera_calib.npz`), CLAHE contrast adjustments, and `cv2.aruco.detectMarkers` logic into the `while True:` loop inside `camera_worker()`.
2. **Publish to Blackboard:** When `ids` are detected, calculate the center `(cX, cY)` of the desired tag, and write it to the `py_trees.blackboard` (`blackboard.tag_x = cX`, `blackboard.tag_y = cY`).
3. **Handle Lost Tags:** If `ids is None`, you must clear the blackboard values (`del blackboard.tag_x`) so the Tree knows the tag is lost and triggers the `TagSearchSweep` behavior instead of `TrackTagInCenter`.
4. **Thread Safety:** Because OpenCV's `VideoCapture.read()` is blocking, running it in this separate `threading.Thread` ensures your robot doesn't freeze while waiting for a network frame.

## Phase 2: The Planar LIDAR Callback (Safety Integration)
Your UGV Twin receives 2D (Planar) LIDAR data over the SDK. This operates as an asynchronous event.

**Action Items:**
1. **Attach the Callback:** In your main setup, assign a callback function to the UGV's LIDAR stream via the SDK (e.g., `ugv_beast.set_lidar_callback(lidar_mqtt_callback)`).
2. **Analyze the Point Cloud:** Inside `lidar_mqtt_callback`, parse the incoming array. Since the Lidar is planar, it cannot see the floor. Check ONLY for imminent frontal collisions (walls, people's legs) under a critical threshold, e.g., 0.5 meters.
3. **Trigger the Blackboard:** If the safety condition is violated, set `blackboard.hazard_detected = True`. The `CheckLidarObstacle` behavior in your tree will instantly read this and forcefully override the mission tree to trigger `StopAndStrobe`.

## Phase 2B: Vision-Based Drop-off Detection (SDK VLA/Segmentation)
Since the planar Lidar cannot see holes, you must rely on the camera.
1. **Send Frames to SDK:** In your `camera_worker`, continuously send sampled frames to the SDK's embedded segmentation or cloud VLA.
2. **Flag Hazards:** If the VLA/Vision model detects a floor drop-off, update the VisionSafety blackboard (`blackboard.hazard_detected = True`). The `CheckVisionDropoff` node handles this.

## Phase 3: Wiring Actuators & Nav2 (`behaviors.py`)
Right now, `behaviors.py` just prints intent to the console. We need to replace those strings with actual SDK API calls to your UGV.

**Action Items:**
1. **SLAM / Global Navigation:** In `MoveToProximity.update()`, replace the stub with a call to the SDK's Nav2 interface to navigate through the RTAB-Map environment: `self.ugv.send_nav2_goal(x, y, yaw)`.
2. **Local Movement Commands:** In `MoveBaseToTag.update()`, replace the print statement with direct velocity commands: `self.ugv.set_velocity(linear=0.2, angular=0.0)`.
3. **PTZ Commands:** In `TrackTagInCenter.update()`, uncomment and configure `self.ugv.set_pantilt_offsets(pan_adjustment, tilt_adjustment)`.
4. **Emergency Halts:** In `StopAndStrobe.update()`, ensure you send an explicit stop (`self.ugv.set_velocity(0, 0)`) and cancel any active Nav2 goals.

## Phase 4: Safe Tuning & Testing
When physical hardware is involved with autonomous logic loops, safety prevents damage.

**Action Items:**
1. **Bench Testing:** Literally put the UGV on a box so its wheels are off the ground.
2. **Vision Verification:** Run the script and move an AprilTag around the camera by hand. Watch the PTZ servos track it. If they oscillate wildly, lower your `kp_pan` and `kp_tilt` values in `TrackTagInCenter`.
3. **Safety Verification:** Put your hand in front of the LIDAR (or simulate a hole) and ensure the wheels immediately stop spinning and the tree reports `EMERGENCY HALT`.
4. **Floor Testing:** Once the bench test passes, set the UGV on the ground and run the full AI Foreman sequence.

---

## Phase 5: Expanding to Advanced Capabilities (The "Foreman" Logic)

Once the core navigation test (Phases 1-4) is stable, start wiring the newly added behavior stubs in `behaviors.py` that tap into the robust features described in your `Behaviours.md`:

### Active Vision Upgrades
* Use **`FoveatedTargetSearch`** and **`VolumetricSweep`**. To implement this physically, you'll update these stubs to trigger multiple `capture_high_res_frame()` commands at different pan/tilt angles (using nested sequences or for-loops within the `update` tick).
* Change your PID tracker to use **`GazePersistence`** — inject RTAB-Map odometry into your PID calculations so the PTZ offsets against the base turning speed using gyroscope data.

### Semantic Construction Inspection & Spatial Mapping
* **Semantic Map Population (RTAB-Map/Nav2):** When the VLA or object detection model identifies an item, query the SDK for the active RTAB-Map pose and depth data. Estimate the 3D world coordinates of the object and send a payload (e.g., `{"label": "PVC Pipe", "x": 1.2, "y": 3.4}`) back through the SDK to the Host computer. The Host can visually drop colored 3D markers onto its UI map interface in real-time.
* **The "Elaborate" VLA Interface**: In `VLA_Query`, you will wire the `input()` or block wait to the SDK's chat/LLM bridge. This supports Human-in-the-Loop clarification text for ambiguity.
* **`SafetyPerimeterAudit` / `InventoryCount` / `ProgressVerification`**: These all run exactly like your existing `VLA_Query`. Duplicate the prompt string methodology, but change the payload text. For example: `prompt="Count the number of mortar bags visible..."`.

### Operational Robustness (The "What-If" Handlers)
* For **`DynamicObstacleAvoidance`**, create a separate MQTT/SDK subscriber looking for objects inside a 1-meter radius box. Place this node alongside `CheckLidarObstacle` in your `Safety Monitor` Sequence subtree. 
* Add **`CommsBuffer`**: Wrap your `py_trees` blackboard calls in a condition node that pings `8.8.8.8` or your cloud IP. If it returns False, write images directly to a `/tmp/` folder instead of triggering the VLA nodes.
* Add **`StuckRecovery`**: Monitor the delta between commanded velocity and `get_rtabmap_pose()`. If $v_{cmd} > 0.5$ but $\text{translation} < 0.05\text{m}$, fail the current mission subtree and branch into a Sequence executing a `Pan-Tilt Wiggle` to get wheels unstuck.
