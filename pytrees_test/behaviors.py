import py_trees

class TrackTagInCenter(py_trees.behaviour.Behaviour):
    def __init__(self, name, ugv_twin):
        super(TrackTagInCenter, self).__init__(name)
        self.ugv = ugv_twin
        
        # PID tuning gains
        self.kp_pan = 0.05
        self.ki_pan = 0.001
        self.kd_pan = 0.01
        
        self.kp_tilt = 0.05
        self.ki_tilt = 0.001
        self.kd_tilt = 0.01

        # PID State
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0
        
        # State for dealing with AprilTag dropouts
        self.last_known_x = None
        self.last_known_y = None
        
        # Blackboard to read camera data shared by other threads
        self.blackboard = py_trees.blackboard.Client(name="Tracker")
        self.blackboard.register_key(key="tag_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="tag_y", access=py_trees.common.Access.READ)

    def initialise(self):
        # Called right before this node starts "RUNNING"
        self.logger.debug(f"{self.name} started.")

    def update(self):
        # Read the latest Tag (u, v) from the vision thread
        try:
            tag_x = self.blackboard.tag_x
            tag_y = self.blackboard.tag_y
            
            if tag_x is None or tag_y is None:
                raise KeyError
            
            # Update dropout memory to the newest read
            self.last_known_x = tag_x
            self.last_known_y = tag_y
            
        except KeyError:
            # If the vision thread doesn't see a tag, check if we have a last known pose
            if self.last_known_x is not None:
                # 1. Read base rotation from the UGV (Mocked if unavailable)
                try:
                    # In a real robot, this gets the current Turn Speed (rad/s)
                    # e.g., base_turn_rate = self.ugv.get_angular_velocity()
                    base_turn_rate = 0.0 
                except AttributeError:
                    base_turn_rate = 0.0
                
                # 2. Shift the estimated X coordinate opposite to the turn direction
                # If robot turns left (+rad/s), the tag shifts right (+x pixels)
                # (Assuming roughly 500 pixels per radian horizontally, ticked at 10Hz)
                pixel_shift = -(base_turn_rate * 500.0 * 0.1)
                self.last_known_x += pixel_shift

                tag_x = self.last_known_x
                tag_y = self.last_known_y
                print(f"[{self.name}] AprilTag dropped out! Compensating for base motion. Est pose: ({tag_x:.1f}, {tag_y:.1f})")
            else:
                # If we've never seen the tag, we cannot track it
                return py_trees.common.Status.FAILURE 

        # Calculate error from center
        error_x = 320 - tag_x
        error_y = 240 - tag_y
        
        # If error is small enough, we are SUCCESSFUL (centered)
        if abs(error_x) < 20 and abs(error_y) < 20:
            return py_trees.common.Status.SUCCESS

        # PID Math
        self.integral_x += error_x
        self.integral_y += error_y
        
        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y

        pan_adjustment = (error_x * self.kp_pan) + (self.integral_x * self.ki_pan) + (derivative_x * self.kd_pan)
        tilt_adjustment = (error_y * self.kp_tilt) + (self.integral_y * self.ki_tilt) + (derivative_y * self.kd_tilt)
        
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        
        print(f"[{self.name}] PID Adjusting -> pan {pan_adjustment:.2f}, tilt {tilt_adjustment:.2f}")

        # Send to Cyberwave 
        # self.ugv.set_pantilt_offsets(pan_adjustment, tilt_adjustment)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f"{self.name} terminated to {new_status}")

class TagSearchSweep(py_trees.behaviour.Behaviour):
    """Actively sweeps the PTZ camera to find a lost AprilTag."""
    def __init__(self, name, ugv_twin):
        super(TagSearchSweep, self).__init__(name)
        self.ugv = ugv_twin
        self.sweep_angle = -45.0
        
        self.blackboard = py_trees.blackboard.Client(name="Searcher")
        self.blackboard.register_key(key="tag_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="tag_y", access=py_trees.common.Access.READ)

    def update(self):
        try:
            # If the background vision thread has written a coordinate, we found it!
            if getattr(self.blackboard, "tag_x", None) is not None:
                print(f"[{self.name}] Tag spotted in vision feed! Disengaging search sweep.")
                return py_trees.common.Status.SUCCESS
        except KeyError:
            pass
            
        # Otherwise, keep moving the camera
        self.sweep_angle += 15.0
        print(f"[{self.name}] Tag lost. Sweeping camera... Pan angle now at {self.sweep_angle} deg")
        
        return py_trees.common.Status.RUNNING

class MoveToProximity(py_trees.behaviour.Behaviour):
    """Simulates moving to the overall construction zone using SLAM/RTAB-Map."""
    def __init__(self, name, ugv_twin):
        super(MoveToProximity, self).__init__(name)
        self.ugv = ugv_twin
        self.journey_ticks = 0

    def update(self):
        self.journey_ticks += 1
        if self.journey_ticks < 3:
            print(f"[{self.name}] Sending SDK Nav2 Goal to coordinate (X: 10.5, Y: -2.3, Yaw: 1.57)...")
            # self.ugv.send_nav2_goal(10.5, -2.3, 1.57)
            return py_trees.common.Status.RUNNING
        
        print(f"[{self.name}] Arrived at inspection zone boundary. Nav2 Goal Reached. SUCCESS.")
        return py_trees.common.Status.SUCCESS


class MoveBaseToTag(py_trees.behaviour.Behaviour):
    """Simulates driving the base slowly towards the target tag while the camera tracks it."""
    def __init__(self, name, ugv_twin):
        super(MoveBaseToTag, self).__init__(name)
        self.ugv = ugv_twin
        self.journey_ticks = 0

    def update(self):
        self.journey_ticks += 1
        if self.journey_ticks < 6:
            print(f"[{self.name}] Commanding SDK Velocity: V=0.2m/s, W=0.0rad/s...")
            # self.ugv.set_velocity(linear=0.2, angular=0.0)
            return py_trees.common.Status.RUNNING
            
        print(f"[{self.name}] Base is at target distance to Tag. SUCCESS.")
        # self.ugv.set_velocity(0, 0)
        return py_trees.common.Status.SUCCESS

class CheckLidarObstacle(py_trees.behaviour.Behaviour):
    """Safety monitor that checks planar LIDAR data for imminent collisions (walls/objects)."""
    def __init__(self, name, ugv_twin):
        super(CheckLidarObstacle, self).__init__(name)
        self.ugv = ugv_twin
        
        self.blackboard = py_trees.blackboard.Client(name="LidarSafety")
        self.blackboard.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.hazard_detected = False

    def update(self):
        # In a real robot, read 2D Lidar array for obstacles
        if getattr(self.blackboard, "hazard_detected", False):
            print(f"[{self.name}] DANGER! Planar obstacle detected. FAILURE triggered.")
            return py_trees.common.Status.FAILURE
            
        print(f"[{self.name}] Planar path clear.")
        return py_trees.common.Status.SUCCESS

class CheckVisionDropoff(py_trees.behaviour.Behaviour):
    """Pipes images to VLA or Segmentation to detect holes/drop-offs that Planar Lidar misses."""
    def __init__(self, name, ugv_twin):
        super(CheckVisionDropoff, self).__init__(name)
        self.ugv = ugv_twin
        
        self.blackboard = py_trees.blackboard.Client(name="VisionSafety")
        self.blackboard.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.hazard_detected = False

    def update(self):
        # Checks vision/LLM classification feed for floor hazards
        if getattr(self.blackboard, "hazard_detected", False):
            print(f"[{self.name}] DANGER! Vision model detected floor drop-off. FAILURE triggered.")
            return py_trees.common.Status.FAILURE
            
        print(f"[{self.name}] Floor geometry looks safe to VLA.")
        return py_trees.common.Status.SUCCESS

class StopAndStrobe(py_trees.behaviour.Behaviour):
    """Emergency action when a hazard is detected."""
    def __init__(self, name, ugv_twin):
        super(StopAndStrobe, self).__init__(name)
        self.ugv = ugv_twin

    def update(self):
        print(f"[{self.name}] EMERGENCY HALT. Flashing warning strobes!")
        print(f"[{self.name}] Cancelling SDK Nav2 goals and cutting motors...")
        # self.ugv.cancel_nav2_goal()
        # self.ugv.set_velocity(0, 0)
        return py_trees.common.Status.SUCCESS

class PanTilt_Scan(py_trees.behaviour.Behaviour):
    """Sweeps the PTZ camera left/right to stitch a wide field-of-view."""
    def __init__(self, name, ugv_twin):
        super(PanTilt_Scan, self).__init__(name)
        self.ugv = ugv_twin
        self.scan_step = 0

    def update(self):
        steps = ["Center", "Left (-30 deg)", "Right (+30 deg)"]
        if self.scan_step < len(steps):
            print(f"[{self.name}] Snapping high-res photo at position: {steps[self.scan_step]}...")
            self.scan_step += 1
            return py_trees.common.Status.RUNNING
            
        print(f"[{self.name}] Pan-Tilt sweep complete. SUCCESS.")
        return py_trees.common.Status.SUCCESS

class VLA_Query(py_trees.behaviour.Behaviour):
    """Sends stitched images to Physical AI Cloud Bridge for semantic inference."""
    def __init__(self, name, prompt):
        super(VLA_Query, self).__init__(name)
        self.prompt = prompt
        self.processing_ticks = 0
        self.elaboration_requested = False
        
        # Blackboard to write the VLA response for other nodes (like Report Generator)
        self.blackboard = py_trees.blackboard.Client(name="VLA_Node")
        self.blackboard.register_key(key="vla_response", access=py_trees.common.Access.WRITE)

    def update(self):
        if self.processing_ticks == 0:
            print(f"[{self.name}] Sending images to VLA Cloud. Prompt: '{self.prompt}'")
        
        self.processing_ticks += 1
        
        if self.processing_ticks < 3:
            print(f"[{self.name}] Awaiting cloud VLA response...")
            return py_trees.common.Status.RUNNING
            
        # Simulate VLA returning an "Elaborate" action for ambiguity once
        if not self.elaboration_requested:
            self.elaboration_requested = True
            print(f"\n[{self.name}] VLA Action: ELABORATE.")
            print(f"[{self.name}] VLA says: 'I see a cylindrical object on the ground, unclear if it's a pipe or a hazard.'")
            
            # Wait for user input (blocks the tree, acceptable for human-in-the-loop exception)
            user_info = input(">>>> [Human-in-the-Loop] Please elaborate for the VLA: ")
            
            # Restructure the input
            self.prompt = f"Context: Cylindrical object. Human classification: {user_info}. Re-evaluate safety and summarize."
            
            print(f"[{self.name}] Restructured input. Re-querying VLA with: '{self.prompt}'")
            self.processing_ticks = 0 # reset processing to simulate another network query
            return py_trees.common.Status.RUNNING
            
        mock_response = "Zone assessed with human input. Status: CLEARED."
        self.blackboard.vla_response = mock_response
        print(f"[{self.name}] VLA Reply: '{mock_response}' SUCCESS.")
        return py_trees.common.Status.SUCCESS

class PublishSemanticMarker(py_trees.behaviour.Behaviour):
    """Queries SDK for RTAB-Map pose and publishes 3D item marker to host UI."""
    def __init__(self, name, ugv_twin):
        super(PublishSemanticMarker, self).__init__(name)
        self.ugv = ugv_twin
        self.blackboard = py_trees.blackboard.Client(name="MarkerPublisher")
        self.blackboard.register_key(key="vla_response", access=py_trees.common.Access.READ)

    def update(self):
        try:
            vla_text = self.blackboard.vla_response
            print(f"[{self.name}] Querying SDK for current RTAB-Map pose & Depth...")
            # rtab_pose = self.ugv.get_rtabmap_pose()
            rtab_pose = {"x": 10.5, "y": -2.3, "z": 0.0} # mock
            
            # If VLA detected an object or human, we drop a marker
            marker_payload = {"label": vla_text[-25:], "x": rtab_pose['x'], "y": rtab_pose['y'], "z": rtab_pose['z']}
            print(f"[{self.name}] Sending Semantic Map Marker to Host UI: {marker_payload}")
            # self.ugv.send_map_marker(marker_payload)
            return py_trees.common.Status.SUCCESS
        except KeyError:
            return py_trees.common.Status.FAILURE

class GenerateSiteReport(py_trees.behaviour.Behaviour):
    """Formats VLA text and Lidar coordinates into a JSON log for the host, as described in the Foreman plan."""
    def __init__(self, name):
        super(GenerateSiteReport, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="ReportGenerator")
        self.blackboard.register_key(key="vla_response", access=py_trees.common.Access.READ)

    def update(self):
        try:
            vla_text = self.blackboard.vla_response
        except KeyError:
            vla_text = "No VLA data available."
            
        print(f"[{self.name}] Generating Final JSON Site Report...")
        print(f"   -> {{'zone': 'Electrical Sector A', 'vla_analysis': '{vla_text}', 'status': 'CLEARED'}}")
        return py_trees.common.Status.SUCCESS

class CheckBattery(py_trees.behaviour.Behaviour):
    """Stub: Checks if battery voltage is sufficient."""
    def update(self):
        # Insert hardware call here
        return py_trees.common.Status.SUCCESS

class HazardFoundOnBlackboard(py_trees.behaviour.Behaviour):
    """Condition node: returns SUCCESS if hazard detected, else FAILURE."""
    def __init__(self, name):
        super(HazardFoundOnBlackboard, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="ConditionCheck")
        self.blackboard.register_key(key="hazard_detected", access=py_trees.common.Access.READ)
    
    def update(self):
        if hasattr(self.blackboard, "hazard_detected") and self.blackboard.hazard_detected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IdleRecharge(py_trees.behaviour.Behaviour):
    """Stub action to return home or sleep."""
    def update(self):
        print(f"[{self.name}] Returning to dock / Idling...")
        return py_trees.common.Status.SUCCESS

# =====================================================================
# ADVANCED STUBS FROM 'Behaviours.md'
# =====================================================================

class FrontierExploration(py_trees.behaviour.Behaviour):
    """Systematic spiral search to fill in the Lidar map in unknown areas."""
    def update(self):
        print(f"[{self.name}] Low map features detected. Executing spiral exploration...")
        return py_trees.common.Status.RUNNING

class DynamicObstacleAvoidance(py_trees.behaviour.Behaviour):
    """High-priority node monitoring Lidar for dynamic obstacles (e.g. walking humans)."""
    def update(self):
        print(f"[{self.name}] Checking for dynamic obstacles in path...")
        return py_trees.common.Status.SUCCESS

class LostRecovery(py_trees.behaviour.Behaviour):
    """Visual Landmark Search to re-anchor global pose if SLAM gets lost."""
    def update(self):
        print(f"[{self.name}] SLAM lost. Spinning Pan-Tilt to find known AprilTag...")
        return py_trees.common.Status.RUNNING

class PrecisionApproach(py_trees.behaviour.Behaviour):
    """Switches to Visual Odometry for centimeter-perfect positioning."""
    def update(self):
        print(f"[{self.name}] Within 1m of target. Engaging Visual Odometry approach...")
        return py_trees.common.Status.SUCCESS

class FoveatedTargetSearch(py_trees.behaviour.Behaviour):
    """Detect general objects and compute pan-tilt angles to center/zoom."""
    def update(self):
        print(f"[{self.name}] Scanning wide-angle for desired object class...")
        return py_trees.common.Status.SUCCESS

class GazePersistence(py_trees.behaviour.Behaviour):
    """Head compensates for base turning to maintain visual lock."""
    def update(self):
        print(f"[{self.name}] Compensating PTZ for base rotation. Target locked.")
        return py_trees.common.Status.SUCCESS

class VolumetricSweep(py_trees.behaviour.Behaviour):
    """Grid-based sweep (e.g., 3x3) to create high-resolution panoramic stitch."""
    def update(self):
        print(f"[{self.name}] Executing 3x3 Volumetric Pan-Tilt sequence...")
        return py_trees.common.Status.SUCCESS

class SafetyPerimeterAudit(py_trees.behaviour.Behaviour):
    """Monitors for PPE compliance via VLA."""
    def update(self):
        print(f"[{self.name}] VLA PPE Audit: All workers wearing hard hats.")
        return py_trees.common.Status.SUCCESS

class InventoryCount(py_trees.behaviour.Behaviour):
    """Navigates to Material Zone and asks VLA to count items."""
    def update(self):
        print(f"[{self.name}] VLA Count: 10 bags of mortar detected. Matches expected.")
        return py_trees.common.Status.SUCCESS

class HazardDetection(py_trees.behaviour.Behaviour):
    """Background behavior piping low-res frames to VLA for hazard ID."""
    def update(self):
        print(f"[{self.name}] Background VLA scan: No standing water or bad wiring.")
        return py_trees.common.Status.SUCCESS

class ProgressVerification(py_trees.behaviour.Behaviour):
    """Compares current scene with Gold Standard image for progress."""
    def update(self):
        print(f"[{self.name}] VLA Diff: HVAC ducts extended 3 meters since yesterday.")
        return py_trees.common.Status.SUCCESS

class CommsBuffer(py_trees.behaviour.Behaviour):
    """Caches images locally if cloud connection drops."""
    def update(self):
        print(f"[{self.name}] Link steady. (Would cache locally if disconnected.)")
        return py_trees.common.Status.SUCCESS

class StuckRecovery(py_trees.behaviour.Behaviour):
    """Initiates a 'Wiggle' if motors draw high current without movement."""
    def update(self):
        print(f"[{self.name}] Checking motor draw vs odometry. Not stuck.")
        return py_trees.common.Status.SUCCESS

class BatteryTriage(py_trees.behaviour.Behaviour):
    """Prunes expensive behaviors if voltage drops."""
    def update(self):
        print(f"[{self.name}] Battery optimal. Full behaviors enabled.")
        return py_trees.common.Status.SUCCESS


