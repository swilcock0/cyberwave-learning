import py_trees
import time
import threading
import cv2

# Import your real UGV Twin class
# from Experiments.UGV import UGVTwin 
from behaviors import (
    TrackTagInCenter, MoveToProximity, MoveBaseToTag, 
    CheckLidarObstacle, CheckVisionDropoff, StopAndStrobe, PanTilt_Scan, VLA_Query, 
    PublishSemanticMarker, TagSearchSweep, GenerateSiteReport, CheckBattery, 
    HazardFoundOnBlackboard, IdleRecharge
)

def camera_worker():
    """ Runs continuously in the background, updating the blackboard. """
    print("[Camera Thread] Starting video stream...")
    # blackboard = py_trees.blackboard.Client(name="CameraThread")
    # blackboard.register_key(key="tag_x", access=py_trees.common.Access.WRITE)
    # blackboard.register_key(key="tag_y", access=py_trees.common.Access.WRITE)
    
    # cap = cv2.VideoCapture("http://<robot-ip>:8080/stream") # Or your Twin's video stream
    
    # while True:
    #     ret, frame = cap.read()
    #     if not ret: continue
        
    #     # Do your robust CLAHE + Unsharp Masking Aruco detection here...
    #     corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters)
        
    #     if ids is not None:
    #         # Calculate center of the tag
    #         cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
    #         cY = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
    #         blackboard.tag_x = cX
    #         blackboard.tag_y = cY
    #     else:
    #         # Optional: clear the blackboard or set to None so the Tree knows it's lost
    #         pass 

def lidar_mqtt_callback(message):
    """ Fired whenever a new Lidar scan arrives via MQTT. """
    # blackboard = py_trees.blackboard.Client(name="LidarThread")
    # blackboard.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
    
    # Analyze the point cloud data for drop-offs or obstacles
    # if hazard_found:
    #     blackboard.hazard_detected = True
    # else:
    #     blackboard.hazard_detected = False
    pass

def create_tree(ugv_twin):
    """ Builds the exact tree defined in Section 6 'Expanded Behavior Tree' """
    # Root: A Parallel policy that runs both subtrees
    root = py_trees.composites.Parallel(
        name="Root (Parallel: Policy)", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    # --- Subtree 1: Safety Monitor ---
    safety_monitor = py_trees.composites.Sequence("Subtree: Safety Monitor", memory=False)
    lidar_check = CheckLidarObstacle(name="CheckLidarObstacle()", ugv_twin=ugv_twin)
    vision_check = CheckVisionDropoff(name="CheckVisionDropoff()", ugv_twin=ugv_twin)
    battery_check = CheckBattery(name="CheckBattery()")
    safety_monitor.add_children([lidar_check, vision_check, battery_check])
    
    # --- Subtree 2: Mission Logic ---
    mission_logic = py_trees.composites.Selector(name="Subtree: Mission Logic", memory=False)
    
    # 2A. Emergency Handling
    emergency_handling = py_trees.composites.Sequence("Sequence: Emergency Handling", memory=False)
    hazard_condition = HazardFoundOnBlackboard("Condition: HazardFoundOnBlackboard")
    emergency_action = StopAndStrobe("Action: StopAndStrobe", ugv_twin=ugv_twin)
    emergency_handling.add_children([hazard_condition, emergency_action])
    
    # 2B. Main Inspection Tour
    inspection_tour = py_trees.composites.Sequence("Sequence: Main Inspection Tour", memory=True)
    nav_zone = MoveToProximity("Action: NavigateToZone", ugv_twin=ugv_twin)
    
    # 2B-1. Approaching & Tracking Parallel
    approach_track = py_trees.composites.Parallel(
        name="Parallel: Approaching & Tracking",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    drive_tag = MoveBaseToTag("Action: DriveToBaseTag", ugv_twin=ugv_twin)
    pid_track = TrackTagInCenter("Action: PID_TrackTagWithHead", ugv_twin=ugv_twin)
    approach_track.add_children([drive_tag, pid_track])
    
    # 2B-2. VLA Survey
    vla_survey = VLA_Query("Action: Conduct_VLA_Survey", prompt="Are there tripping hazards?")
    map_marker = PublishSemanticMarker("Action: Publish_Map_Marker", ugv_twin=ugv_twin)
    report_gen = GenerateSiteReport("Generate Site Report Log")
    
    vla_survey_seq = py_trees.composites.Sequence("Semantic Assessment", memory=True)
    vla_survey_seq.add_children([vla_survey, map_marker, report_gen])
    
    inspection_tour.add_children([nav_zone, approach_track, vla_survey_seq])
    
    # 2C. Idle / Recharge
    idle_recharge = IdleRecharge("Action: Idle/Recharge")
    
    # Assemble Mission Logic Selector
    mission_logic.add_children([emergency_handling, inspection_tour, idle_recharge])
    
    # Assemble Root
    root.add_children([safety_monitor, mission_logic])
    
    return py_trees.trees.BehaviourTree(root)

if __name__ == '__main__':
    print("--- Connecting to Real UGV ---")
    
    # 1. Connect to Hardware
    # ugv_beast = UGVTwin(uuid="your-uuid", token="your-token")
    # ugv_beast.connect()
    ugv_beast = None # Placeholder
    
    # 2. Start Sensor Threads
    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()
    
    # ugv_beast.on_lidar_message = lidar_mqtt_callback
    
    # 3. Setup Tree
    tree = create_tree(ugv_beast)
    tree.setup(timeout=15)
    
    print("--- Commencing Mission Loop ---")
    try:
        while True:
            # The tree evaluates the blackboard and triggers actuator commands
            tree.tick()
            
            # Tick at 10 Hz
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Mission aborted by user.")
        # ugv_beast.stop_all()