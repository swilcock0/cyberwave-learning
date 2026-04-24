import py_trees
import time
from behaviors import (
    TrackTagInCenter, MoveToProximity, MoveBaseToTag, 
    CheckLidarObstacle, CheckVisionDropoff, StopAndStrobe, PanTilt_Scan, VLA_Query, 
    PublishSemanticMarker, TagSearchSweep, GenerateSiteReport, CheckBattery, 
    HazardFoundOnBlackboard, IdleRecharge
)

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
    # Mock UGV client just so the script can compile and run locally for testing
    class MockUGVClient:
        pass
    ugv_beast = MockUGVClient()
    
    print("--- Initializing Site Foreman Demo ---")

    # We need to setup a 'Vision' system that writes data to the Blackboard
    vision_publisher = py_trees.blackboard.Client(name="VisionSystem")
    vision_publisher.register_key(key="tag_x", access=py_trees.common.Access.WRITE)
    vision_publisher.register_key(key="tag_y", access=py_trees.common.Access.WRITE)

    # Let's say our vision system suddenly detects a tag in the bottom right corner
    vision_publisher.tag_x = 550.0  # Far right (assuming center is 320)
    vision_publisher.tag_y = 400.0  # Far down (assuming center is 240)

    # Execution
    tree = create_tree(ugv_beast)
    tree.setup(timeout=15)

    py_trees.logging.level = py_trees.logging.Level.INFO

    # Simulate the passage of time (Game Loop)
    for i in range(1, 14):
        print(f"\n--- Tick {i} ---")
        
        # Simulate Vision Dropout
        if i == 5:
            print("\n*** SIMULATOR: Tag occluded! (Vision Dropout) ***")
            vision_publisher.tag_x = None
            vision_publisher.tag_y = None
        elif i == 7:
            print("\n*** SIMULATOR: Tag re-acquired! ***")
            vision_publisher.tag_x = 350.0
            vision_publisher.tag_y = 260.0
        
        # In a real robot, 'TrackTagInCenter' would move servos, bringing the tag closer to 320/240
        # Here we manually mock the camera turning and updating vision data
        if getattr(vision_publisher, "tag_x", None) is not None:
            if vision_publisher.tag_x > 320:
                vision_publisher.tag_x -= 30  # Simulated camera pan turning left
            if vision_publisher.tag_y > 240:
                vision_publisher.tag_y -= 20  # Simulated camera tilt pitching up
            
        tree.tick()
        
        # In tick 9, let's simulate a hazardous drop-off that the planar Lidar misses but the VLA spots
        if i == 9:
            hazard_publisher = py_trees.blackboard.Client(name="HazardUpdater")
            hazard_publisher.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
            hazard_publisher.hazard_detected = True
            print("\n*** SIMULATOR: VLA Vision suddenly detects a 3-foot floor drop-off (unseen by planar Lidar)! ***")
            
        time.sleep(0.5)
        
    print("\n--- Mission Simulation Finished ---")
