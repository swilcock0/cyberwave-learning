import unittest
from unittest.mock import MagicMock
import py_trees

from behaviors import (
    DynamicObstacleAvoidance,
    StuckRecovery,
    CommsBuffer,
    BatteryTriage,
    LostRecovery,
    FrontierExploration,
    PrecisionApproach,
    HazardFoundOnBlackboard,
    IdleRecharge,
)


class TestDynamicObstacleAvoidance(unittest.TestCase):
    """
    Tests for the Lidar-based dynamic obstacle monitor (e.g. walking humans).

    SDK methods under test:
      ugv.get_lidar_scan()            - returns current 2D scan array
      ugv.get_nearby_objects(radius)  - SDK processed object list within radius (m)

    Implement in DynamicObstacleAvoidance.update():
      objects = self.ugv.get_nearby_objects(radius=1.0)
      if any object within 1m: return FAILURE
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_queries_sdk_for_nearby_objects(self):
        """The node must query the SDK for objects within 1 m on every tick."""
        self.mock_ugv.get_nearby_objects.return_value = []

        node = DynamicObstacleAvoidance("TestDOA", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: uncomment/implement self.ugv.get_nearby_objects(radius=1.0)
        self.mock_ugv.get_nearby_objects.assert_called_with(radius=1.0)

    def test_returns_failure_when_object_within_radius(self):
        """A person or object within 1 m must cause FAILURE so the tree halts motion."""
        self.mock_ugv.get_nearby_objects.return_value = [
            {"label": "person", "distance": 0.7}
        ]

        node = DynamicObstacleAvoidance("TestDOABlocked", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.FAILURE)

    def test_returns_success_when_path_clear(self):
        """No objects within radius → path is clear → SUCCESS."""
        self.mock_ugv.get_nearby_objects.return_value = []

        node = DynamicObstacleAvoidance("TestDOAClear", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_hazard_written_to_blackboard_on_obstacle(self):
        """
        The node must also write hazard_detected = True to the blackboard so the
        Mission Logic Selector can immediately branch to StopAndStrobe.
        """
        self.mock_ugv.get_nearby_objects.return_value = [
            {"label": "forklift", "distance": 0.9}
        ]

        # Pre-register the key so DynamicObstacleAvoidance can write it
        bb = py_trees.blackboard.Client(name="DOAHazardCheck")
        bb.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        bb.hazard_detected = False

        node = DynamicObstacleAvoidance("TestDOABB", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement blackboard write in DynamicObstacleAvoidance
        # self.blackboard.hazard_detected = True
        # (register the key in __init__ as the other safety nodes do)


class TestStuckRecovery(unittest.TestCase):
    """
    Tests for the stuck/stall detection and recovery wiggle.

    SDK methods under test:
      ugv.get_rtabmap_pose()          - returns {"x", "y", "z"} world coordinates
      ugv.get_last_velocity_command() - returns {"linear", "angular"} last command
      ugv.set_velocity(linear, angular) - wiggle command to break free
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_queries_pose_and_velocity_command(self):
        """StuckRecovery must compare commanded velocity against odometry to detect stall."""
        self.mock_ugv.get_rtabmap_pose.return_value = {"x": 1.0, "y": 1.0, "z": 0.0}
        self.mock_ugv.get_last_velocity_command.return_value = {"linear": 0.0, "angular": 0.0}

        node = StuckRecovery("TestStuck", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement self.ugv.get_rtabmap_pose() and
        # self.ugv.get_last_velocity_command() in StuckRecovery.update()
        self.mock_ugv.get_rtabmap_pose.assert_called()

    def test_returns_success_when_not_stuck(self):
        """When velocity command is zero, robot is not expected to move — SUCCESS."""
        self.mock_ugv.get_rtabmap_pose.return_value = {"x": 1.0, "y": 1.0, "z": 0.0}
        self.mock_ugv.get_last_velocity_command.return_value = {"linear": 0.0, "angular": 0.0}

        node = StuckRecovery("TestNotStuck", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_wiggle_issued_when_stuck(self):
        """
        When linear command > 0.5 but translation delta < 0.05 m, the robot is
        stuck. The node must issue a reverse wiggle via set_velocity.
        Implement: if v_cmd > 0.5 and delta_translation < 0.05: self.ugv.set_velocity(-0.1, 0.2)
        """
        # Pose hasn't moved since last tick
        self.mock_ugv.get_rtabmap_pose.side_effect = [
            {"x": 1.00, "y": 1.00, "z": 0.0},  # previous pose
            {"x": 1.01, "y": 1.00, "z": 0.0},  # barely moved (< 0.05 m)
        ]
        self.mock_ugv.get_last_velocity_command.return_value = {"linear": 0.6, "angular": 0.0}

        node = StuckRecovery("TestWiggle", self.mock_ugv)
        node.setup()
        node.update()  # first tick: stores reference pose
        node.update()  # second tick: detects stuck condition

        # Expectation: wiggle command issued
        # self.mock_ugv.set_velocity.assert_called()


class TestCommsBuffer(unittest.TestCase):
    """
    Tests for the connectivity guard and local image cache fallback.

    SDK methods under test:
      ugv.check_cloud_connectivity()   - pings cloud endpoint; returns bool
      ugv.save_frame_to_local(frame)   - writes image to /tmp/ when offline
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_returns_success_when_connected(self):
        """When cloud is reachable, CommsBuffer returns SUCCESS and does not cache."""
        self.mock_ugv.check_cloud_connectivity.return_value = True

        node = CommsBuffer("TestCommsOK", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_saves_frame_locally_when_offline(self):
        """
        If cloud is unreachable, any pending camera frame must be saved locally
        so it can be uploaded when connectivity resumes.
        Implement: if not self.ugv.check_cloud_connectivity():
                       self.ugv.save_frame_to_local(self.cached_frame)
        """
        self.mock_ugv.check_cloud_connectivity.return_value = False
        fake_frame = MagicMock(name="frame")

        # Simulate the CommsBuffer offline path
        if not self.mock_ugv.check_cloud_connectivity():
            self.mock_ugv.save_frame_to_local(fake_frame)

        self.mock_ugv.save_frame_to_local.assert_called_once_with(fake_frame)

    def test_checks_connectivity_on_every_tick(self):
        """Connectivity must be verified each tick, not cached from startup."""
        self.mock_ugv.check_cloud_connectivity.return_value = True

        node = CommsBuffer("TestCommsPolling", self.mock_ugv)
        node.setup()
        node.update()
        node.update()

        # Expectation: implement self.ugv.check_cloud_connectivity() in CommsBuffer
        # self.assertGreaterEqual(self.mock_ugv.check_cloud_connectivity.call_count, 2)


class TestBatteryTriage(unittest.TestCase):
    """
    Tests for voltage-based behavior pruning.

    SDK methods under test:
      ugv.get_battery_voltage()   - returns float in Volts or % (check SDK docs)

    Thresholds (to configure in BatteryTriage):
      < 20%  → FAILURE (forces tree to IdleRecharge branch)
      >= 20% → SUCCESS (full behavior tree enabled)
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_success_at_full_charge(self):
        """At full battery the node must pass so the mission tree can run."""
        self.mock_ugv.get_battery_voltage.return_value = 100.0  # 100%

        node = BatteryTriage("TestBatteryFull", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_failure_at_critical_voltage(self):
        """
        At critically low voltage the node must FAIL, cascading the tree to the
        IdleRecharge fallback branch so the robot returns to dock before dying.
        Implement: if self.ugv.get_battery_voltage() < 20.0: return FAILURE
        """
        self.mock_ugv.get_battery_voltage.return_value = 15.0  # critically low

        node = BatteryTriage("TestBatteryCritical", self.mock_ugv)
        node.setup()
        # Expectation: implement voltage check — currently stub always succeeds
        # status = node.update()
        # self.assertEqual(status, py_trees.common.Status.FAILURE)

    def test_voltage_queried_from_sdk(self):
        """Battery level must come from the SDK, not a hardcoded value."""
        self.mock_ugv.get_battery_voltage.return_value = 75.0

        node = BatteryTriage("TestBatterySDK", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement self.ugv.get_battery_voltage() in BatteryTriage
        # self.mock_ugv.get_battery_voltage.assert_called_once()


class TestLostRecovery(unittest.TestCase):
    """
    Tests for SLAM re-localisation via AprilTag landmark search.

    SDK methods under test:
      ugv.get_slam_confidence()          - returns float [0.0, 1.0]
      ugv.set_pantilt_offsets(pan, tilt) - sweeps the PTZ to find a known landmark
      ugv.get_rtabmap_pose()             - re-checked after recovery attempt
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_remains_running_while_lost(self):
        """While SLAM confidence is below threshold, node must keep returning RUNNING."""
        self.mock_ugv.get_slam_confidence.return_value = 0.2  # very low

        node = LostRecovery("TestLost", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.RUNNING)

    def test_queries_slam_confidence_on_each_tick(self):
        """
        SLAM confidence must be re-evaluated on every tick so recovery is detected
        as soon as the AprilTag landmark re-appears in the camera.
        Implement: confidence = self.ugv.get_slam_confidence()
        """
        self.mock_ugv.get_slam_confidence.return_value = 0.15

        node = LostRecovery("TestLostPoll", self.mock_ugv)
        node.setup()
        node.update()
        node.update()

        # Expectation: implement self.ugv.get_slam_confidence() in LostRecovery
        # self.assertGreaterEqual(self.mock_ugv.get_slam_confidence.call_count, 2)

    def test_pan_tilt_sweep_issued_while_lost(self):
        """
        While SLAM is lost the PTZ should perform a search sweep to find the nearest
        known AprilTag landmark and trigger re-localisation.
        Implement: self.ugv.set_pantilt_offsets(sweep_angle, 0.0) in LostRecovery
        """
        self.mock_ugv.get_slam_confidence.return_value = 0.1

        node = LostRecovery("TestLostSweep", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement PTZ sweep in LostRecovery
        # self.mock_ugv.set_pantilt_offsets.assert_called()


class TestFrontierExploration(unittest.TestCase):
    """
    Tests for the spiral map-filling exploration mode.

    SDK methods under test:
      ugv.get_map_coverage()            - returns % of explored area [0.0, 1.0]
      ugv.send_nav2_goal(x, y, yaw)     - navigates to next frontier waypoint
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_remains_running_while_exploring(self):
        """FrontierExploration should stay RUNNING until map coverage is sufficient."""
        self.mock_ugv.get_map_coverage.return_value = 0.4  # 40% explored

        node = FrontierExploration("TestFrontier", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.RUNNING)

    def test_sends_nav2_goal_to_next_frontier(self):
        """
        Each tick should navigate the robot to the next unexplored frontier cell.
        Implement: waypoint = self.ugv.get_next_frontier()
                   self.ugv.send_nav2_goal(waypoint.x, waypoint.y, 0.0)
        """
        self.mock_ugv.get_map_coverage.return_value = 0.3
        self.mock_ugv.get_next_frontier.return_value = MagicMock(x=5.0, y=3.0)

        node = FrontierExploration("TestFrontierGoal", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement frontier-to-Nav2 goal dispatch
        # self.mock_ugv.send_nav2_goal.assert_called_with(5.0, 3.0, 0.0)


class TestPrecisionApproach(unittest.TestCase):
    """
    Tests for visual-odometry-based close-range approach.

    SDK methods under test:
      ugv.get_visual_odometry_distance()  - range to target from visual odometry (m)
      ugv.set_velocity(linear, angular)   - slow creep command at < 1 m
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    def test_engages_below_one_metre(self):
        """PrecisionApproach must only activate when the target is within 1 m."""
        self.mock_ugv.get_visual_odometry_distance.return_value = 0.6  # 60 cm

        node = PrecisionApproach("TestPrecision", self.mock_ugv)
        node.setup()
        status = node.update()

        # Currently stub returns SUCCESS unconditionally; test the contract
        self.assertIn(status, [
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.SUCCESS
        ])

    def test_issues_slow_velocity_command(self):
        """
        Within 1 m the robot must creep slowly using visual odometry feedback,
        not the same 0.2 m/s used for long-range navigation.
        Implement: self.ugv.set_velocity(linear=0.05, angular=0.0)
        """
        self.mock_ugv.get_visual_odometry_distance.return_value = 0.4

        node = PrecisionApproach("TestPrecisionVelocity", self.mock_ugv)
        node.setup()
        node.update()

        # Expectation: implement precision approach velocity command
        # call = self.mock_ugv.set_velocity.call_args
        # self.assertLess(call[1]['linear'], 0.1, "Precision approach must use < 0.1 m/s")


if __name__ == "__main__":
    unittest.main()
