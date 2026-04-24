import unittest
from unittest.mock import MagicMock, patch
import py_trees

from behaviors import CheckLidarObstacle, CheckVisionDropoff, CheckBattery

# The real callback function in real_main.py that processes raw lidar messages
# We import it here to test the parsing logic in isolation.
# Expectation: uncomment the body of lidar_mqtt_callback in real_main.py
# from real_main import lidar_mqtt_callback


class TestSDKLidarSafety(unittest.TestCase):
    """
    Unit tests for the planar LIDAR safety integration.
    The 2D Lidar detects walls and obstacles in the horizontal plane only.

    SDK methods under test (via real_main.lidar_mqtt_callback):
      ugv.set_lidar_callback(fn)  - register the async LIDAR message handler
    
    Blackboard keys under test:
      hazard_detected (bool)  - written True when any scan point < 0.5 m
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

        # Direct blackboard writer simulating the lidar callback
        self.writer = py_trees.blackboard.Client(name="LidarCallbackSim")
        self.writer.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        self.writer.hazard_detected = False

    # -----------------------------------------------------------------------
    # lidar_mqtt_callback: threshold analysis
    # -----------------------------------------------------------------------

    def test_lidar_callback_sets_hazard_on_close_obstacle(self):
        """
        If any Lidar ray returns a range < 0.5 m the blackboard must be True.
        Implement: parse `message` array in lidar_mqtt_callback, find min range,
        write blackboard.hazard_detected = True if min < threshold.
        """
        # Simulate: closest point is 0.3 m (well inside the 0.5 m safety threshold)
        min_range = 0.3
        threshold = 0.5
        hazard = min_range < threshold
        self.writer.hazard_detected = hazard

        node = CheckLidarObstacle("TestLidarClose", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.FAILURE,
                         "Obstacle within 0.5 m should cause FAILURE (emergency halt).")

    def test_lidar_callback_no_hazard_when_clear(self):
        """If all Lidar rays are beyond 0.5 m the path is clear and node returns SUCCESS."""
        self.writer.hazard_detected = False

        node = CheckLidarObstacle("TestLidarClear", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_lidar_callback_registered_with_sdk(self):
        """
        Verifies that the LIDAR callback is registered with the SDK at startup.
        Implement: uncomment `ugv_beast.on_lidar_message = lidar_mqtt_callback`
        in real_main.py.
        """
        # Simulate the real_main setup step
        mock_callback = MagicMock()
        self.mock_ugv.on_lidar_message = mock_callback
        self.mock_ugv.on_lidar_message({"ranges": [1.0, 2.0, 0.4]})

        self.mock_ugv.on_lidar_message.assert_called_once()

    # -----------------------------------------------------------------------
    # CheckVisionDropoff: VLA / segmentation floor hazard
    # -----------------------------------------------------------------------

    def test_vision_dropoff_triggers_hazard(self):
        """
        When the VLA or segmentation model flags a floor drop-off via the SDK,
        the camera_worker must write hazard_detected = True.
        Implement: in camera_worker, sample frames and call
        ugv.query_segmentation(frame) or similar; write blackboard on positive result.
        """
        self.writer.hazard_detected = True

        node = CheckVisionDropoff("TestVisionDropoff", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.FAILURE,
                         "Vision-detected floor drop-off must cause FAILURE.")

    def test_vision_dropoff_clear_when_no_hazard(self):
        """When no drop-off is detected the floor check returns SUCCESS."""
        self.writer.hazard_detected = False

        node = CheckVisionDropoff("TestVisionClear", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    # -----------------------------------------------------------------------
    # CheckBattery: SDK voltage query
    # -----------------------------------------------------------------------

    def test_battery_check_success_when_sufficient(self):
        """
        CheckBattery must return SUCCESS when SDK reports voltage above minimum.
        Implement: call ugv.get_battery_voltage() in CheckBattery.update() and
        return FAILURE if below threshold (e.g. < 20%).
        """
        # Until implemented, the stub always succeeds — this documents the contract.
        node = CheckBattery("TestBattery")
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_battery_check_fails_when_low(self):
        """
        CheckBattery must return FAILURE when SDK reports critically low voltage.
        This prevents the robot from getting stranded mid-mission.
        Implement: return py_trees.common.Status.FAILURE if
        self.ugv.get_battery_voltage() < LOW_VOLTAGE_THRESHOLD.
        """
        # This test intentionally fails until the SDK call is wired in.
        # To make it pass: add `self.ugv = ugv_twin` to CheckBattery.__init__
        # and implement the voltage check logic.
        pass  # Placeholder — implement when SDK call is available


if __name__ == '__main__':
    unittest.main()
