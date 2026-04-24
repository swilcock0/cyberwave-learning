import unittest
from unittest.mock import MagicMock
import py_trees

# Import the behaviors that interact with the PTZ and Vision SDKs
from behaviors import TrackTagInCenter

class TestSDKPTZVision(unittest.TestCase):
    """
    Unit tests for the PTZ tracking loop SDK operations.
    NOTE: These tests will fail until you uncomment the `self.ugv...` lines in behaviors.py
    and implement the corresponding functions in your UGV SDK wrapper.

    SDK methods under test:
      ugv.set_pantilt_offsets(pan, tilt)  - command PTZ servo adjustment
      ugv.get_angular_velocity()          - read current base yaw rate (rad/s) for
                                            tag-dropout coast compensation
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()
        self.mock_ugv.get_angular_velocity.return_value = 0.0

        self.blackboard = py_trees.blackboard.Client(name="TestClient")
        self.blackboard.register_key(key="tag_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="tag_y", access=py_trees.common.Access.WRITE)

    # -----------------------------------------------------------------------
    # set_pantilt_offsets: active tracking
    # -----------------------------------------------------------------------

    def test_ptz_tracking_sdk_command_called(self):
        """Tests that the PID result is forwarded to the SDK PTZ actuator."""
        self.blackboard.tag_x = 400.0  # +80px error right
        self.blackboard.tag_y = 300.0  # +60px error down

        node = TrackTagInCenter("TestPTZ", self.mock_ugv)
        node.setup()
        node.tick()

        # Expectation: uncomment `self.ugv.set_pantilt_offsets(pan_adjustment, tilt_adjustment)`
        self.mock_ugv.set_pantilt_offsets.assert_called_once()

    def test_ptz_tracking_output_is_float(self):
        """Pan and tilt values sent to the SDK must be floats (servo API rejects ints)."""
        self.blackboard.tag_x = 400.0
        self.blackboard.tag_y = 300.0

        node = TrackTagInCenter("TestPTZFloat", self.mock_ugv)
        node.setup()
        node.tick()

        args, _ = self.mock_ugv.set_pantilt_offsets.call_args
        pan, tilt = args
        self.assertIsInstance(pan, float)
        self.assertIsInstance(tilt, float)

    def test_ptz_tracking_pan_direction(self):
        """Tag right of center → negative pan error → SDK receives a leftward pan."""
        # Tag is right of center (320), so error_x = 320 - 400 = -80 → pan left
        self.blackboard.tag_x = 400.0
        self.blackboard.tag_y = 240.0  # vertically centred

        node = TrackTagInCenter("TestPTZDir", self.mock_ugv)
        node.setup()
        node.tick()

        args, _ = self.mock_ugv.set_pantilt_offsets.call_args
        pan, _ = args
        self.assertLess(pan, 0.0, "Tag right of center should produce a negative (left) pan command.")

    def test_ptz_not_called_when_tag_centered(self):
        """When the tag is already centered, the behavior returns SUCCESS without adjusting."""
        self.blackboard.tag_x = 320.0  # exact center
        self.blackboard.tag_y = 240.0

        node = TrackTagInCenter("TestPTZCentered", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)
        self.mock_ugv.set_pantilt_offsets.assert_not_called()

    # -----------------------------------------------------------------------
    # get_angular_velocity: dropout coast compensation
    # -----------------------------------------------------------------------

    def test_get_angular_velocity_called_on_dropout(self):
        """
        When the tag drops out, the behavior must call ugv.get_angular_velocity()
        to compensate for base rotation.
        Expectation: uncomment `self.ugv.get_angular_velocity()` in behaviors.py
        """
        # Seed a last-known position, then simulate a tag dropout
        self.blackboard.tag_x = 350.0
        self.blackboard.tag_y = 250.0

        node = TrackTagInCenter("TestDropout", self.mock_ugv)
        node.setup()
        node.update()  # tick 1: tag visible, last_known set

        # Drop the tag
        self.blackboard.tag_x = None
        self.blackboard.tag_y = None

        node.update()  # tick 2: tag lost → should call get_angular_velocity

        self.mock_ugv.get_angular_velocity.assert_called()

    def test_angular_velocity_shifts_estimated_x(self):
        """
        A positive angular velocity (turning left) should shift the estimated
        tag X coordinate to the right (positive pixel shift).
        """
        self.blackboard.tag_x = 350.0
        self.blackboard.tag_y = 250.0
        self.mock_ugv.get_angular_velocity.return_value = 1.0  # turning left at 1 rad/s

        node = TrackTagInCenter("TestDropoutShift", self.mock_ugv)
        node.setup()
        node.update()  # tick 1: tag visible

        initial_x = node.last_known_x

        self.blackboard.tag_x = None
        self.blackboard.tag_y = None
        node.update()  # tick 2: tag lost, shift applied

        # pixel_shift = -(1.0 * 500.0 * 0.1) = -50 → last_known_x should decrease by 50
        self.assertAlmostEqual(node.last_known_x, initial_x - 50.0, places=1)

    def test_failure_when_tag_never_seen(self):
        """If the tag has never been seen there is no fallback — node must return FAILURE."""
        self.blackboard.tag_x = None
        self.blackboard.tag_y = None

        node = TrackTagInCenter("TestNeverSeen", self.mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.FAILURE)

if __name__ == '__main__':
    unittest.main()
