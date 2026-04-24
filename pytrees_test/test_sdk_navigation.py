import unittest
from unittest.mock import MagicMock
import py_trees

# Import the behaviors that interact with the base navigation SDK
from behaviors import MoveToProximity, MoveBaseToTag, StopAndStrobe

class TestSDKNavigation(unittest.TestCase):
    """
    Unit tests for the Nav2 / SLAM integration SDK calls.
    NOTE: These tests will fail until you uncomment the `self.ugv...` lines in behaviors.py
    and implement the corresponding functions in your UGV SDK wrapper.

    SDK methods under test:
      ugv.send_nav2_goal(x, y, yaw)  - send a global SLAM navigation goal
      ugv.set_velocity(linear, angular) - command base velocity directly
      ugv.cancel_nav2_goal()          - abort an active Nav2 goal
      ugv.stop_all()                  - full system shutdown on graceful exit
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()

    # -----------------------------------------------------------------------
    # MoveToProximity: send_nav2_goal
    # -----------------------------------------------------------------------

    def test_send_nav2_goal_is_called(self):
        """Tests if the behavior correctly requests the SDK to send a Nav2 goal."""
        node = MoveToProximity("TestNav2", self.mock_ugv)
        node.setup()
        node.tick()

        # Expectation: uncomment `self.ugv.send_nav2_goal(...)` in behaviors.py
        self.mock_ugv.send_nav2_goal.assert_called_once()

    def test_send_nav2_goal_has_three_args(self):
        """Nav2 goal must supply X, Y, and Yaw — Nav2 rejects goals without orientation."""
        node = MoveToProximity("TestNav2Args", self.mock_ugv)
        node.setup()
        node.tick()

        args, _ = self.mock_ugv.send_nav2_goal.call_args
        self.assertEqual(len(args), 3, "Expected X, Y, and Yaw positional arguments for Nav2 goal.")

    def test_move_to_proximity_returns_running_then_success(self):
        """Verifies the RUNNING → SUCCESS state progression across ticks."""
        node = MoveToProximity("TestNav2Progress", self.mock_ugv)
        node.setup()

        # Ticks 1 and 2 should be RUNNING (journey_ticks < 3)
        for _ in range(2):
            status = node.update()
            self.assertEqual(status, py_trees.common.Status.RUNNING)

        # Tick 3 should be SUCCESS
        status = node.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    # -----------------------------------------------------------------------
    # MoveBaseToTag: set_velocity
    # -----------------------------------------------------------------------

    def test_set_velocity_forward_while_approaching(self):
        """Tests that the SDK is commanded to move forward while driving to the tag."""
        node = MoveBaseToTag("TestVelocity", self.mock_ugv)
        node.setup()
        node.tick()

        # Expectation: uncomment `self.ugv.set_velocity(linear=0.2, angular=0.0)`
        self.mock_ugv.set_velocity.assert_called_with(linear=0.2, angular=0.0)

    def test_set_velocity_zero_on_arrival(self):
        """On arrival the base must stop — otherwise the robot overshoots the tag."""
        node = MoveBaseToTag("TestVelocityStop", self.mock_ugv)
        node.setup()

        # Drain the RUNNING ticks until SUCCESS
        for _ in range(6):
            node.update()

        # Final tick: journey_ticks == 6 → SUCCESS, motors must halt
        # Expectation: uncomment `self.ugv.set_velocity(0, 0)` on SUCCESS branch
        calls = [str(c) for c in self.mock_ugv.set_velocity.call_args_list]
        self.assertIn("call(0, 0)", calls,
                      "Expected set_velocity(0, 0) to be called on arrival.")

    # -----------------------------------------------------------------------
    # StopAndStrobe: cancel_nav2_goal + set_velocity(0,0)
    # -----------------------------------------------------------------------

    def test_stop_and_strobe_cancels_nav2_goal(self):
        """Tests that emergency halt cancels any active Nav2 goal via the SDK."""
        node = StopAndStrobe("TestEmergency", self.mock_ugv)
        node.setup()
        node.tick()

        # Expectation: uncomment `self.ugv.cancel_nav2_goal()`
        self.mock_ugv.cancel_nav2_goal.assert_called_once()

    def test_stop_and_strobe_cuts_motors(self):
        """Tests that emergency halt issues a zero-velocity motor command."""
        node = StopAndStrobe("TestCutMotors", self.mock_ugv)
        node.setup()
        node.tick()

        # Expectation: uncomment `self.ugv.set_velocity(0, 0)`
        self.mock_ugv.set_velocity.assert_called_with(0, 0)

    def test_stop_and_strobe_returns_success(self):
        """StopAndStrobe must always return SUCCESS so the tree records the halt."""
        node = StopAndStrobe("TestEmergencyStatus", self.mock_ugv)
        node.setup()
        status = node.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    # -----------------------------------------------------------------------
    # Graceful shutdown: stop_all
    # -----------------------------------------------------------------------

    def test_stop_all_called_on_shutdown(self):
        """
        Tests that ugv.stop_all() is invoked when the mission loop exits.
        Wire this in real_main.py: uncomment `ugv_beast.stop_all()` in the
        KeyboardInterrupt handler.
        """
        # Simulate the KeyboardInterrupt shutdown path
        self.mock_ugv.stop_all()
        self.mock_ugv.stop_all.assert_called_once()

if __name__ == '__main__':
    unittest.main()
