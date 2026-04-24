import unittest
from unittest.mock import MagicMock
import py_trees

# Import the behaviors that interact with the SLAM/VLA Map UI SDK
from behaviors import PublishSemanticMarker

class TestSDKMapping(unittest.TestCase):
    """
    Unit tests for the RTAB-Map and VLA SDK mapping integration logic.
    NOTE: These tests will fail until you uncomment the `self.ugv...` lines in behaviors.py
    and implement the corresponding functions in your UGV SDK wrapper.
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.mock_ugv = MagicMock()
        
        # Pre-seed the blackboard with the expected VLA response string
        self.blackboard = py_trees.blackboard.Client(name="TestClient")
        self.blackboard.register_key(key="vla_response", access=py_trees.common.Access.WRITE)
        self.blackboard.vla_response = "Identified object: Hazard model - PVC Pipe"

    def test_publish_semantic_marker(self):
        """Tests if the node pulls RTAB-Map pose and sends a UI marker payload down the SDK."""
        
        # 1. Mock the expected output from your SDK implementation of get_rtabmap_pose
        self.mock_ugv.get_rtabmap_pose.return_value = {"x": 5.0, "y": 2.0, "z": 0.0}
        
        node = PublishSemanticMarker("TestMarker", self.mock_ugv)
        node.setup()
        node.tick()
        
        # 2. Verify the pose was requested from the Edge SLAM system
        self.mock_ugv.get_rtabmap_pose.assert_called_once()
        
        # 3. Verify the SDK dispatched the Map Marker payload
        self.mock_ugv.send_map_marker.assert_called_once()
        
        # 4. Inspect the generated payload argument
        args, kwargs = self.mock_ugv.send_map_marker.call_args
        payload = args[0]
        
        self.assertIn("label", payload)
        self.assertEqual(payload["x"], 5.0)
        self.assertEqual(payload["y"], 2.0)

if __name__ == '__main__':
    unittest.main()
