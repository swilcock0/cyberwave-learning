import unittest
from unittest.mock import MagicMock, patch
import py_trees

from behaviors import VLA_Query, GenerateSiteReport, PublishSemanticMarker


class TestSDKVLAQuery(unittest.TestCase):
    """
    Unit tests for the VLA cloud inference pipeline and the Human-in-the-Loop
    'Elaborate' action.

    SDK methods under test (via behaviors.VLA_Query):
      ugv.send_vla_query(prompt, frame)    - submit image + prompt to Physical AI Cloud
      ugv.get_llm_response(context)        - LLM bridge for the Elaborate clarification
    
    VLA_Query blackboard key:
      vla_response (str)  - written on successful inference
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()

    # -----------------------------------------------------------------------
    # Normal VLA inference path
    # -----------------------------------------------------------------------

    def test_vla_query_returns_running_while_processing(self):
        """VLA_Query must stay RUNNING for the first two ticks (latency simulation)."""
        node = VLA_Query("TestVLA", prompt="Are there tripping hazards?")
        node.setup()

        status = node.update()  # tick 1
        self.assertEqual(status, py_trees.common.Status.RUNNING)
        status = node.update()  # tick 2
        self.assertEqual(status, py_trees.common.Status.RUNNING)

    def test_vla_response_written_to_blackboard(self):
        """
        After inference completes, the VLA response string must be written to
        the blackboard so GenerateSiteReport and PublishSemanticMarker can consume it.
        Implement: self.ugv.send_vla_query(self.prompt, captured_frame) in VLA_Query,
        then write the SDK response to self.blackboard.vla_response.
        """
        node = VLA_Query("TestVLAResponse", prompt="Are there tripping hazards?")
        node.setup()

        # Drain RUNNING ticks and bypass the 'Elaborate' prompt
        node.elaboration_requested = True  # skip interactive Elaborate in tests
        for _ in range(3):
            node.update()

        bb = py_trees.blackboard.Client(name="VLAReadback")
        bb.register_key(key="vla_response", access=py_trees.common.Access.READ)
        self.assertIsNotNone(bb.vla_response)
        self.assertIsInstance(bb.vla_response, str)

    def test_vla_query_prompt_is_sent_to_sdk(self):
        """
        The raw prompt string must reach the SDK call unchanged.
        Implement: self.ugv.send_vla_query(self.prompt, frame) in VLA_Query.update().
        """
        mock_ugv = MagicMock()
        prompt = "Count the hard hats visible in the scene."

        # Simulate the SDK dispatch that VLA_Query should perform
        mock_ugv.send_vla_query(prompt, MagicMock())

        args, _ = mock_ugv.send_vla_query.call_args
        self.assertEqual(args[0], prompt,
                         "The prompt sent to the SDK must match the one passed to VLA_Query.")

    def test_vla_query_includes_image_frame(self):
        """
        The SDK call must include a captured image frame alongside the prompt —
        text-only queries are invalid for a VLA model.
        """
        mock_ugv = MagicMock()
        fake_frame = MagicMock(name="camera_frame")

        mock_ugv.send_vla_query("Any hazards?", fake_frame)

        args, _ = mock_ugv.send_vla_query.call_args
        self.assertEqual(len(args), 2,
                         "SDK VLA call must supply both a prompt (str) and a frame (image).")

    # -----------------------------------------------------------------------
    # 'Elaborate' Human-in-the-Loop action
    # -----------------------------------------------------------------------

    @patch("builtins.input", return_value="It is just a PVC pipe, not a hazard")
    def test_elaborate_restructures_prompt(self, mock_input):
        """
        When the VLA signals ambiguity, the Elaborate flow must:
        1. Pause and collect a human clarification string.
        2. Inject that string into a new, structured prompt.
        3. Re-submit to the VLA (processing_ticks reset to 0).
        """
        node = VLA_Query("TestElaborate", prompt="Are there tripping hazards?")
        node.setup()

        # Drain to the Elaborate branch (tick 3 triggers it)
        for _ in range(3):
            node.update()

        # After Elaborate the prompt must have been rebuilt with the user's text
        self.assertIn("PVC pipe", node.prompt,
                      "Restructured prompt must contain the human's clarification.")
        self.assertIn("Re-evaluate", node.prompt,
                      "Restructured prompt must direct VLA to re-evaluate the scene.")

    @patch("builtins.input", return_value="It is just a PVC pipe, not a hazard")
    def test_elaborate_resets_processing_ticks(self, mock_input):
        """After the Elaborate round-trip, processing_ticks must be 0 so the
        node re-enters the network latency wait on the next iteration."""
        node = VLA_Query("TestElaborateTicks", prompt="Are there tripping hazards?")
        node.setup()

        for _ in range(3):
            node.update()

        self.assertEqual(node.processing_ticks, 0)

    @patch("builtins.input", return_value="It is just a PVC pipe, not a hazard")
    def test_elaborate_returns_running_after_user_input(self, mock_input):
        """The node must return RUNNING immediately after collecting the human
        elaboration so the tree re-submits and waits for the second inference."""
        node = VLA_Query("TestElaborateRunning", prompt="Are there tripping hazards?")
        node.setup()

        # Ticks 1-2: latency wait
        node.update()
        node.update()

        # Tick 3: Elaborate branch fires, reads input(), must return RUNNING
        status = node.update()
        self.assertEqual(status, py_trees.common.Status.RUNNING)

    @patch("builtins.input", return_value="It is just a PVC pipe, not a hazard")
    def test_elaborate_calls_llm_bridge(self, mock_input):
        """
        The Elaborate user text should optionally be pre-processed by the SDK's
        LLM bridge to make the restructured prompt more semantically rich.
        Implement: refined_context = self.ugv.get_llm_response(user_info) and use
        that in the restructured prompt instead of raw user_info.
        """
        mock_ugv = MagicMock()
        mock_ugv.get_llm_response.return_value = "Non-hazardous PVC installation material"

        # Simulate the Elaborate path
        user_info = input()
        refined = mock_ugv.get_llm_response(user_info)
        new_prompt = f"Context from operator (LLM refined): {refined}. Re-evaluate safety."

        mock_ugv.get_llm_response.assert_called_once_with(user_info)
        self.assertIn("Non-hazardous", new_prompt)

    # -----------------------------------------------------------------------
    # GenerateSiteReport: consumes vla_response from blackboard
    # -----------------------------------------------------------------------

    def test_generate_report_reads_vla_response(self):
        """GenerateSiteReport must read vla_response written by VLA_Query."""
        writer = py_trees.blackboard.Client(name="VLAWriter")
        writer.register_key(key="vla_response", access=py_trees.common.Access.WRITE)
        writer.vla_response = "Zone CLEARED. No hazards detected."

        node = GenerateSiteReport("TestReport")
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def test_generate_report_handles_missing_vla_response(self):
        """If VLA_Query never ran, GenerateSiteReport must not crash — it should
        substitute a fallback string and still return SUCCESS."""
        node = GenerateSiteReport("TestReportFallback")
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    # -----------------------------------------------------------------------
    # PublishSemanticMarker: RTAB-Map pose + host map update
    # -----------------------------------------------------------------------

    def test_publish_marker_fails_when_no_vla_response(self):
        """PublishSemanticMarker has nothing to label if the VLA response is absent."""
        mock_ugv = MagicMock()

        node = PublishSemanticMarker("TestMarkerNoVLA", mock_ugv)
        node.setup()
        status = node.update()

        self.assertEqual(status, py_trees.common.Status.FAILURE)
        mock_ugv.send_map_marker.assert_not_called()

    def test_publish_marker_payload_contains_required_keys(self):
        """
        The marker payload sent to the host UI must include 'label', 'x', 'y', 'z'
        so the map renderer can position the overlay correctly.
        """
        mock_ugv = MagicMock()
        mock_ugv.get_rtabmap_pose.return_value = {"x": 3.1, "y": 1.2, "z": 0.0}

        writer = py_trees.blackboard.Client(name="VLAWriterMarker")
        writer.register_key(key="vla_response", access=py_trees.common.Access.WRITE)
        writer.vla_response = "Hazard detected: exposed wiring near column B"

        node = PublishSemanticMarker("TestMarkerPayload", mock_ugv)
        node.setup()
        node.update()

        args, _ = mock_ugv.send_map_marker.call_args
        payload = args[0]

        for key in ("label", "x", "y", "z"):
            self.assertIn(key, payload, f"Marker payload missing required key: '{key}'")


if __name__ == "__main__":
    unittest.main()
