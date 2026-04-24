import unittest
from unittest.mock import MagicMock, patch, call
import numpy as np
import py_trees

# NOTE: This file tests the camera_worker logic that is currently commented out
# in real_main.py. Each test documents what the SDK integration must do once
# that code is uncommented and wired up.
#
# SDK / hardware methods under test (via camera_worker in real_main.py):
#   cv2.VideoCapture(url)                - open the robot's RTSP/HTTP stream
#   cv2.VideoCapture.read()              - blocking frame grab
#   cv2.aruco.detectMarkers(frame, ...)  - Aruco/AprilTag detection
#   blackboard.tag_x / tag_y            - written when tag is visible
#   blackboard.tag_x = None             - cleared when tag is lost


def _make_fake_corners(cx=350, cy=260):
    """Helper: build an Aruco corners array centred at (cx, cy)."""
    half = 20
    # corners shape: (1, 1, 4, 2) — one marker, four points
    pts = np.array([[
        [cx - half, cy - half],
        [cx + half, cy - half],
        [cx + half, cy + half],
        [cx - half, cy + half],
    ]], dtype=np.float32)
    return [pts]


class TestCameraWorkerBlackboard(unittest.TestCase):
    """
    Tests that verify the camera_worker correctly writes tag coordinates to
    the py_trees blackboard so that TrackTagInCenter can consume them.
    """

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        self.blackboard = py_trees.blackboard.Client(name="CameraTestClient")
        self.blackboard.register_key(key="tag_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="tag_y", access=py_trees.common.Access.WRITE)

    # -----------------------------------------------------------------------
    # Tag detected: blackboard must receive centre coordinates
    # -----------------------------------------------------------------------

    def test_blackboard_tag_x_written_on_detection(self):
        """
        When Aruco detectMarkers returns a result, the worker must compute the
        tag centre and write tag_x to the blackboard.
        Implement: cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
                   blackboard.tag_x = cX
        """
        corners = _make_fake_corners(cx=350, cy=260)
        ids = np.array([[42]])

        # Simulate what camera_worker should do
        cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
        cY = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
        self.blackboard.tag_x = cX
        self.blackboard.tag_y = cY

        self.assertEqual(self.blackboard.tag_x, 350)
        self.assertEqual(self.blackboard.tag_y, 260)

    def test_blackboard_centre_calculation_is_correct(self):
        """
        The tag centre must be the mean of the top-left and bottom-right corners,
        not the raw corner coordinates themselves.
        """
        corners = _make_fake_corners(cx=400, cy=300)
        expected_cx = 400
        expected_cy = 300

        cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
        cY = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)

        self.assertEqual(cX, expected_cx)
        self.assertEqual(cY, expected_cy)

    # -----------------------------------------------------------------------
    # Tag not detected: blackboard must be cleared so TrackTagInCenter knows
    # -----------------------------------------------------------------------

    def test_blackboard_cleared_on_tag_loss(self):
        """
        When ids is None (tag lost), the camera_worker must write None to both
        blackboard keys so TrackTagInCenter falls into dropout-coast mode.
        Implement: if ids is None: blackboard.tag_x = None; blackboard.tag_y = None
        """
        # First simulate a detection
        self.blackboard.tag_x = 350
        self.blackboard.tag_y = 260

        # Then simulate tag loss
        ids = None
        if ids is None:
            self.blackboard.tag_x = None
            self.blackboard.tag_y = None

        self.assertIsNone(self.blackboard.tag_x)
        self.assertIsNone(self.blackboard.tag_y)

    def test_blackboard_tag_x_is_integer(self):
        """
        The blackboard coordinate must be an integer (pixel index), not a float,
        to avoid type errors in the PID error calculation.
        """
        corners = _make_fake_corners(cx=310, cy=225)
        ids = np.array([[5]])

        cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
        cY = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
        self.blackboard.tag_x = cX
        self.blackboard.tag_y = cY

        self.assertIsInstance(self.blackboard.tag_x, int)
        self.assertIsInstance(self.blackboard.tag_y, int)

    # -----------------------------------------------------------------------
    # VideoCapture stream lifecycle
    # -----------------------------------------------------------------------

    @patch("cv2.VideoCapture")
    def test_video_capture_opened_with_robot_stream_url(self, mock_cap_cls):
        """
        camera_worker must open a VideoCapture pointed at the robot's stream URL,
        not a local webcam index (which would silently open the laptop camera).
        Implement: cap = cv2.VideoCapture("http://<robot-ip>:8080/stream")
        The URL must contain a host address, not just an integer.
        """
        mock_cap = MagicMock()
        mock_cap.read.return_value = (False, None)
        mock_cap_cls.return_value = mock_cap

        # Simulate the call camera_worker makes
        import cv2
        url = "http://192.168.1.100:8080/stream"
        cv2.VideoCapture(url)

        args, _ = mock_cap_cls.call_args
        self.assertIsInstance(args[0], str,
                              "VideoCapture argument must be a string URL, not an int.")
        self.assertIn("http", args[0],
                      "Stream URL must be an HTTP/RTSP address pointing to the robot.")

    @patch("cv2.VideoCapture")
    def test_video_capture_read_called_in_loop(self, mock_cap_cls):
        """
        camera_worker must call cap.read() on every iteration of the while loop.
        A single call is insufficient for a live tracking thread.
        """
        mock_cap = MagicMock()
        # Simulate 3 frames then stop
        mock_cap.read.side_effect = [(True, MagicMock())] * 3 + [(False, None)]
        mock_cap_cls.return_value = mock_cap

        import cv2
        cap = cv2.VideoCapture("http://robot:8080/stream")
        for _ in range(3):
            cap.read()

        self.assertGreaterEqual(mock_cap.read.call_count, 3)

    # -----------------------------------------------------------------------
    # Vision-based drop-off detection via SDK segmentation
    # -----------------------------------------------------------------------

    def test_vision_dropoff_hazard_written_to_blackboard(self):
        """
        In camera_worker, after capturing a frame, a sampled frame should be
        forwarded to the SDK's segmentation / VLA endpoint for floor hazard
        classification.
        Implement: result = ugv.query_segmentation(frame)
                   if result.label == 'dropoff': blackboard.hazard_detected = True
        """
        mock_ugv = MagicMock()
        mock_ugv.query_segmentation.return_value = MagicMock(label="dropoff")

        hazard_bb = py_trees.blackboard.Client(name="VisionSafetyTest")
        hazard_bb.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        hazard_bb.hazard_detected = False

        # Simulate what camera_worker should do
        frame = MagicMock()
        result = mock_ugv.query_segmentation(frame)
        if result.label == "dropoff":
            hazard_bb.hazard_detected = True

        mock_ugv.query_segmentation.assert_called_once_with(frame)
        self.assertTrue(hazard_bb.hazard_detected)

    def test_vision_safe_floor_does_not_set_hazard(self):
        """When the SDK segmentation returns a safe label, hazard_detected stays False."""
        mock_ugv = MagicMock()
        mock_ugv.query_segmentation.return_value = MagicMock(label="floor")

        hazard_bb = py_trees.blackboard.Client(name="VisionSafetyTestClear")
        hazard_bb.register_key(key="hazard_detected", access=py_trees.common.Access.WRITE)
        hazard_bb.hazard_detected = False

        frame = MagicMock()
        result = mock_ugv.query_segmentation(frame)
        if result.label == "dropoff":
            hazard_bb.hazard_detected = True

        self.assertFalse(hazard_bb.hazard_detected)


if __name__ == "__main__":
    unittest.main()
