"""
apriltag_localizer.py
=====================
Converts solvePnP output + known tag definition into an absolute robot pose
in the map API coordinate frame.

Coordinate systems
------------------
map_navigator WORLD frame  (map_navigator.px_to_world output):
    x  : metres, increases toward bottom of screen
    y  : metres, increases toward right of screen
    yaw: not directly relevant here

map API frame  (what goto / initialpose accept):
    api_x = map_world_y + WORLD_TO_API_X_OFFSET
    api_y = -map_world_x + WORLD_TO_API_Y_OFFSET
    api_yaw: angle in radians, 0 = facing +api_x, CCW positive

Screen PIXEL frame  (what heading_hint dx/dy are in):
    +dx = screen right  →  +api_x  →  +world_y
    +dy = screen down   →  +api_y  →  +world_x

OpenCV camera frame (solvePnP output):
    +X = right of image
    +Y = DOWN (not up!)
    +Z = forward / into scene

ArUco tag frame (solvePnP convention):
    +X = right when looking at the tag face
    +Y = DOWN when looking at the tag face
    +Z = OUT of the tag face (toward the camera)
"""

import math
import numpy as np
import cv2

# ──────────────────────────────────────────────────────────────────────────────
# Tuning knobs (change these if the output is flipped/rotated)
# ──────────────────────────────────────────────────────────────────────────────

# Negate computed camera yaw to match API convention.
# +1 = yaw is CCW-positive as computed; -1 = flip sign (use when measured
# yaw is mirrored relative to ground truth, as confirmed empirically).
YAW_SIGN = -1

# World (map_navigator) -> API calibration.
# Keep this aligned with map_navigator's Ctrl+click initialpose conversion.
#   api_x = world_y + WORLD_TO_API_X_OFFSET
#   api_y = -world_x + WORLD_TO_API_Y_OFFSET
WORLD_TO_API_X_OFFSET = 3.3
WORLD_TO_API_Y_OFFSET = -2.6

# Camera optical center offset from robot base in robot frame (metres).
# +forward: robot forward, +left: robot left.
# If unknown, keep both at 0.0 and tune from measurements.
CAMERA_FROM_BASE_FORWARD_M = 0.0
CAMERA_FROM_BASE_LEFT_M = 0.0

# Interpretation of heading_hint from APRILTAG_MARKERS:
#   False -> heading_hint already points along tag +Z (outward normal)
#   True  -> heading_hint points along tag face tangent (+X), so +90 deg needed
# If your localization appears rotated by +/-90 deg, this flag is the first knob
# to check.
HEADING_HINT_IS_FACE_TANGENT = False


def _normalize_angle(angle_rad: float) -> float:
    """Normalize angle to (-pi, pi]."""
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def world_to_api(wx: float, wy: float) -> tuple[float, float]:
    """Apply the calibrated world->API transform used by map control."""
    api_x = wy + WORLD_TO_API_X_OFFSET
    api_y = -wx + WORLD_TO_API_Y_OFFSET
    return api_x, api_y


# ──────────────────────────────────────────────────────────────────────────────
# Step 1 – Tag definition → API frame
# ──────────────────────────────────────────────────────────────────────────────

def tag_to_api(known_tag: dict) -> tuple[float, float, float]:
    """
    Convert a tag entry from APRILTAG_MARKERS (map_navigator world frame) to
    the API position and facing-direction yaw.

    APRILTAG_MARKERS stores positions in the map_navigator world frame where:
        world_x → drawn at pixel_y  (world_to_px: py = (wx/MAP_H_M + 0.5)*DISP_H)
        world_y → drawn at pixel_x  (world_to_px: px = (wy/MAP_W_M + 0.5)*DISP_W)

    The runtime uses a calibrated world->API transform:
        api_x = world_y + WORLD_TO_API_X_OFFSET
        api_y = -world_x + WORLD_TO_API_Y_OFFSET

    heading_hint (dx, dy) is a direction in screen pixels:
        +dx = screen right = +api_x
        +dy = screen down
    heading_hint can be either:
        - tag normal (+Z): use as-is for tag yaw, or
        - tag face tangent (+X): add +pi/2 to convert to +Z normal.
    This behavior is controlled by HEADING_HINT_IS_FACE_TANGENT.
    We first convert heading_hint into world yaw, then world->API yaw:
        world_yaw = atan2(dx_pixel, dy_pixel)
        api_yaw   = world_yaw - pi/2

    The default in this project treats heading_hint as the tag normal because
    a tangent interpretation introduces a 90° rotation error.

    Returns: (api_x, api_y, api_yaw)
    """
    wx = known_tag["x"]
    wy = known_tag["y"]
    api_x, api_y = world_to_api(wx, wy)

    dx = known_tag["heading_hint"]["dx"]
    dy = known_tag["heading_hint"]["dy"]

    # heading_hint in display pixels -> world yaw -> API yaw
    heading_world_yaw = math.atan2(dx, dy)
    heading_api_yaw = heading_world_yaw - math.pi / 2

    if HEADING_HINT_IS_FACE_TANGENT:
        # heading_hint is along tag face (+X), so rotate to outward normal (+Z)
        api_yaw = _normalize_angle(heading_api_yaw + math.pi / 2)
    else:
        # heading_hint already describes the outward normal (+Z)
        api_yaw = _normalize_angle(heading_api_yaw)

    return api_x, api_y, api_yaw


# ──────────────────────────────────────────────────────────────────────────────
# Step 2 – solvePnP output → camera pose in API frame
# ──────────────────────────────────────────────────────────────────────────────

def camera_pose_in_api(
    rvec: np.ndarray,
    tvec: np.ndarray,
    tag_api_x: float,
    tag_api_y: float,
    tag_api_yaw: float,
) -> tuple[float, float, float]:
    """
    Given solvePnP rvec/tvec for a known tag, return the camera's absolute
    position and yaw in the API map frame.

    Math
    ----
    R_cam_tag  : rotation from tag frame → camera frame
    tvec       : tag origin in camera frame
    P_cam_tag  : camera origin in tag frame = -R_cam_tag.T @ tvec

    Tag frame axes projected into API frame (2-D horizontal plane only):
        tag +Z (face normal, heading_hint direction): (cos(yaw), sin(yaw))
        tag +X (right of face):                       (-sin(yaw), cos(yaw))

    Camera API position:
        cam_api = tag_api + tag_x_api * x_cam_in_tag + tag_z_api * z_cam_in_tag

    Camera API yaw:
        Project camera forward (+Z_cam) into tag frame, then into API frame.
        Note: OpenCV Y is DOWN, so CW rotation produces CCW atan2 increase.
        YAW_SIGN corrects this; default +1 for CCW-positive (ROS convention).

    Returns: (cam_api_x, cam_api_y, cam_api_yaw)
    """
    R_cam_tag, _ = cv2.Rodrigues(rvec)
    tvec_flat = tvec.reshape(3)

    # Camera origin expressed in tag frame
    P_cam_tag = -R_cam_tag.T @ tvec_flat
    x_cam = P_cam_tag[0]   # camera is x_cam metres to the right of the tag
    z_cam = P_cam_tag[2]   # camera is z_cam metres in front of the tag

    # Tag-frame unit vectors in API space (only XY plane matters for 2-D nav)
    cyw, syw = math.cos(tag_api_yaw), math.sin(tag_api_yaw)
    tag_z_api = np.array([cyw,  syw])   # tag +Z in API frame
    tag_x_api = np.array([-syw, cyw])   # tag +X in API frame (90° CCW from +Z)

    # Camera absolute position in API frame
    cam_api_x = tag_api_x + tag_x_api[0] * x_cam + tag_z_api[0] * z_cam
    cam_api_y = tag_api_y + tag_x_api[1] * x_cam + tag_z_api[1] * z_cam

    # Camera forward direction: +Z_cam expressed in tag frame, then API frame
    cam_fwd_tag = R_cam_tag.T @ np.array([0.0, 0.0, 1.0])
    cam_fwd_api_x = tag_x_api[0] * cam_fwd_tag[0] + tag_z_api[0] * cam_fwd_tag[2]
    cam_fwd_api_y = tag_x_api[1] * cam_fwd_tag[0] + tag_z_api[1] * cam_fwd_tag[2]

    # YAW_SIGN flips direction if OpenCV parity is opposite to API convention
    cam_api_yaw = YAW_SIGN * math.atan2(cam_fwd_api_y, cam_fwd_api_x)

    return cam_api_x, cam_api_y, cam_api_yaw


# ──────────────────────────────────────────────────────────────────────────────
# Step 3 – Camera pose → robot chassis pose
# ──────────────────────────────────────────────────────────────────────────────

def camera_to_robot_pose(
    cam_api_x: float,
    cam_api_y: float,
    cam_api_yaw: float,
    pan_rad: float,
) -> tuple[float, float, float]:
    """
    Convert the camera's API pose to the robot chassis pose.

    1) Remove PTZ pan from the camera yaw to recover robot yaw.
    2) Subtract the camera's static translation from robot base to camera.

    Returns: (robot_api_x, robot_api_y, robot_api_yaw) with yaw normalised to (-pi, pi].
    """
    api_yaw = _normalize_angle(cam_api_yaw - pan_rad)

    # camera = base + R(base_yaw) * [forward, left]
    # so base = camera - R(base_yaw) * [forward, left]
    offset_x = (
        math.cos(api_yaw) * CAMERA_FROM_BASE_FORWARD_M
        - math.sin(api_yaw) * CAMERA_FROM_BASE_LEFT_M
    )
    offset_y = (
        math.sin(api_yaw) * CAMERA_FROM_BASE_FORWARD_M
        + math.cos(api_yaw) * CAMERA_FROM_BASE_LEFT_M
    )
    api_x = cam_api_x - offset_x
    api_y = cam_api_y - offset_y

    return api_x, api_y, api_yaw


# ──────────────────────────────────────────────────────────────────────────────
# Step 4 – Helpers
# ──────────────────────────────────────────────────────────────────────────────

def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert a 2-D yaw (radians) to a quaternion (qx, qy, qz, qw)."""
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_initialpose_payload(api_x: float, api_y: float, api_yaw: float) -> dict:
    """Build the MQTT initialpose payload dict."""
    qx, qy, qz, qw = yaw_to_quaternion(api_yaw)
    return {
        "frame_id": "map",
        "position": [api_x, api_y, 0.0],
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        "covariance": [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.25, 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.25,
        ],
    }


# ──────────────────────────────────────────────────────────────────────────────
# Top-level convenience
# ──────────────────────────────────────────────────────────────────────────────

def localise_from_tag(
    rvec: np.ndarray,
    tvec: np.ndarray,
    known_tag: dict,
    pan_rad: float = 0.0,
    debug: bool = True,
) -> tuple[dict, float, float, float]:
    """
    Full pipeline: solvePnP result + tag definition → initialpose payload.

    Parameters
    ----------
    rvec, tvec : solvePnP outputs
    known_tag  : entry from APRILTAG_MARKERS
    pan_rad    : actual PTZ pan in radians (from joint state)
    debug      : if True, print intermediate values for debugging

    Returns
    -------
    (payload_dict, api_x, api_y, api_yaw)
    """
    tag_api_x, tag_api_y, tag_api_yaw = tag_to_api(known_tag)

    cam_api_x, cam_api_y, cam_api_yaw = camera_pose_in_api(
        rvec, tvec, tag_api_x, tag_api_y, tag_api_yaw
    )

    api_x, api_y, api_yaw = camera_to_robot_pose(
        cam_api_x, cam_api_y, cam_api_yaw, pan_rad
    )

    if debug:
        dist = float(np.linalg.norm(tvec))
        print(
            f"[Localizer] tag_id={known_tag['id']}  dist={dist:.2f}m\n"
            f"  tag  API pos: ({tag_api_x:+.3f}, {tag_api_y:+.3f})"
            f"  yaw={math.degrees(tag_api_yaw):+.1f}°\n"
            f"  cam  API pos: ({cam_api_x:+.3f}, {cam_api_y:+.3f})"
            f"  yaw={math.degrees(cam_api_yaw):+.1f}°\n"
            f"  robot API pos: ({api_x:+.3f}, {api_y:+.3f})"
            f"  yaw={math.degrees(api_yaw):+.1f}°  (pan offset={math.degrees(pan_rad):+.1f}°)"
        )

    payload = build_initialpose_payload(api_x, api_y, api_yaw)
    return payload, api_x, api_y, api_yaw
