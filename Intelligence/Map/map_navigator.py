#!/usr/bin/env python3
"""
Map Navigator
=============
Displays rtabmap.png (10.25 × 12.1 m, centre = world origin) and lets you
drive the UGV by dragging an arrow on the map:

  • Left-click drag  → send  goto(target_position)
                       Arrow origin = XY goal, arrow direction = heading hint
  • S                → stop the robot
  • Q / Esc          → quit

The robot's live position and heading are rendered on the map in real time.
"""

import os
import sys
import math
import threading
import json

import cv2
import numpy as np

# ── Allow importing from Experiments/ ───────────────────────────────────────
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_THIS_DIR, "..", "..", "Experiments"))

from config import TWIN_UUID, TWIN_LIDAR_UUID, ENVIRONMENT_UUID
from cyberwave import Cyberwave

# ── Physical map dimensions (metres) ────────────────────────────────────────
MAP_W_M = 10.25   # x-axis  (left ↔ right)
MAP_H_M = 12.1    # y-axis  (down ↕ up)

# ── Display scaling (pixels per source pixel) ────────────────────────────────
DISPLAY_SCALE = 5

# ── Load map image ───────────────────────────────────────────────────────────
_MAP_PATH = os.path.join(_THIS_DIR, "rtabmap.png")
_BASE_IMG = cv2.imread(_MAP_PATH)
if _BASE_IMG is None:
    raise FileNotFoundError(f"Cannot load map image: {_MAP_PATH}")

_SRC_H, _SRC_W = _BASE_IMG.shape[:2]
DISP_W = _SRC_W * DISPLAY_SCALE
DISP_H = _SRC_H * DISPLAY_SCALE

# ── Load logo (BGRA so we can use the alpha channel) ─────────────────────────
_LOGO_PATH = os.path.join(_THIS_DIR, "CyberwaveLogo.png")
_LOGO_BGRA = cv2.imread(_LOGO_PATH, cv2.IMREAD_UNCHANGED)  # may be BGR or BGRA
if _LOGO_BGRA is not None:
    # Scale logo so its width is ~18 % of the display width
    _logo_target_w = max(120, int(DISP_W * 0.38))
    _logo_h, _logo_w = _LOGO_BGRA.shape[:2]
    _logo_scale = _logo_target_w / _logo_w
    _logo_target_h = int(_logo_h * _logo_scale)
    _LOGO_BGRA = cv2.resize(_LOGO_BGRA, (_logo_target_w, _logo_target_h), interpolation=cv2.INTER_AREA)
    # If no alpha channel, add one (fully opaque)
    if _LOGO_BGRA.shape[2] == 3:
        _LOGO_BGRA = cv2.cvtColor(_LOGO_BGRA, cv2.COLOR_BGR2BGRA)
else:
    print(f"Warning: could not load logo from {_LOGO_PATH}")
_LOGO_MARGIN = 10   # pixels from edge

# ── Coordinate conversion helpers ────────────────────────────────────────────
def world_to_px(wx: float, wy: float) -> tuple[int, int]:
    """World metres → display-image pixel (90° CW: world-y→image-x, world-x→image-y)."""
    px = int((wy / MAP_W_M + 0.5) * DISP_W)
    py = int((wx / MAP_H_M + 0.5) * DISP_H)
    return px, py


def px_to_world(px: int, py: int) -> tuple[float, float]:
    """Display-image pixel → world metres (inverse of 90° CW rotation)."""
    wx = (py / DISP_H - 0.5) * MAP_H_M
    wy = (px / DISP_W - 0.5) * MAP_W_M
    return wx, wy


def display_to_api(wx: float, wy: float) -> tuple[float, float]:
    """Convert from display frame to API frame (reverse the 90° CW rotation)."""
    # Display frame has 90° CW rotation, so undo it: swap and negate appropriately
    api_x = wy
    api_y = wx
    return api_x, api_y


# ── Robot state (updated from subscription callbacks) ────────────────────────
_robot_lock = threading.Lock()
_robot: dict = {"x": 0.0, "y": 0.0, "yaw": 0.0}

# ── Lidar state ──────────────────────────────────────────────────────────────
# Lidar is mounted at [0.04, 0.0, 0.12]m from base_footprint with 90° Z rotation
_LIDAR_OFFSET_X = 0.04  # metres
_LIDAR_OFFSET_Y = 0.0   # metres
# Rotation: lidar_optical → base_footprint is 90° CCW around Z
# Transform: base_x = -lidar_y + offset_x, base_y = lidar_x + offset_y

_lidar_lock = threading.Lock()
_lidar_scan = {"angles": [], "ranges": [], "intensities": []}


def _on_lidar_scan(data, topic=None, *args, **kwargs):
    """Lidar subscription callback – parse and update scan data."""
    if isinstance(data, str) and isinstance(topic, dict):
        data, topic = topic, data
    
    if topic and "/scan" in topic:
        ranges = data.get("ranges", [])
        intensities = data.get("intensities", [])
        angle_min = data.get("angle_min", 0.0)
        angle_increment = data.get("angle_increment", 0.0)
        
        if ranges:
            angles = [angle_min + i * angle_increment for i in range(len(ranges))]
            with _lidar_lock:
                _lidar_scan["angles"] = angles
                _lidar_scan["ranges"] = ranges
                _lidar_scan["intensities"] = intensities



def _quat_to_yaw(q: dict) -> float | None:
    if not isinstance(q, dict):
        return None
    x = q.get("x", 0.0)
    y = q.get("y", 0.0)
    z = q.get("z", 0.0)
    w = q.get("w", 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _on_position(data):
    """Position subscription callback – parse and update robot x/y."""
    if not isinstance(data, dict):
        return
    # Unwrap common nesting patterns
    if "message" in data and isinstance(data["message"], dict):
        return _on_position(data["message"])
    pos = None
    if "x" in data and "y" in data:
        pos = data
    elif "position" in data and isinstance(data["position"], dict):
        pos = data["position"]
    elif "pose" in data:
        p = data["pose"]
        if isinstance(p, dict):
            inner = p.get("pose", p)
            if isinstance(inner, dict):
                pos = inner.get("position", inner)
    if pos and "x" in pos and "y" in pos:
        with _robot_lock:
            _robot["x"] = float(pos["x"])
            _robot["y"] = float(pos["y"])


def _on_rotation(data):
    """Rotation subscription callback – parse and update robot yaw."""
    if not isinstance(data, dict):
        return
    if "message" in data and isinstance(data["message"], dict):
        return _on_rotation(data["message"])
    yaw = None
    if "yaw" in data:
        yaw = float(data["yaw"])
    elif "rotation" in data and isinstance(data["rotation"], dict):
        yaw = _quat_to_yaw(data["rotation"])
    elif "w" in data or "z" in data:
        yaw = _quat_to_yaw(data)
    elif "orientation" in data:
        yaw = _quat_to_yaw(data["orientation"])
    elif "pose" in data:
        p = data["pose"]
        if isinstance(p, dict):
            inner = p.get("pose", p)
            if isinstance(inner, dict):
                yaw = _quat_to_yaw(inner.get("orientation", {}))
    if yaw is not None:
        with _robot_lock:
            _robot["yaw"] = yaw


# ── Cyberwave connection ─────────────────────────────────────────────────────
cw = Cyberwave()
cw.affect("live")
ugv = cw.twin(
    "waveshare/ugv-beast",
    twin_id=TWIN_UUID,
    environment_id=ENVIRONMENT_UUID,
)
ugv.subscribe_position(_on_position)
ugv.subscribe_rotation(_on_rotation)

# Connect to lidar and subscribe to scans
d500 = cw.twin(twin_id=TWIN_LIDAR_UUID)
d500.subscribe(_on_lidar_scan)

print("Connected to UGV twin. Subscribing to position/rotation...")
print(f"Connected to Lidar twin {TWIN_LIDAR_UUID}. Subscribing to scans...")

# ── Drag state ───────────────────────────────────────────────────────────────
_drag: dict = {"active": False, "start": None, "end": None}
_last_goal: dict | None = None   # {"x": ..., "y": ..., "yaw": ...}


def _on_mouse(event, x, y, flags, param):
    global _last_goal
    is_ctrl_down = flags & cv2.EVENT_FLAG_CTRLKEY
    is_shift_down = flags & cv2.EVENT_FLAG_SHIFTKEY
    
    if event == cv2.EVENT_LBUTTONDOWN:
        _drag["active"] = True
        _drag["start"]  = (x, y)
        _drag["end"]    = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and _drag["active"]:
        _drag["end"] = (x, y)
    elif event == cv2.EVENT_LBUTTONUP and _drag["active"]:
        _drag["active"] = False
        sx, sy = _drag["start"]
        ex, ey = x, y
        # Target position = arrow start
        tx, ty = px_to_world(sx, sy)
        
        if is_ctrl_down:
            # CTRL+Click: Set initial pose via MQTT
            # Reverse the calibration transform: translate then rotate by -π/2
            # First apply inverse translation
            x_trans = tx + 2.6
            y_trans = ty + 3.3
            # Then apply inverse rotation (-π/2 around Z)
            api_x = y_trans
            api_y = -x_trans
            
            # Build quaternion from heading
            dx, dy = ex - sx, ey - sy
            heading_yaw = math.atan2(dx, dy)
            # Reverse the calibration rotation by subtracting π/2 (inverse of the forward +π/2)
            heading_yaw = heading_yaw - math.pi / 2
            half_yaw = heading_yaw / 2.0
            qx, qy, qz = 0.0, 0.0, math.sin(half_yaw)
            qw = math.cos(half_yaw)
            
            # Build payload matching mqtt_bridge expectations
            payload = {
                "frame_id": "map",
                "position": [api_x, api_y, 0.0],
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                "covariance": [0.25, 0, 0, 0, 0, 0,
                              0, 0.25, 0, 0, 0, 0,
                              0, 0, 0.25, 0, 0, 0,
                              0, 0, 0, 0.0, 0, 0,
                              0, 0, 0, 0, 0.0, 0,
                              0, 0, 0, 0, 0, 0.25]
            }
            
            try:
                topic = f"cyberwave/twin/{TWIN_UUID}/navigate/initialpose"
                # JSON-encode the payload to ensure proper serialization through MQTT
                payload_json = json.dumps(payload)
                if hasattr(cw, 'mqtt'):
                    cw.mqtt.publish(topic, payload_json)
                else:
                    print(f"Cannot publish: Cyberwave object has no mqtt attribute")
                    raise AttributeError("No MQTT publish method available")
                print(
                    f"→ setInitialPose ({api_x:+.3f}, {api_y:+.3f}) "
                    f"heading={math.degrees(heading_yaw):+.1f}°"
                )
            except Exception as e:
                print(f"Error publishing initialpose: {e}")
        elif is_shift_down:
            # Shift+Click: Set goal pose via goto
            # Reverse the calibration transform: translate then rotate by -π/2
            # First apply inverse translation
            x_trans = tx + 2.6
            y_trans = ty + 3.3
            # Then apply inverse rotation (-π/2 around Z)
            api_x = y_trans
            api_y = -x_trans
            
            # Build quaternion from heading
            dx, dy = ex - sx, ey - sy
            goal_yaw = math.atan2(dx, dy)
            # Reverse the calibration rotation by subtracting π/2 (inverse of the forward +π/2)
            goal_yaw = goal_yaw - math.pi / 2
            _last_goal = {"x": tx, "y": ty, "yaw": goal_yaw}
            
            resp = ugv.navigation.goto([api_x, api_y, 0.0], source_type="tele")
            print(
                f"→ goto ({api_x:+.3f}, {api_y:+.3f}) "
                f"heading={math.degrees(goal_yaw):+.1f}°  |  resp: {resp}"
            )
        _drag["start"] = _drag["end"] = None


# ── Drawing helpers ──────────────────────────────────────────────────────────
_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_COL_ROBOT = (0, 220, 0)        # green
_COL_GOAL  = (0, 140, 255)      # orange
_COL_GRID  = (180, 60, 60)      # slate-blue  – visible on both white & black map
_COL_AXIS  = (220, 100, 100)    # brighter blue for the zero axes
_COL_HUD   = (200, 200, 200)    # light grey


def _draw_grid(frame: np.ndarray, step_m: float = 1.0):
    """Draw a metric grid with labelled axes.
    After the 90° CW swap: world-y → image-x (vertical lines),
                           world-x → image-y (horizontal lines).
    """
    # Vertical lines: constant world-y → constant image-x
    wy_range = math.ceil(MAP_W_M / 2 / step_m)
    for i in range(-wy_range, wy_range + 1):
        px, _ = world_to_px(0, i * step_m)   # world-y in 2nd arg
        colour = _COL_AXIS if i == 0 else _COL_GRID
        cv2.line(frame, (px, 0), (px, DISP_H - 1), colour, 1)
        if i != 0:
            cv2.putText(frame, f"y={i*step_m:g}", (px + 2, 12),
                        _FONT, 0.32, _COL_GRID, 1, cv2.LINE_AA)

    # Horizontal lines: constant world-x → constant image-y
    wx_range = math.ceil(MAP_H_M / 2 / step_m)
    for i in range(-wx_range, wx_range + 1):
        _, py = world_to_px(i * step_m, 0)   # world-x in 1st arg
        colour = _COL_AXIS if i == 0 else _COL_GRID
        cv2.line(frame, (0, py), (DISP_W - 1, py), colour, 1)
        if i != 0:
            cv2.putText(frame, f"x={i*step_m:g}", (2, py - 2),
                        _FONT, 0.32, _COL_GRID, 1, cv2.LINE_AA)

    # Origin cross-hair
    ox, oy = world_to_px(0, 0)
    cv2.drawMarker(frame, (ox, oy), _COL_AXIS, cv2.MARKER_CROSS, 14, 1)
    cv2.putText(frame, "0", (ox + 3, oy - 3), _FONT, 0.32, _COL_AXIS, 1, cv2.LINE_AA)


def _draw_robot(frame: np.ndarray):
    with _robot_lock:
        rx, ry, ryaw = _robot["x"], _robot["y"], _robot["yaw"]
    px, py = world_to_px(rx, ry)
    r = 10
    cv2.circle(frame, (px, py), r, _COL_ROBOT, 2)
    # Heading arrow: world-yaw rotated 90° CW into image space
    # image-x ∝ world-y → sin(ryaw), image-y ∝ world-x (down+) → cos(ryaw)
    ex = int(px + r * 2.4 * math.sin(ryaw))
    ey = int(py + r * 2.4 * math.cos(ryaw))
    cv2.arrowedLine(frame, (px, py), (ex, ey), _COL_ROBOT, 2, tipLength=0.35)
    cv2.putText(frame, f"({rx:+.2f}, {ry:+.2f})", (px + 13, py - 6),
                _FONT, 0.38, _COL_ROBOT, 1, cv2.LINE_AA)


def _draw_lidar_scan(frame: np.ndarray):
    """Draw lidar scan points transformed to world frame and projected on map."""
    with _lidar_lock:
        angles = _lidar_scan["angles"]
        ranges = _lidar_scan["ranges"]
        intensities = _lidar_scan["intensities"]
    
    if not angles or not ranges:
        return
    
    with _robot_lock:
        rx, ry, ryaw = _robot["x"], _robot["y"], _robot["yaw"]
    
    for i, (angle, distance) in enumerate(zip(angles, ranges)):
        if distance <= 0.01 or distance > 12.0:  # Skip invalid or too-far points
            continue
        
        # Lidar point in lidar optical frame (XY plane, assuming scan in horizontal plane)
        lidar_x = distance * math.cos(angle)
        lidar_y = distance * math.sin(angle)
        
        # Transform to base_footprint frame (90° CCW rotation + translation)
        # base_x = -lidar_y + offset_x
        # base_y = lidar_x + offset_y
        base_x = -lidar_y + _LIDAR_OFFSET_X
        base_y = lidar_x + _LIDAR_OFFSET_Y
        
        # Rotate to world frame using robot's yaw
        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)
        world_x = rx + base_x * cos_yaw - base_y * sin_yaw
        world_y = ry + base_x * sin_yaw + base_y * cos_yaw
        
        # Skip if coordinates are invalid
        if not (math.isfinite(world_x) and math.isfinite(world_y)):
            continue
        
        # Project to image
        px, py = world_to_px(world_x, world_y)
        
        # Determine colour based on intensity using rainbow (HSV -> BGR)
        if intensities and len(intensities) == len(ranges):
            intensity = intensities[i]
            # Map intensity (0-255) to hue (0-180 in OpenCV)
            h = int((intensity / 255.0) * 180)
            # Create a single-pixel HSV image and convert to BGR
            hsv_px = np.uint8([[[h, 255, 255]]])  # Full saturation and value for bright colors
            bgr_px = cv2.cvtColor(hsv_px, cv2.COLOR_HSV2BGR)[0][0]
            col = tuple(int(x) for x in bgr_px)
        else:
            col = (255, 255, 255)  # White if no intensity data
        
        cv2.circle(frame, (px, py), 4, col, -1)


def _draw_last_goal(frame: np.ndarray):
    if _last_goal is None:
        return
    gx, gy, gyaw = _last_goal["x"], _last_goal["y"], _last_goal["yaw"]
    px, py = world_to_px(gx, gy)
    r = 8
    ex = int(px + r * 2.6 * math.sin(gyaw))
    ey = int(py + r * 2.6 * math.cos(gyaw))
    cv2.circle(frame, (px, py), r, _COL_GOAL, 1)
    cv2.arrowedLine(frame, (px, py), (ex, ey), _COL_GOAL, 1, tipLength=0.35)


def _draw_drag_arrow(frame: np.ndarray):
    if _drag["start"] is None or _drag["end"] is None:
        return
    sx, sy = _drag["start"]
    ex, ey = _drag["end"]
    tx, ty = px_to_world(sx, sy)
    cv2.circle(frame, (sx, sy), 7, _COL_GOAL, -1)
    if (ex - sx) ** 2 + (ey - sy) ** 2 > 16:
        cv2.arrowedLine(frame, (sx, sy), (ex, ey), _COL_GOAL, 2, tipLength=0.25)
    cv2.putText(frame, f"({tx:+.2f}, {ty:+.2f})", (sx + 10, sy - 8),
                _FONT, 0.4, _COL_GOAL, 1, cv2.LINE_AA)


def _draw_logo(frame: np.ndarray):
    """Composite the Cyberwave logo into the top-right corner."""
    if _LOGO_BGRA is None:
        return
    lh, lw = _LOGO_BGRA.shape[:2]
    x1 = frame.shape[1] - lw - _LOGO_MARGIN
    y1 = _LOGO_MARGIN
    x2, y2 = x1 + lw, y1 + lh
    # Clamp to frame bounds
    if x1 < 0 or y1 < 0 or x2 > frame.shape[1] or y2 > frame.shape[0]:
        return
    logo_bgr   = _LOGO_BGRA[:, :, :3].astype(np.float32)
    alpha_mask = (_LOGO_BGRA[:, :, 3] / 255.0).astype(np.float32)[:, :, np.newaxis]
    roi = frame[y1:y2, x1:x2].astype(np.float32)
    blended = (logo_bgr * alpha_mask + roi * (1.0 - alpha_mask)).astype(np.uint8)
    frame[y1:y2, x1:x2] = blended


def _draw_hud(frame: np.ndarray):
    """Bottom-left status / legend bar."""
    h = frame.shape[0]
    lines = [
        "Shift+drag -> goto target",
        "Ctrl+drag -> set initial pose",
        "S = stop robot   Q / Esc = quit",
    ]
    for i, txt in enumerate(reversed(lines)):
        cv2.putText(frame, txt, (6, h - 8 - 16 * i),
                    _FONT, 0.38, _COL_HUD, 1, cv2.LINE_AA)


# ── Main loop ────────────────────────────────────────────────────────────────
WIN = "Map Navigator"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, DISP_W, DISP_H)
cv2.setMouseCallback(WIN, _on_mouse)

print(f"Map: {_SRC_W}×{_SRC_H} px → {DISP_W}×{DISP_H} px  "
      f"({MAP_W_M} × {MAP_H_M} m).  Drag on window to navigate. Press Q to quit.")

try:
    while True:
        frame = cv2.resize(_BASE_IMG, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
        _draw_grid(frame)
        _draw_last_goal(frame)
        _draw_drag_arrow(frame)
        _draw_lidar_scan(frame)
        _draw_robot(frame)
        _draw_logo(frame)
        _draw_hud(frame)
        cv2.imshow(WIN, frame)

        key = cv2.waitKey(33) & 0xFF
        if key in (ord('q'), 27):   # Q or Esc
            break
        elif key == ord('s'):
            resp = ugv.navigation.stop(source_type="tele")
            print(f"→ stop  |  resp: {resp}")
finally:
    cv2.destroyAllWindows()
    cw.disconnect()
    print("Disconnected.")
