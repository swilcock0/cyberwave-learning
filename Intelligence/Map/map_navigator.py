#!/usr/bin/env python3
"""
Map Navigator
=============
Displays rtabmap.png (10.25 x 12.1 m, centre = world origin) and lets you
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

# ── Load circle logo (smaller, positioned below main logo) ───────────────────
_LOGO_CIRCLE_PATH = os.path.join(_THIS_DIR, "LogoCircleBW.png")
_LOGO_CIRCLE_BGRA = cv2.imread(_LOGO_CIRCLE_PATH, cv2.IMREAD_UNCHANGED)  # may be BGR or BGRA
if _LOGO_CIRCLE_BGRA is not None:
    # Scale circle logo smaller than main logo (~40% of main logo width)
    _circle_target_w = max(50, int(DISP_W * 0.10))
    _circle_h, _circle_w = _LOGO_CIRCLE_BGRA.shape[:2]
    _circle_scale = _circle_target_w / _circle_w
    _circle_target_h = int(_circle_h * _circle_scale)
    _LOGO_CIRCLE_BGRA = cv2.resize(_LOGO_CIRCLE_BGRA, (_circle_target_w, _circle_target_h), interpolation=cv2.INTER_AREA)
    # If no alpha channel, add one (fully opaque)
    if _LOGO_CIRCLE_BGRA.shape[2] == 3:
        _LOGO_CIRCLE_BGRA = cv2.cvtColor(_LOGO_CIRCLE_BGRA, cv2.COLOR_BGR2BGRA)
else:
    _LOGO_CIRCLE_BGRA = None
    print(f"Warning: could not load circle logo from {_LOGO_CIRCLE_PATH}")

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


def _check_zone(rx: float, ry: float) -> dict | None:
    """Check if robot is in any zone. Returns zone dict or None."""
    if not _zones:
        return None
    
    # Convert robot world coords to pixel coords
    px, py = world_to_px(rx, ry)
    
    # Check each zone mask at the robot's pixel position
    for zone_id, zone_data in sorted(_zones.items()):
        mask = zone_data["mask"]
        # Scale pixel coords to mask dimensions
        scale_x = mask.shape[1] / DISP_W
        scale_y = mask.shape[0] / DISP_H
        mx = int(px * scale_x)
        my = int(py * scale_y)
        
        # Clamp to mask bounds
        if 0 <= mx < mask.shape[1] and 0 <= my < mask.shape[0]:
            pixel = mask[my, mx]
            # Check if pixel is not in the black/transparent region
            # For any mask format, if it's not black/fully transparent, consider it active
            if isinstance(pixel, np.ndarray):
                # Multi-channel: check if any channel is significantly non-zero
                if np.max(pixel[:3] if pixel.shape[0] >= 3 else pixel) > 1:
                    return zone_data
            else:
                # Single channel (grayscale/alpha): check if non-zero
                if pixel > 1:
                    return zone_data
    
    return None

# ── Load zone masks ───────────────────────────────────────────────────────────
_ZONES_PATH = os.path.join(_THIS_DIR, "Zones")
_zones: dict = {}  # {zone_id: {"name": str, "mask": np.ndarray, "mask_scaled": np.ndarray}}
if os.path.exists(_ZONES_PATH):
    print(f"Loading zones from: {_ZONES_PATH}")
    for filename in sorted(os.listdir(_ZONES_PATH)):
        if filename.endswith(".png"):
            # Parse filename: "1_ZoneName.png" → id=1, name="ZoneName"
            base = filename[:-4]  # remove .png
            parts = base.split("_", 1)
            if len(parts) == 2 and parts[0].isdigit():
                zone_id = int(parts[0])
                zone_name = parts[1]
                zone_path = os.path.join(_ZONES_PATH, filename)
                zone_img = cv2.imread(zone_path, cv2.IMREAD_UNCHANGED)
                if zone_img is not None:
                    # Find centroid of colored (non-transparent) region
                    if zone_img.shape[2] == 4:
                        # Use alpha channel to mask non-transparent
                        alpha = zone_img[:, :, 3]
                        mask = alpha > 0
                    else:
                        # No alpha, use any non-black pixel
                        mask = np.any(zone_img[:, :, :3] > 1, axis=2)

                    coords = np.column_stack(np.where(mask))
                    if coords.size > 0:
                        centroid_px = coords.mean(axis=0)  # (y, x)
                        centroid_y, centroid_x = centroid_px
                        # Convert to world coordinates
                        # zone_img is (H, W), so map to display and then to world
                        disp_x = int(centroid_x * DISP_W / zone_img.shape[1])
                        disp_y = int(centroid_y * DISP_H / zone_img.shape[0])
                        wx, wy = px_to_world(disp_y, disp_x)
                        # print(f"    Centroid: (display px: {disp_x:.1f}, {disp_y:.1f})  (world: {wx:.3f}, {wy:.3f})")
                    else:
                        print("    Centroid: (no colored region found)")
                    # Scale to display dimensions
                    zone_scaled = cv2.resize(zone_img, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
                    _zones[zone_id] = {"name": zone_name, "mask": zone_img, "mask_scaled": zone_scaled, "centroid_disp": (disp_x, disp_y)}
                    print(f"  ✓ Zone {zone_id}: {zone_name} Centroid px({disp_x:.1f}, {disp_y:.1f}) Centroid world({wx:.3f}, {wy:.3f})")
                else:
                    print(f"  ✗ Failed to load {filename}")
else:
    print(f"Note: Zones folder not found at {_ZONES_PATH}")

if not _zones:
    print("  (No zones loaded)")

# ── AprilTag marker definitions ─────────────────────────────────────────────
# Each tag: id, size (m), x, y, heading_hint (dx, dy)
APRILTAG_MARKERS = [
    { #Charging
        "id": 0,
        "size_m": 0.2,  # 200mm
        "x": -2.070,
        "y": -3.905,
        "heading_hint": {"dx": 71, "dy": -46},
    },
        { # Bedroom
        "id": 1,
        "size_m": 0.2,  # 200mm
        "x": 4.11,
        "y": 2.32,
        "heading_hint": {"dx": -46, "dy": -76},
    },
    { #Hallway
        "id": 2,
        "size_m": 0.2,  # 200mm
        "x": 1.23,
        "y": 0.145,
        "heading_hint": {"dx": 60, "dy": -36},
    },
    { #Shower
        "id": 3,
        "size_m": 0.2,  # 200mm
        "x": 0.02,
        "y": 1.905,
        "heading_hint": {"dx": -51, "dy": 38},
    },
    # Add more tags here as needed
]

# ── Drawing function for AprilTags ──────────────────────────────────────────
def _draw_apriltags(frame: np.ndarray, tags=APRILTAG_MARKERS):
    for tag in tags:
        # Get display coordinates
        px, py = world_to_px(tag["x"], tag["y"])
        size_px = int((tag["size_m"]*2 / MAP_W_M) * DISP_W)
        # Heading: angle from dx, dy (display frame)
        dx, dy = tag["heading_hint"]["dx"], tag["heading_hint"]["dy"]
        angle = math.atan2(dy, dx)  # display frame: y down, x right
        # Rectangle corners (centered at px, py, rotated by angle)
        w, h = max(6, int(size_px * 0.12)), size_px  # much wider: 1:8 aspect, min height 6px
        rect = np.array([[-w//2, -h//2], [w//2, -h//2], [w//2, h//2], [-w//2, h//2]], dtype=np.float32)
        # Rotation matrix
        rot = np.array([
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle),  math.cos(angle)]
        ])
        rect_rot = np.dot(rect, rot.T) + np.array([px, py])
        pts = rect_rot.astype(np.int32)
        # Draw filled rectangle (light color)
        cv2.fillPoly(frame, [pts], (255, 20, 147))
        # Draw border
        cv2.polylines(frame, [pts], isClosed=True, color=(255, 20, 147), thickness=1)
        # Draw tag number (centered, larger, high contrast)
        font_scale = max(0.9, h / 16.0)  # scale with height, min 0.9
        text = str(tag["id"])
        (tw, th), _ = cv2.getTextSize(text, _FONT, font_scale, 3)
        text_x = int(px - tw / 2 - 40)
        text_y = int(py + th / 2)
        # Draw outline for contrast
        cv2.putText(frame, text, (text_x, text_y), _FONT, font_scale, (0, 0, 0), 2, cv2.LINE_AA)
        # Draw number (bright yellow)
        cv2.putText(frame, text, (text_x, text_y), _FONT, font_scale, (0, 220, 255), 2, cv2.LINE_AA)

_current_zone: dict | None = None  # Currently active zone (if robot is in one)

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

# ── Battery state ────────────────────────────────────────────────────────────
_battery_lock = threading.Lock()
_battery = {"level": 0.0, "voltage": 0.0}  # Battery percentage and voltage


def _on_battery_telemetry(data, topic=None, *args, **kwargs):
    """Battery telemetry callback - parse sensor_msgs/BatteryState from battery/status topic."""
    # print(f"[DEBUG] _on_battery_telemetry called with topic={topic}, data={data}")
    # Swap if parameters are reversed
    if isinstance(data, str) and isinstance(topic, dict):
        data, topic = topic, data
    
    if not isinstance(data, dict):
        print("[DEBUG] Ignored: data is not a dict")
        return
    
    # Accept if topic contains 'battery/status', or if topic is None but data looks like battery telemetry
    if not ((topic and "battery/status" in topic) or (topic is None and "percentage" in data)):
        print(f"[DEBUG] Ignored: topic does not contain 'battery/status' and data does not look like battery telemetry (topic={topic})")
        return
    # print(f"[DEBUG] Processing battery/status message: {data}")
    # sensor_msgs/BatteryState has a 'percentage' field (0-100)
    percentage = data.get("percentage")
    voltage = data.get("voltage")
    
    if percentage is not None:
        try:
            level_float = float(percentage)
            # If value is <= 1.0, treat as fraction and convert to percent
            if level_float <= 1.0:
                level_float *= 100.0
            with _battery_lock:
                _battery["level"] = level_float
                if voltage is not None:
                    try:
                        _battery["voltage"] = float(voltage)
                    except (ValueError, TypeError):
                        pass
            # print(f"[Battery] Level: {level_float:.1f}%, Voltage: {_battery['voltage']:.2f}V")
            pass
        except (ValueError, TypeError):
            print(f"[DEBUG] Failed to parse percentage: {percentage}")


def _on_lidar_scan(data, topic=None, *args, **kwargs):
    """Lidar subscription callback - parse and update scan data."""
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
    """Position subscription callback - parse and update robot x/y."""
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
    """Rotation subscription callback - parse and update robot yaw."""
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
cw.affect("simulation")
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

# Subscribe to battery telemetry

# Subscribe to battery telemetry via MQTT topic directly (sensor_msgs/BatteryState)
if hasattr(cw, 'mqtt') and hasattr(cw.mqtt, 'subscribe'):
    battery_topic = f"cyberwave/twin/{TWIN_UUID}/battery/status"
    cw.mqtt.subscribe(battery_topic, _on_battery_telemetry)
    print(f"Subscribed to battery topic: {battery_topic}")
else:
    ugv.subscribe(_on_battery_telemetry)
    print("Subscribed to battery telemetry via ugv.subscribe (fallback)")


print("Connected to UGV twin. Subscribing to position/rotation...")
print(f"Connected to Lidar twin {TWIN_LIDAR_UUID}. Subscribing to scans...")
print("Subscribing to battery telemetry...")


# --- Periodically trigger battery status update ---
import time
def _battery_check_loop():
    battery_cmd_topic = f"cyberwave/twin/{TWIN_UUID}/command"
    battery_cmd_payload = {
        "command": "battery_check",
        "data": {},
        "source_type": "tele"
    }
    while True:
        if hasattr(cw, 'mqtt') and hasattr(cw.mqtt, 'publish'):
            # print(f"[DEBUG] Publishing battery_check command to {battery_cmd_topic} with payload {battery_cmd_payload}")
            cw.mqtt.publish(battery_cmd_topic, json.dumps(battery_cmd_payload))
        else:
            # print("[DEBUG] MQTT publish not available on cw.mqtt")
            pass
        time.sleep(15)

_battery_check_thread = threading.Thread(target=_battery_check_loop, daemon=True)
_battery_check_thread.start()

# ── Drag state ───────────────────────────────────────────────────────────────
_drag: dict = {"active": False, "start": None, "end": None}
_last_goal: dict | None = None   # {"x": ..., "y": ..., "yaw": ...}


def _on_mouse(event, x, y, flags, param):
    global _last_goal
    is_ctrl_down = flags & cv2.EVENT_FLAG_CTRLKEY
    is_shift_down = flags & cv2.EVENT_FLAG_SHIFTKEY
    is_alt_down = flags & cv2.EVENT_FLAG_ALTKEY
    
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
            print(tx)
            print(ty)
            # Also print heading
            print(f"Heading hint: dx={ex-sx}, dy={ey-sy}")
            
            api_x = ty + 3.3
            api_y = -tx - 2.6
            
            # Build quaternion from heading
            dx, dy = ex - sx, ey - sy
            heading_yaw = math.atan2(dx, dy)
            # Reverse the calibration rotation by subtracting π/2 (inverse of the forward +π/2)
            api_yaw = heading_yaw - math.pi / 2
            
            half_yaw = api_yaw / 2.0
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
                    f"heading={math.degrees(api_yaw):+.1f}°"
                )
            except Exception as e:
                print(f"Error publishing initialpose: {e}")
        elif is_shift_down:
            # Shift+Click: Set goal pose via goto
            # Reverse the calibration transform
            api_x = ty + 3.3
            api_y = -tx - 2.6
            
            # Build quaternion from heading
            dx, dy = ex - sx, ey - sy
            ui_yaw = math.atan2(dx, dy)
            # Retain purely UI-based unrotated angle for the rendering
            _last_goal = {"x": tx, "y": ty, "yaw": ui_yaw}
            
            # Apply the -pi/2 offset to map rotation
            api_yaw = ui_yaw - math.pi / 2
            
            resp = ugv.navigation.goto([api_x, api_y, api_yaw], source_type="tele")
            print(
                f"→ goto ({api_x:+.3f}, {api_y:+.3f}) "
                f"heading={math.degrees(api_yaw):+.1f}°  |  resp: {resp}"
            )
        elif is_alt_down:
            # Alt+Click: Print world coordinates and heading hint for AprilTag placement
            dx, dy = ex - sx, ey - sy
            print(f"-- AprilTag Placement --\n"
                  f"\"x\": {tx:.3f},\n"
                  f"\"y\": {ty:.3f},\n"
                  f"\"heading_hint\": {{\"dx\": {dx}, \"dy\": {dy}}}")
            
        _drag["start"] = _drag["end"] = None


# ── Drawing helpers ──────────────────────────────────────────────────────────
_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_COL_ROBOT = (0, 0, 0)        # black
_COL_GOAL  = (0, 140, 255)      # orange
_COL_GRID  = (180, 60, 60)      # slate-blue - visible on both white & black map
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
        cv2.line(frame, (px, 0), (px, DISP_H - 1), colour, 2)
        if i != 0:
            cv2.putText(frame, f"y={i*step_m:g}", (px + 2, 12),
                        _FONT, 0.6, _COL_GRID, 1, cv2.LINE_AA)

    # Horizontal lines: constant world-x → constant image-y
    wx_range = math.ceil(MAP_H_M / 2 / step_m)
    for i in range(-wx_range, wx_range + 1):
        _, py = world_to_px(i * step_m, 0)   # world-x in 1st arg
        colour = _COL_AXIS if i == 0 else _COL_GRID
        cv2.line(frame, (0, py), (DISP_W - 1, py), colour, 2)
        if i != 0:
            cv2.putText(frame, f"x={i*step_m:g}", (2, py - 2),
                        _FONT, 0.6, _COL_GRID, 1, cv2.LINE_AA)

    # Origin cross-hair
    ox, oy = world_to_px(0, 0)
    cv2.drawMarker(frame, (ox, oy), _COL_AXIS, cv2.MARKER_CROSS, 14, 1)
    cv2.putText(frame, "0", (ox + 3, oy - 3), _FONT, 0.32, _COL_AXIS, 1, cv2.LINE_AA)


def _draw_robot(frame: np.ndarray):
    with _robot_lock:
        rx, ry, ryaw = _robot.get("x", 0.0), _robot.get("y", 0.0), _robot.get("yaw", 0.0)
        camera_yaw = _robot.get("camera_yaw", None)
        
    px, py = world_to_px(rx, ry)
    r = 10
    
    # Draw Camera field of view / orientation (orange arrow)
    if camera_yaw is not None:
        total_yaw = ryaw + camera_yaw
        
        # Camera arrow
        cam_ex = int(px + r * 3.5 * math.sin(total_yaw))
        cam_ey = int(py + r * 3.5 * math.cos(total_yaw))
        # Draw camera direction (Orange)
        cv2.arrowedLine(frame, (px, py), (cam_ex, cam_ey), (0, 165, 255), 2, tipLength=0.25)
        
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
        rx, ry, ryaw = _robot.get("x", 0.0), _robot.get("y", 0.0), _robot.get("yaw", 0.0)
    
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
    """Composite the Cyberwave logo and circle logo into the top-right corner."""
    if _LOGO_BGRA is None:
        return
    
    lh, lw = _LOGO_BGRA.shape[:2]
    x1 = frame.shape[1] - lw - _LOGO_MARGIN
    y1 = _LOGO_MARGIN
    x2, y2 = x1 + lw, y1 + lh
    
    # Clamp to frame bounds
    if x1 < 0 or y1 < 0 or x2 > frame.shape[1] or y2 > frame.shape[0]:
        return
    
    # Draw main Cyberwave logo
    logo_bgr   = _LOGO_BGRA[:, :, :3].astype(np.float32)
    alpha_mask = (_LOGO_BGRA[:, :, 3] / 255.0).astype(np.float32)[:, :, np.newaxis]
    roi = frame[y1:y2, x1:x2].astype(np.float32)
    blended = (logo_bgr * alpha_mask + roi * (1.0 - alpha_mask)).astype(np.uint8)
    frame[y1:y2, x1:x2] = blended
    
    # Draw circle logo underneath (if available)
    if _LOGO_CIRCLE_BGRA is not None:
        ch, cw = _LOGO_CIRCLE_BGRA.shape[:2]
        # Right-align circle logo to match main logo
        circle_x1 = frame.shape[1] - cw - _LOGO_MARGIN
        circle_y1 = y2 + _LOGO_MARGIN  # Position below main logo
        circle_x2 = circle_x1 + cw
        circle_y2 = circle_y1 + ch
        
        # Clamp to frame bounds
        if circle_x1 >= 0 and circle_y1 >= 0 and circle_x2 <= frame.shape[1] and circle_y2 <= frame.shape[0]:
            circle_bgr   = _LOGO_CIRCLE_BGRA[:, :, :3].astype(np.float32)
            circle_alpha = (_LOGO_CIRCLE_BGRA[:, :, 3] / 255.0).astype(np.float32)[:, :, np.newaxis]
            circle_roi = frame[circle_y1:circle_y2, circle_x1:circle_x2].astype(np.float32)
            circle_blended = (circle_bgr * circle_alpha + circle_roi * (1.0 - circle_alpha)).astype(np.uint8)
            frame[circle_y1:circle_y2, circle_x1:circle_x2] = circle_blended



def _draw_zone_mask(frame: np.ndarray, zone_data: dict):
    """Composite zone mask with 40% opacity underneath the robot marker."""
    mask_scaled = zone_data["mask_scaled"]
    if mask_scaled is None or mask_scaled.shape[:2] != (DISP_H, DISP_W):
        return
    
    # Extract color channels and create a mask of active pixels
    if mask_scaled.ndim == 3:
        if mask_scaled.shape[2] >= 3:
            mask_bgr = mask_scaled[:, :, :3].astype(np.float32)
            zone_pixels = np.any(mask_scaled[:, :, :3] > 1, axis=2)
        else:
            mask_bgr = cv2.cvtColor(mask_scaled[:, :, 0], cv2.COLOR_GRAY2BGR).astype(np.float32)
            zone_pixels = mask_scaled[:, :, 0] > 1
    else:
        # Grayscale: convert to BGR
        mask_bgr = cv2.cvtColor(mask_scaled, cv2.COLOR_GRAY2BGR).astype(np.float32)
        zone_pixels = mask_scaled > 1
    
    # Apply 40% opacity blend
    zone_opacity = 0.4
    frame_f = frame.astype(np.float32)
    blended = (mask_bgr * zone_opacity + frame_f * (1.0 - zone_opacity)).astype(np.uint8)
    
    # Apply blended color only where zone pixels are non-black
    frame[zone_pixels] = blended[zone_pixels]


def _draw_hud(frame: np.ndarray):
    """Bottom-left status / legend bar, zone name, and battery indicator."""
    h = frame.shape[0]
    lines = [
        "Shift+drag -> goto target",
        "Ctrl+drag -> set initial pose",
        "Alt+drag   -> print AprilTag info",
        "S = stop robot   Q / Esc = quit",
    ]
    for i, txt in enumerate(reversed(lines)):
        cv2.putText(frame, txt, (6, h - 12 - 22 * i),
                    _FONT, 0.7, _COL_HUD, 2, cv2.LINE_AA)
    

    # Display zone name if robot is in a zone
    y_pos = 40
    if _current_zone is not None:
        zone_name = _current_zone.get("name", "Unknown")
        cv2.putText(frame, f"Zone: {zone_name}", (6, y_pos),
                    _FONT, 1.0, (50, 200, 100), 2, cv2.LINE_AA)
        y_pos += 20

    # Draw large battery bar under the zone label
    bar_w, bar_h = 220, 32
    x_pos, y_pos_bat = 6, y_pos + 8
    # Background
    cv2.rectangle(frame, (x_pos, y_pos_bat), (x_pos + bar_w, y_pos_bat + bar_h), (60, 60, 60), -1)
    # Border
    cv2.rectangle(frame, (x_pos, y_pos_bat), (x_pos + bar_w, y_pos_bat + bar_h), (150, 150, 150), 2)
    with _battery_lock:
        battery_level = _battery["level"]
    if battery_level > 50:
        battery_color = (0, 255, 0)  # Green
    elif battery_level > 25:
        battery_color = (0, 165, 255)  # Orange
    elif battery_level > 0:
        battery_color = (0, 0, 255)  # Red
    else:
        battery_color = (100, 100, 100)  # Gray (no data)
    if battery_level > 0:
        fill_w = int(bar_w * battery_level / 100.0)
        cv2.rectangle(frame, (x_pos, y_pos_bat), (x_pos + fill_w, y_pos_bat + bar_h), battery_color, -1)
    label = f"Battery: {battery_level:.0f}%" if battery_level > 0 else "Battery: --"
    cv2.putText(frame, label, (x_pos + 12, y_pos_bat + bar_h - 8), _FONT, 0.85, (255, 255, 255), 2, cv2.LINE_AA)


# ── Main loop ────────────────────────────────────────────────────────────────
def main():
    WIN = "Map Navigator"

    # Get screen dimensions and ensure window fits (leaving room for taskbar)
    try:
        import ctypes
        screen_width = ctypes.windll.user32.GetSystemMetrics(0)
        screen_height = ctypes.windll.user32.GetSystemMetrics(1)
        # Leave margin for taskbar (~60px) and window decorations (~40px)
        max_window_height = screen_height - 100
        if DISP_H > max_window_height:
            scale = max_window_height / DISP_H
            window_w = int(DISP_W * scale)
            window_h = int(DISP_H * scale)
        else:
            window_w, window_h = DISP_W, DISP_H
    except Exception:
        # Fallback if screen detection fails
        window_w, window_h = DISP_W, DISP_H

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, window_w, window_h)
    cv2.setMouseCallback(WIN, _on_mouse)

    print(f"Map: {_SRC_W}x{_SRC_H} px → {DISP_W}x{DISP_H} px  "
          f"({MAP_W_M} x {MAP_H_M} m).  Drag on window to navigate. Press Q to quit.")

    try:
        while True:
            frame = cv2.resize(_BASE_IMG, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
            _draw_grid(frame)
            
            # Check and draw zone
            with _robot_lock:
                rx, ry = _robot["x"], _robot["y"]
            global _current_zone
            _current_zone = _check_zone(rx, ry)
            if _current_zone is not None:
                _draw_zone_mask(frame, _current_zone)
            
            _draw_last_goal(frame)
            _draw_drag_arrow(frame)
            _draw_lidar_scan(frame)
            _draw_apriltags(frame)  # Draw AprilTags
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

if __name__ == "__main__":
    main()
