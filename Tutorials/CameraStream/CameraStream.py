from cyberwave import Cyberwave
import time
import math
import threading

# Optional: OpenCV for displaying video and numpy for decoding JPEG bytes
try:
    import cv2
    import numpy as np
    import os
    
    calib_mtx = None
    calib_dist = None
    undistort_map = None
    
    calib_path = os.path.join(os.path.dirname(__file__), 'camera_calib.npz')
    if os.path.exists(calib_path):
        with np.load(calib_path) as calib:
            calib_mtx = calib['camera_matrix']
            calib_dist = calib['dist_coeffs']
            resolution = tuple(calib.get('resolution', (640, 480)))
            is_fisheye = calib.get('fisheye', False)
            
            print("--- ROS camera_info ---")
            print(f"height: {resolution[1]}")
            print(f"width: {resolution[0]}")
            print(f"distortion_model: {'fisheye' if is_fisheye else 'plumb_bob'}")
            print(f"D: {calib_dist.flatten().tolist()}")
            print(f"K: {calib_mtx.flatten().tolist()}")
            print("-----------------------")
            
            if is_fisheye:
                new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    calib_mtx, calib_dist, resolution, np.eye(3), balance=0.5)
                undistort_map = cv2.fisheye.initUndistortRectifyMap(
                    calib_mtx, calib_dist, np.eye(3), new_camera_matrix, resolution, cv2.CV_16SC2)
            else:
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    calib_mtx, calib_dist, resolution, 0.5, resolution)
                undistort_map = cv2.initUndistortRectifyMap(
                    calib_mtx, calib_dist, None, new_camera_matrix, resolution, 5)
            print("--- Distorted to Undistorted Info ---")
            print(f"New K: {new_camera_matrix.flatten().tolist()}")
            print("Note: Use 'New K' and zero distortion for PnP/AprilTags on the undistorted frame.")
            print("-----------------------------------")
except Exception as e:
    print(f"Failed to initialize OpenCV/calibration: {e}")
    cv2 = None
    np = None
from config import TWIN_UUID, ENVIRONMENT_UUID

cw = Cyberwave()
cw.affect("live")

ugv_beast = cw.twin(
    "waveshare/ugv-beast",
    twin_id=TWIN_UUID,
    environment_id=ENVIRONMENT_UUID
)


def _quat_to_yaw(q):
    x = q.get("x", 0.0)
    y = q.get("y", 0.0)
    z = q.get("z", 0.0)
    w = q.get("w", 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _extract_pose(payload):
    # Accept either an odom-like dict (contains 'pose') or a wrapper with 'message'
    if not isinstance(payload, dict):
        return None
    odom = None
    if "pose" in payload and isinstance(payload["pose"], dict):
        odom = payload
    elif "message" in payload and isinstance(payload["message"], dict):
        inner = payload["message"]
        if "pose" in inner and isinstance(inner["pose"], dict):
            odom = inner
    if odom is None:
        return None
    pose_block = odom.get("pose", {}).get("pose") if isinstance(odom.get("pose"), dict) else None
    if not pose_block:
        return None
    pos = pose_block.get("position", {})
    ori = pose_block.get("orientation", {})
    try:
        x = pos.get("x")
        y = pos.get("y")
        z = pos.get("z")
        yaw = _quat_to_yaw(ori)
        return {"x": x, "y": y, "z": z, "yaw": yaw}
    except Exception:
        return None


def message_handler(data, topic=None, *args, **kwargs):
    # Normalize callback signatures: some call (topic, payload) or (payload, topic)
    # Handle swapped args where 'data' is a topic string and 'topic' is payload dict
    if isinstance(data, str) and isinstance(topic, dict):
        data, topic = topic, data

    # If we got a tuple/list like (payload, topic)
    if isinstance(data, (list, tuple)) and len(data) == 2:
        a0, a1 = data
        if isinstance(a0, dict) and isinstance(a1, str):
            data, topic = a0, a1

    # If topic not set, try to find it inside data or extra args
    if topic is None:
        if isinstance(data, dict) and "topic" in data and isinstance(data["topic"], str):
            topic = data.get("topic")
        else:
            for a in args:
                if isinstance(a, str) and "/" in a:
                    topic = a
                    break

    # Check if this is a large photo payload
    image_b64 = None
    if isinstance(data, dict):
        if "image" in data:
            image_b64 = data["image"]
        elif "data" in data and isinstance(data["data"], dict) and "image" in data["data"]:
            image_b64 = data["data"]["image"]
            
        if isinstance(image_b64, str) and len(image_b64) > 1000:
            import base64
            if image_b64.startswith("data:image/jpeg;base64,"):
                image_b64 = image_b64.replace("data:image/jpeg;base64,", "")
            try:
                latest_frame["bytes"] = base64.b64decode(image_b64)
                return
            except Exception as e:
                print("Failed to decode base64 MQTT image:", e)

    # Try to extract odometry pose
    pose = _extract_pose(data)
    if pose is not None:
        yaw_deg = math.degrees(pose["yaw"]) if pose.get("yaw") is not None else None
        if topic:
            print(f"Topic: {topic} | x={pose['x']:.3f}, y={pose['y']:.3f}, yaw_deg={yaw_deg:.2f}")
        else:
            print(f"Pose: x={pose['x']:.3f}, y={pose['y']:.3f}, yaw_deg={yaw_deg:.2f}")
        return

    # Fallback prints
    if "message" in data:
        if topic:
            print(f"Topic: {topic} | Message:", data["message"])
        else:
            print("Message:", data["message"])
    else:
        if topic:
            print(f"Topic: {topic} | Data:", data)
        else:
            print("Data:", data)

latest_frame = {"bytes": None}

def on_video_frame(data, topic=None, *args, **kwargs):
    """Receive JPEG frames pushed directly via MQTT /video topic."""
    import base64
    if isinstance(data, (bytes, bytearray)):
        latest_frame["bytes"] = bytes(data)
        return
    if isinstance(data, dict):
        image_b64 = data.get("image") or (data.get("data") or {}).get("image")
        if isinstance(image_b64, str) and len(image_b64) > 100:
            if image_b64.startswith("data:image/jpeg;base64,"):
                image_b64 = image_b64[len("data:image/jpeg;base64,"):]
            try:
                latest_frame["bytes"] = base64.b64decode(image_b64)
            except Exception as e:
                print("Video frame decode error:", e)

# ugv_beast.subscribe(message_handler)
ugv_beast.client.mqtt.subscribe_video_stream(ugv_beast.uuid, on_video_frame)
ugv_beast.navigation.follow_path([[0, 0, 0], [1, 0, 0], [1, 1, 0]])

ugv_beast.edit_position(x=0.0, y=0.0, z=-0.07)
# If OpenCV available, start a background thread that continuously
# fetches the latest JPEG frame and shows it in a window.
stop_event = threading.Event()
_fetch_logged = {"rest_ok": False, "rest_fail": 0}

def _fetch_loop():
    """Background thread: request frames from edge via MQTT take_photo command."""
    while not stop_event.is_set():
        try:
            raw_bytes = ugv_beast.camera.edge_photo(format="bytes", timeout=3.0)
            if raw_bytes and len(raw_bytes) > 200:
                latest_frame["bytes"] = raw_bytes
                if not _fetch_logged["rest_ok"]:
                    print(f"edge_photo OK, len={len(raw_bytes)}")
                    _fetch_logged["rest_ok"] = True
        except Exception as e:
            _fetch_logged["rest_fail"] += 1
            if _fetch_logged["rest_fail"] % 10 == 1:
                print(f"edge_photo error: {e}")
            time.sleep(0.5)  # back off only on error

fetch_thread = threading.Thread(target=_fetch_loop, daemon=True)
fetch_thread.start()

# OpenCV display runs on the main thread (required on Windows)
if cv2 is None or np is None:
    print("OpenCV or numpy not available; skipping video display.")
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
else:
    window_name = "UGV Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while True:
            raw_bytes = latest_frame["bytes"]
            if raw_bytes:
                arr = np.frombuffer(raw_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    if undistort_map is not None:
                        img = cv2.remap(img, undistort_map[0], undistort_map[1], cv2.INTER_LINEAR)
                    cv2.imshow(window_name, img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


stop_event.set()
try:
    fetch_thread.join(timeout=1)
except Exception:
    pass
if cv2 is not None:
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
cw.disconnect()