from cyberwave import Cyberwave
import time
import math
import threading

# Optional: OpenCV for displaying video and numpy for decoding JPEG bytes
try:
    import cv2
    import numpy as np
    import os
    
    # Setup AprilTag Dictionary (36h11 is standard)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        aruco_params = cv2.aruco.DetectorParameters()
        
        # Increase the resolution of the adaptive thresholding
        aruco_params.adaptiveThreshWinSizeMin = 3
        aruco_params.adaptiveThreshWinSizeMax = 23
        aruco_params.adaptiveThreshWinSizeStep = 10

        # AprilTag specific refinement (if your OpenCV version supports it)
        # This is often more robust for AprilTags than SUBPIX
        try:
            aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        except:
            aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        # Try lowering this to accept slightly distorted quads
        aruco_params.maxErroneousBitsInBorderRate = 0.8
        
        # Robustness: Accept more deformed/jagged quads (default is 0.03)
        # This helps when interpolation makes the straight lines look wavy/jagged
        aruco_params.polygonalApproxAccuracyRate = 0.055
        
        # Robustness: Catch skinnier/smaller tags that get compressed at the edges
        aruco_params.minMarkerPerimeterRate = 0.015
        
        # Robustness: Boost the internal payload error correction to recover blurry bits
        aruco_params.errorCorrectionRate = 0.9
        
        if hasattr(cv2.aruco, 'ArucoDetector'):
            aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        else:
            aruco_detector = None
    except AttributeError:
        aruco_dict = None
        aruco_params = None
        aruco_detector = None
        print("cv2.aruco not available. AprilTag detection disabled.")
        
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
                    # Display image without distortion correction
                    display_img = img.copy()
                        
                    # --- AprilTag Detection ---
                    if aruco_dict is not None:
                        gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
                        
                        # Apply CLAHE to boost contrast
                        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
                        gray_boost = clahe.apply(gray)
                        
                        # Apply Unsharp Mask to sharpen edges
                        blur = cv2.GaussianBlur(gray_boost, (0, 0), 3)
                        sharp_gray = cv2.addWeighted(gray_boost, 2.0, blur, -1.0, 0)

                        if aruco_detector is not None:
                            corners, ids, rejected = aruco_detector.detectMarkers(sharp_gray)
                        else:
                            corners, ids, rejected = cv2.aruco.detectMarkers(sharp_gray, aruco_dict, parameters=aruco_params)
                    else:
                        ids = None

                    if ids is not None and len(ids) > 0:
                        # Draw detected markers
                        cv2.aruco.drawDetectedMarkers(display_img, corners)
                        
                        # Draw tag numbers in white
                        for i in range(len(ids)):
                            corners_pts = corners[i][0].astype(int)
                            center_x = int(np.mean(corners_pts[:, 0])+20)
                            center_y = int(np.mean(corners_pts[:, 1]))
                            cv2.putText(display_img, str(ids[i][0]), (center_x - 10, center_y + 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        
                        # Pose estimation with default camera matrix (no calibration)
                        # Calculate focal length from camera FOV (170 degrees horizontal)
                        h, w = display_img.shape[:2]
                        fov_degrees = 118.0
                        fov_radians = math.radians(fov_degrees / 2.0)
                        focal_length = (w / 2.0) / math.tan(fov_radians)
                        center = (w / 2.0, h / 2.0)
                        camera_matrix = np.array([
                            [focal_length, 0, center[0]],
                            [0, focal_length, center[1]],
                            [0, 0, 1]
                        ], dtype=np.float32)
                        
                        tag_size = 0.2
                        obj_pts = np.array([
                            [-tag_size/2,  tag_size/2, 0],
                            [ tag_size/2,  tag_size/2, 0],
                            [ tag_size/2, -tag_size/2, 0],
                            [-tag_size/2, -tag_size/2, 0]
                        ], dtype=np.float32)
                        
                        zero_dist = np.zeros((4, 1), dtype=np.float32)
                        for i in range(len(ids)):
                            success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i][0], camera_matrix, zero_dist)
                            if success:
                                # Draw custom axes: only X (red) and Y (green), no Z (blue)
                                axis_length = tag_size / 2
                                axis_points_3d = np.array([
                                    [0, 0, 0],                    # Origin
                                    [axis_length, 0, 0],          # X-axis (red)
                                    [0, axis_length, 0]           # Y-axis (green)
                                ], dtype=np.float32)
                                
                                axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, camera_matrix, zero_dist)
                                axis_points_2d = axis_points_2d.astype(int)
                                
                                origin = tuple(axis_points_2d[0][0])
                                x_end = tuple(axis_points_2d[1][0])
                                y_end = tuple(axis_points_2d[2][0])
                                
                                # Draw X-axis (red)
                                cv2.line(display_img, origin, x_end, (0, 0, 255), 2)
                                # Draw Y-axis (green)
                                cv2.line(display_img, origin, y_end, (0, 255, 0), 2)
                                
                                dist = np.linalg.norm(tvec)
                                cv2.putText(display_img, f"Dist: {dist:.2f}m", 
                                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    cv2.imshow(window_name, display_img)
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