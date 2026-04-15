import threading
import time

try:
    import cv2
    import numpy as np
except ImportError:
    print("Please install opencv-python and numpy: pip install opencv-python numpy")
    exit(1)

from cyberwave import Cyberwave


def main():
    # 1. Initialize and connect to Cyberwave
    cw = Cyberwave()
    cw.affect("live")

    print("Connecting to UGV Beast...")
    ugv_beast = cw.twin(
        "waveshare/ugv-beast",
        twin_id="40797a0a-9824-4fec-bf24-b4bd4757a13c",
        environment_id="7423dae7-0587-4ad9-892a-6144fb9838f3"
    )

    window_name = "UGV Live Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Track the latest frame via MQTT subscriber bypass
    latest_frame = {"bytes": None}
    
    def on_mqtt_message(data, topic=None, *args, **kwargs):
        # Normalize callback signatures
        if isinstance(data, str) and isinstance(topic, dict):
            data, topic = topic, data
        elif isinstance(data, (list, tuple)) and len(data) == 2:
            a0, a1 = data
            if isinstance(a0, dict) and isinstance(a1, str):
                data, topic = a0, a1

        # Check if this is the photo payload (big base64 string)
        import base64
        if isinstance(data, dict):
            # The structure might be {"image": "..."} or {"data": {"image": "..."}}
            image_b64 = None
            if "image" in data:
                image_b64 = data["image"]
            elif "data" in data and isinstance(data["data"], dict) and "image" in data["data"]:
                image_b64 = data["data"]["image"]
                
            if isinstance(image_b64, str) and len(image_b64) > 1000:
                # Strip prefix if present
                if image_b64.startswith("data:image/jpeg;base64,"):
                    image_b64 = image_b64.replace("data:image/jpeg;base64,", "")
                try:
                    latest_frame["bytes"] = base64.b64decode(image_b64)
                    return
                except Exception as e:
                    print("Failed to decode base64 MQTT image:", e)

    ugv_beast.subscribe(on_mqtt_message)

    print("Starting video stream (press 'q' to quit)...")

    stop_event = threading.Event()
    
    def video_loop():
        failure_count = 0
        while not stop_event.is_set():
            try:
                # Issue the command in the background (ignore timeout since we catch it in on_mqtt_message)
                threading.Thread(target=lambda: [ugv_beast.camera.edge_photo(format="bytes", timeout=0.1) for _ in [1] if not stop_event.is_set()], daemon=True).start()
                
                # Wait for the frame to arrive via MQTT subscription
                time.sleep(0.1)
                jpg_bytes = latest_frame["bytes"]
                
                if not jpg_bytes:
                    jpg_bytes = ugv_beast.camera.read(format="bytes", sensor_id="/image_raw")
                    if jpg_bytes and failure_count % 30 == 0:
                        print("Success path: REST API camera.read(/image_raw)")
                
                if not jpg_bytes:
                    failure_count += 1
                    time.sleep(0.1)
                    continue

                # Check if it's a valid JPEG (starts with 0xFFD8)
                if not jpg_bytes.startswith(b"\xff\xd8"):
                    failure_count += 1
                    time.sleep(0.1)
                    continue

                # Decode the raw JPEG bytes into a numpy array (BGR for OpenCV)
                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                if img is not None:
                    failure_count = 0
                    cv2.imshow(window_name, img)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        stop_event.set()
                        break
                else:
                    time.sleep(0.05)

            except Exception as e:
                print(f"Stream error: {e}")
                time.sleep(0.5)

    # Start stream in background thread so we could do other twin commands in main thread
    v_thread = threading.Thread(target=video_loop, daemon=True)
    v_thread.start()

    try:
        # Keep main thread alive while video window is open
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()
        
    v_thread.join(timeout=2.0)
    cv2.destroyAllWindows()
    cw.disconnect()
    print("Disconnected.")

if __name__ == "__main__":
    main()
