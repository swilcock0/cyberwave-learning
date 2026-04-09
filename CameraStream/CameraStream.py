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

    # Frames are fetched via camera.edge_photo(), which sends a take_photo MQTT
    # command to the edge driver and receives the JPEG response on camera/photo.
    # This bypasses WebRTC and the REST /latest-frame endpoint entirely, and works
    # as long as the edge device is connected over MQTT.
    latest_frame = {"bytes": None}
    stop_event = threading.Event()

    def fetch_loop():
        """Request frames from the edge as fast as the MQTT round-trip allows."""
        while not stop_event.is_set():
            try:
                raw_bytes = ugv_beast.camera.edge_photo(format="bytes", timeout=3.0)
                if raw_bytes and len(raw_bytes) > 200:
                    latest_frame["bytes"] = raw_bytes
            except Exception as e:
                print(f"edge_photo error: {e}")
                time.sleep(0.5)  # back off only on error

    fetch_thread = threading.Thread(target=fetch_loop, daemon=True)
    fetch_thread.start()

    # OpenCV GUI must run on the main thread on Windows.
    print("Starting video stream (press 'q' to quit)...")
    window_name = "UGV Live Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            raw_bytes = latest_frame["bytes"]
            if raw_bytes:
                arr = np.frombuffer(raw_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.imshow(window_name, img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()

    fetch_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    cw.disconnect()
    print("Disconnected.")


if __name__ == "__main__":
    main()
