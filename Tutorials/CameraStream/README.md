# Camera Stream

This tutorial demonstrates how to stream the live camera feed from the UGV Beast digital twin using the Cyberwave API and OpenCV.

## Prerequisites

1.  Make sure you have cyberwave properly configured in your python environment.
2.  Install the required dependencies:
    ```bash
    pip install opencv-python numpy
    ```

## Usage

Run the [CameraStream.py](./CameraStream.py) script:

```bash
python CameraStream/CameraStream.py
```

### Explanation
We use `twin.camera.edge_photo(format="bytes", timeout=2.0)` to bypass the cloud WebRTC/REST cache.
When invoked over MQTT, this command directly asks the edge device for the latest raw JPEG bytes, which are then decoded and displayed with OpenCV in a background thread.
