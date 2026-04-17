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
python Tutorials/CameraStream/CameraStream.py
```

### Explanation
We use `twin.camera.edge_photo(format="bytes", timeout=3.0)` to bypass the cloud WebRTC/REST cache.
When invoked over MQTT, this command directly asks the edge device for the latest raw JPEG bytes. In the background thread, OpenCV dynamically decodes those bytes and uses the loaded calibration map parameters (`cv2.remap()`) to undistort the extreme fisheye curvature of the raw frame before displaying it on screen.


### Camera Calibration
When started, the script checks for a local `camera_calib.npz` array container. If found, it parses the projection matrices and prints out the calibration data in ROS `camera_info` standard format before starting the video stream window:

```text
--- ROS camera_info ---
height: 480
width: 640
distortion_model: fisheye
D: [0.02362927647301538, 0.010092809609273667, -0.02946640913386099, 0.00827924994219285]
K: [266.03143310546875, 0.0, 307.1657409667969, 0.0, 264.33270263671875, 244.2731475830078, 0.0, 0.0, 1.0]
-----------------------
--- Distorted to Undistorted Info ---
New K: [153.14744567871094, 0.0, 283.3395690917969, 0.0, 152.16954040527344, 245.83091735839844, 0.0, 0.0, 1.0]
Note: Use 'New K' and zero distortion for PnP/AprilTags on the undistorted frame.
-----------------------------------
```

This undistortion process gets rid of the curvy distortion inherent in the wide-angle lens. The amount of black space around the processed image is dictated by the `balance` parameter in the OpenCV configuration.

| Before Calibration (Distorted) | After Calibration (Undistorted) |
|:---:|:---:|
| ![Distorted Camera](/resources/Camera_Distorted.png) | ![Undistorted Camera](/resources/Camera_Undistort.png) |

### Computer Vision (AprilTags & Pose Estimation)
When processing the **undistorted** image for pose estimation (like calculating 3D distance to AprilTags using `cv2.solvePnP`), the optical properties of the image have been fundamentally changed. The original camera matrix and raw distortion coefficients are **no longer valid**. 

If you are running 3D projection mathematical algorithms on the **undistorted** image, you must:
1. Apply the dynamically calculated **New Camera Matrix** (which the Python script prints to the terminal interface at startup).
2. Set your distortion coefficients array directly to **Zero** (e.g., `np.zeros((4, 1), dtype=np.float32)`). Since the physical lens distortion has already been computationally removed, applying the old distortion correction factors again would ruin the 3D calculation.
