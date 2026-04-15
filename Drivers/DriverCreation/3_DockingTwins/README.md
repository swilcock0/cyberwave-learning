# Tutorial 3: Docking Twins in Environments

Now that you have your **Driver** ([1. Creating a driver](../1_CreatingADriver/README.md)) and your **Catalogue Asset** ([2. Creating a catalogue asset](../1_CreatingACatalogueAsset/README.md)), it is time to bring them to life in an Environment. This tutorial explains how to instantiate your asset, "dock" it to a parent robot, and link the driver container. I'm starting with my existing UGV environment, where the UGV twin has been connected to a real robot following [UGV Beast - Get started](https://docs.cyberwave.com/hardware/ugv/get-started).

For more information on Environment editing, see [Environment Editor](https://docs.cyberwave.com/use-cyberwave/environment-editor).
For more information on Drivers metadata, see [Drivers](https://docs.cyberwave.com/edge/drivers/overview)

## Overview

In Cyberwave, devices rarely exist in isolation. A Lidar is usually "docked" to a UGV or a static mount. This physical relationship in the 3D world determines:
1.  **Coordinate Transformations (TF)**: Where the sensor data is relative to the robot's center.
2.  **Deployment Hierarchy**: When you deploy a robot to the Edge, its docked "child" devices are deployed automatically.

---

## Step 1: Instantiate the Asset in an Environment

Before you can dock a device, you must create an instance of it.
1.  Open the **Cyberwave Catalogue** and select your newly created Asset.
2.  At the top, select "Use" -> Existing Environment.
3.  Find your existing environment and it should spawn at the origin!

![Add asset](/resources/Catalogue4.png)

---

## Step 2: Docking to a Parent Twin

To give your Twin a location, you "dock" it onto a parent object (like a robot chassis).

1.  Select your newly created Lidar Twin in the 3D view or the Scene Tree.
2.  Ensure that "Edit" mode is selected at the top.
2.  In the **Properties Panel**, look for the **Dock to twin** section.
3.  Select the **Parent Twin** (e.g., "UGV Beast") and the specific **Link** you want to attach to (e.g., `base_link`).
4.  **Transformation Offset**: Adjust the relative XYZ/RPY values if the physical mounting point differs from the default URDF origin. Scale can also be modified here. Its easy enough to drag the twin around, although for accuracy you may wish to measure or look at manufacturer supplied
5.  **Save Changes**: The Lidar will now "follow" the robot wherever it moves in the digital world.

![Docked to parent](/resources/Docking.png)

---

## Step 3: Linking the Driver

Finally, associate your Asset with the Docker Image you created in [Tutorial 1](../1_CreatingADriver/README.md).

1.  Select your asset and scroll down to the bottom of the properties.
2.  Edit the metadata json like this example:

```json
{
  "drivers": {
    "default": {
      "docker_image": "registry.gitlab.com/swilcock0/d500-lidar-driver",
      "version": "0.0.1",
      "params": ["--device", "/dev/ttyUSB0:/dev/ttyUSB0"]
    }
  }
}
```

(Of course, replacing the docker registry link and params with those pertinent to your own driver)

![Metadata editing](/resources/Metadata2.png)

---

## Next Steps

- [4. Using the device on the local machine](/Drivers/DriverCreation/4_TestingConnection/README.md)
- [Drivers](https://docs.cyberwave.com/edge/drivers/overview)
- [Cyberwave SDK Documentation](https://docs.cyberwave.com)
- [cyberwave-edge-d500-lidar-driver](https://github.com/swilcock0/cyberwave-edge-d500-lidar-driver)