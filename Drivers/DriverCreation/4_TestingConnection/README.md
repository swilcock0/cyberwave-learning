# Driver Creation & End-to-End Validation

Once you have successfully created your Driver, defined your Catalogue Asset, and docked the Twins, it is time for the final validation: **Testing the Connection**.

## 1. Deploying to the Edge

After everything is configured in the Cyberwave Dashboard, you need to ensure your physical Edge node (e.g., Raspberry Pi, Jetson, or PC) is running the latest configuration.

I found it easiest to re-run the install command on the edge device:
```bash
sudo cyberwave edge install
```

This ensures the new environment setup, environment variables, and Docker containers are pulled. If you experience issues or have updated the container image without changing the version tag, you can force a refresh:
```bash
sudo cyberwave edge pull
sudo cyberwave edge restart
```

---

## 2. Validating with the SDK (`LidarCheck.py`)

At this stage, your driver is running on the hardware, publishing data to the **Cyberwave Cloud Layer**, and you are ready to consume it on your local machine using the Python SDK.

The [LidarCheck.py](Drivers/DriverCreation/4_TestingConnection/LidarCheck.py) script provides a complete template for real-time validation:

### Key Sections of the Test Script:
1.  **Metadata Inspection**: It reads the `_data` schema from the platform to verify the physical attachment. We can see the exact $Z$ offset (e.g., `0.027m`) defined in the URDF in Tutorial 2 ([2. Creating a catalogue asset](/Drivers/DriverCreation/2_CreatingACatalogueAsset/README.md)).
2.  **Live Subscription**: Using `d500.subscribe(message_handler)`, we listen for the `.../scan` topic.
3.  **Intensity Mapping**: The script uses a Matplotlib **Polar Scatter Plot**. It maps the `intensities` array to a colormap, allowing you to see which surfaces are more reflective (like metallic legs vs. dark carpet).
4.  **Real-time Visualization**: Since the driver is high-frequency, the script uses a `threading.Lock` to ensure the UI remains responsive while the data stream pours in.

![Real lidar](/resources/lidar_real.png) *(Happy UGV Beast with some new cloud connected hardware. And its got blue lights!)*

![Lidar Polar Plot](/resources/lidar_plot_example.png) *(Example of the live Lidar scan reaching a local machine)*

---

## 3. The Big Picture: Agentic Workflows

The most powerful part of this setup is that your hardware is no longer trapped on a local serial port. 

Because the driver is publishing via the **Cyberwave Cloud**, your data is now globally accessible. You can now:
*   **Trigger Workflows**: Use the scan data to trigger low-latency alerts or automation.
*   **Agentic Actions**: Feed these real-time streams into LLM-driven agents or navigation workflows.
*   **Remote Monitoring**: Monitor your robot's environment from across the world with the same latency as if you were standing next to it.

Congratulations—you've built a cloud-compatible hardware driver from scratch!

---

## Tutorial Series
1. [Creating a Driver](/Drivers/DriverCreation/1_CreatingADriver/README.md)
2. [Creating a Catalogue Asset](/Drivers/DriverCreation/2_CreatingACatalogueAsset/README.md)
3. [Docking Twins](/Drivers/DriverCreation/3_DockingTwins/README.md)
4. [Testing Connection](/Drivers/DriverCreation/4_TestingConnection/README.md)
