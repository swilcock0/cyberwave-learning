# Modifying Driver In-Place

This guide demonstrates how to make "thin edits" to Cyberwave edge driver images on the fly without performing a full Docker rebuild. This is particularly useful for rapid prototyping, changing parameters, or activating experimental components like the `NavigationBridge`. Of course, things might break when the driver image is updated!

## 📂 Overview

Instead of modifying the base `cyberwaveos/ugv-driver:dev` image, we use **Docker Volume Mounts** to inject local configuration files and source code directly into the running container.

### Key Components
- **[my_configs/](./my_configs/)**: Local directory structure containing the files to be injected.
- **[my_twin_uuid.json](./my_twin_uuid.json)**: An example digital twin configuration JSON showing the necessary volume mount parameters.

## 🛠️ Step-by-Step Guide

### 1. Prepare Local Files
Place your modified files in a specific directory on the edge device (e.g., `/home/ws/my_configs/`). The structure may match what you intend to mount (I like to do so). In this example, we have:
- `params.yaml`: Modified configuration parameters.
- `mqtt_bridge_node.py`: Experimental node logic.
- `ugv_beast_command_handler.py`: Custom command handling.

### 2. Configure Volume Mounts
To tell Cyberwave to mount these files, you must add `-v` (volume) flags to the driver's `params` array in the asset/twin configuration. The original file is found on the digital twin page as a metadata json, or under /etc/cyberwave (with the UUID of the twin as name). Note that you will need to use sudo to get the file.

**Example JSON snippet to adjust:**
```json
"drivers": {
  "default": {
    "params": [
      "--device /dev/ttyAMA0:/dev/ttyAMA0",
      "--device /dev/video0:/dev/video0",
      "-v", "/home/ws/my_configs/ugv/src/mqtt_bridge/config/params.yaml:/home/ws/ugv_ws/src/mqtt_bridge/config/params.yaml",
      "-v", "/home/ws/my_configs/ugv/src/mqtt_bridge/mqtt_bridge/mqtt_bridge_node.py:/home/ws/ugv_ws/src/mqtt_bridge/mqtt_bridge/mqtt_bridge_node.py",
      "-v", "/home/ws/my_configs/ugv/src/mqtt_bridge/mqtt_bridge/plugins/ugv_beast_command_handler.py:/home/ws/ugv_ws/src/mqtt_bridge/mqtt_bridge/plugins/ugv_beast_command_handler.py"
    ],
    "docker_image": "cyberwaveos/ugv-driver:dev"
  }
}
```

### 3. Apply the Configuration
On your Cyberwave Environment page, first select "Edit" at the top, and click on your Edge Twin. In the right-hand panel there will be an "Edit Metadata (Advanced)" button:

![Metadata button](/resources/MetaData.png)

Once there, paste your JSON file into the textbox, click "Save metadata":

![Metadata save](/resources/MetaDataAdvanced.png)

Finally, back in the edge terminal, run `sudo cyberwave edge restart` to pull the new data and load the files.

## 🚀 Benefits
- **No Build Time**: Changes take effect immediately upon container restart.
- **Safe Rollback**: Simply remove the volume mount in the JSON to revert to the original driver behavior.
- **Easy Debugging**: Edit files directly on the robot and restart the driver through the Cyberwave dashboard.


For more info, see the [Drivers docs page](https://docs.cyberwave.com/edge/drivers/overview)