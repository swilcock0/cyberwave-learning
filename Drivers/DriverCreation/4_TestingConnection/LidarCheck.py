from cyberwave import Cyberwave
import time
import math
import threading

from config import TWIN_UUID, TWIN_LIDAR_UUID, ENVIRONMENT_UUID

import matplotlib.pyplot as plt
import numpy as np

cw = Cyberwave()
cw.affect("live")

d500 = cw.twin(
    twin_id=TWIN_LIDAR_UUID,
    environment_id=ENVIRONMENT_UUID
)

# Get twin pose etc metadata to understand the attachment of the lidar to the UGV
print("Attachment relative to UGV Beast\n===============================")
print("Position XYZ : ", d500._data.position_x, d500._data.position_y, d500._data.position_z)
print("Rotation WXYZ : ", d500._data.rotation_w, d500._data.rotation_x, d500._data.rotation_y, d500._data.rotation_z)

# Assuming 'd500' is your twin object
metadata = d500._data.metadata  # or d500._data.get('metadata') if it's a dict

# Find the joint in the joints list
joints = metadata.get('joints', [])
lidar_joint = next((j for j in joints if j['name'] == 'lidar_base_to_optical_joint'), None)

if lidar_joint:
    z_offset = lidar_joint['pose']['position']['z']
    print(f"Lidar Optical Z offset: {z_offset}") # Result: 0.027


# Setup Matplotlib Polar Plot
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
# Use scatter for intensity-based coloring
sc = ax.scatter([], [], c=[], cmap='viridis', s=5, vmin=0, vmax=255)
ax.set_rmax(12.0)  # Max range of D500
ax.set_rticks([2, 4, 6, 8, 10, 12])
ax.set_title("Live Lidar Scan with Intensity (D500)", va='bottom')
plt.colorbar(sc, label='Intensity')

scan_lock = threading.Lock()
current_scan = {"angles": [], "ranges": [], "intensities": []}

def message_handler(data, topic=None, *args, **kwargs):
    # Normalize callback signatures
    if isinstance(data, str) and isinstance(topic, dict):
        data, topic = topic, data

    if topic and "/scan" in topic:
        ranges = data.get("ranges", [])
        intensities = data.get("intensities", [])
        angle_min = data.get("angle_min", 0.0)
        angle_increment = data.get("angle_increment", 0.0)
        
        if ranges:
            angles = [angle_min + i * angle_increment for i in range(len(ranges))]
            with scan_lock:
                current_scan["angles"] = angles
                current_scan["ranges"] = ranges
                current_scan["intensities"] = intensities
        return

    # Fallback prints
    if "type" in data and data["type"] == "edge_health":
        return # Skip health noise for now
    
    if topic:
        print(f"Topic: {topic}")
    else:
        print("Data received")

d500.subscribe(message_handler)

while True:
    with scan_lock:
        if current_scan["angles"]:
            # Update scatter plot with points and intensity colors
            # Convert to numpy for easier manipulation
            angles = np.array(current_scan["angles"])
            ranges = np.array(current_scan["ranges"])
            
            # Mask out zero-range values (errors/shadows)
            mask = ranges > 0.01
            
            if len(current_scan["intensities"]) == len(ranges):
                intensities = np.array(current_scan["intensities"])
                # Update positions
                offsets = np.column_stack([angles[mask], ranges[mask]])
                sc.set_offsets(offsets)
                # Update colors based on intensity
                sc.set_array(intensities[mask])
            else:
                # Fallback if no intensities provided
                offsets = np.column_stack([angles[mask], ranges[mask]])
                sc.set_offsets(offsets)
            
            fig.canvas.draw()
            fig.canvas.flush_events()
    time.sleep(0.05)
    
cw.disconnect()
    
cw.disconnect()