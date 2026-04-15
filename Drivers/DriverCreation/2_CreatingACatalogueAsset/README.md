# Tutorial 2: Creating a Catalogue Asset

Once you have a functional driver, the next step is to ensure your hardware is represented accurately in the Cyberwave Digital Twin. This involves creating a **Catalogue Asset** with a physical 3D model (URDF).

## Overview

A Catalogue Asset provides the metadata, visual model, and physical constraints for a device. This allows the Cyberwave platform to:
1.  **Visualize**: Render the device in the 3D dashboard.
2.  **Transform**: Calculate coordinate frames (TFs) relative to the robot's base.
3.  **Interact**: Run teleop controllers and similar.
4.  **Simulate**: Trial control operations in simulation (through a Mujoco service) before actually carrying out.


I've put here the [d500_description](Drivers/DriverCreation/2_CreatingACatalogueAsset/d500_description) folder I used to create an asset.

For reference, see the [Cyberwave Docs - Catalog](https://docs.cyberwave.com/use-cyberwave/catalog) (I totally didn't see this earlier and just kind of figured it out, so props to the team for making it intuitive!)


I've also made my version of this asset public, have a look in the asset catalogue [d500-lidar](https://cyberwave.com/sam-wilcocks-workspace/d500-lidar-2).

---

## Step 1: Create a URDF (Unified Robot Description Format)

The URDF is an XML file that describes the physical geometry and kinematic structure of your hardware ([urdf/d500.urdf](Drivers/DriverCreation/2_CreatingACatalogueAsset/d500_description/urdf/d500.urdf)).

### 1.1 Sourcing STLs/Meshes
Before writing XML, you need a 3D model.
*   Find or create an `.stl` or `.dae` (Collada) file for your hardware.
*   **Optimization**: Keep meshes simple (low poly) to ensure smooth browser rendering. For the D500 Lidar, we used the `LD19.stl` ([meshes/LD19.stl](Drivers/DriverCreation/2_CreatingACatalogueAsset/d500_description/meshes/LD19.stl)).
*   **Scaling**: Most CAD software exports in mm, but URDF expects meters. Use `<mesh filename="..." scale="0.001 0.001 0.001"/>` to scale down.

### 1.2 Defining the Base Link
The `base_link` is the primary mounting point of your hardware. This is where you define the visual mesh and the physical collision boundaries.

```xml
<link name="lidar_base_link">
  <visual>
    <geometry>
      <mesh filename="package://d500_description/meshes/LD19.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.04" radius="0.035"/>
    </geometry>
  </collision>
</link>
```

### 1.3 Adding Additional Frames (Links & Joints)
Hardware often has specific internal frames (like a Lidar's rotation plane or a Camera's optical center).
*   **Fixed Joints**: Used to define static offsets within the device.
*   **Example (D500)**: The sensor's mounting base is different from its optical center. We add a `lidar_optical_link` positioned 27mm above the base:

```xml
<link name="lidar_optical_link"/>
<joint name="lidar_base_to_optical_joint" type="fixed">
  <parent link="lidar_base_link"/>
  <child link="lidar_optical_link"/>
  <!-- 27mm offset on Z axis -->
  <origin xyz="0 0 0.027" rpy="0 0 0"/>
</joint>
```

---

## Step 2: Upload to Cyberwave Catalogue

Once your URDF and meshes are ready, you can register the asset in the platform. First, login to Cyberwave, and go to `Catalogue` -> `Upload Asset`

![Upload asset](/resources/Catalogue1.png)

Next, we can name our asset, and upload our zipped folder containing the urdf and meshes:

![Upload asset2](/resources/Catalogue2.png)

It should really be able to work out the urdf file location unless you have multiple .urdf files there, so leave that bit alone in "Advanced". I think it's worth adding a thumbnail though!

![Thumbnail](/Drivers/DriverCreation/2_CreatingACatalogueAsset/d500_description/snapshot.png)

### Step 3: Finalizing the Metadata
You can add additional metadata about the device as shown here:

![metadata](/resources/Catalogue3.png)

Personally I've not played with this too much yet, but as is shown in the example image, sensors can be set up, so presumably the plan is to make driver creation even more intuitive and plug'n'play in future!

---

## Next Steps
- [3. Docking twins in environments](/Drivers/DriverCreation/3_DockingTwins/README.md)
- [Modifying an Existing Driver](../../ModifyingDriverInPlace/README.md)
- [Catalogue](https://docs.cyberwave.com/use-cyberwave/catalog)
- [d500-lidar](https://cyberwave.com/sam-wilcocks-workspace/d500-lidar-2)