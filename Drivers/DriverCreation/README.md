# Cyberwave Driver Creation Series

This 4-part series takes you from a raw hardware SDK to a cloud-connected Digital Twin, using the **Waveshare D500 Lidar** as our primary example.

1.  **[Creating a Driver](1_CreatingADriver/README.md)**: Building the containerized Python driver and mapping C-structs via `ctypes`.
2.  **[Creating a Catalogue Asset](2_CreatingACatalogueAsset/README.md)**: Defining the physical representation (meshes and URDF) and uploading to the platform.
3.  **[Docking Twins](3_DockingTwins/README.md)**: Learning the physical relationship between twins and linking the Docker registry to the asset.
4.  **[Testing Connection](4_TestingConnection/README.md)**: Final end-to-end validation—pulling the driver to real edge hardware and monitoring live data from the cloud.

---

### The Big Picture
By the end of this series, your hardware is no longer trapped on a physical serial port. It is **Cloud Native**, allowing for remote monitoring, automation workflows, and agentic AI integration across the Cyberwave platform.
