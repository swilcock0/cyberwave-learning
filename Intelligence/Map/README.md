# Map Navigator

Interactive operator map for the UGV Beast twin.

The script in this folder renders the RTAB-Map floorplan, overlays live robot telemetry, lidar points, zone masks, and static AprilTag markers, and lets you send navigation commands directly from the map UI.

## Main Script

- `map_navigator.py`

## Features

- Real-time robot pose overlay (position + heading)
- Lidar scan projection into map coordinates
- Zone masking and active-zone label when the robot enters a zone
- Battery telemetry bar
- Click-drag navigation controls
- Initial-pose publishing via MQTT
- AprilTag map marker overlay + placement helper output

## Prerequisites

1. Install Python dependencies:

	```bash
	pip install cyberwave opencv-python numpy
	```

2. Ensure IDs are configured in `Experiments/config.py`:

	- `TWIN_UUID`
	- `TWIN_LIDAR_UUID`
	- `ENVIRONMENT_UUID`

3. Ensure map assets exist in this folder:

	- `rtabmap.png`
	- `CyberwaveLogo.png`
	- `LogoCircleBW.png`

## Run

From repository root:

```bash
python Intelligence/Map/map_navigator.py
```

## Controls

- Left mouse drag: set drag arrow origin/heading preview
- Shift + drag release: send `goto` target
- Ctrl + drag release: publish initial pose on MQTT `.../navigate/initialpose`
- Alt + drag release: print AprilTag placement snippet (`x`, `y`, `heading_hint`)
- `S`: stop robot
- `Q` or `Esc`: quit

## Zones

Zone masks are loaded from `Zones/` using this filename convention:

- `<id>_<name>.png`

Example:

- `1_Kitchen.png`

Any non-black/non-transparent pixel is treated as part of the zone.

## Notes

- The UI uses a display frame that is rotated with respect to API/world frame. The script handles conversion internally before sending commands.
- The script currently connects with `cw.affect("simulation")`.
- Battery refresh is forced periodically by publishing a `battery_check` command every 15 seconds.

## Troubleshooting

- If no map window appears, verify OpenCV is installed and importable.
- If robot pose does not update, verify twin/environment IDs and MQTT connectivity.
- If lidar points are missing, verify `TWIN_LIDAR_UUID` and lidar twin availability.
