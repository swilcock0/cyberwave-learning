import sys
import os

# Add paths to make imports work
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_THIS_DIR, ".."))
sys.path.insert(0, os.path.join(_THIS_DIR, "..", "Experiments"))
sys.path.insert(0, os.path.join(_THIS_DIR, "..", "Intelligence", "Map"))

import customtkinter as ctk
from PIL import Image
import cv2
import threading
import numpy as np
import time
import math
import json

# Import shared state/drawing functions from map_navigator
from Intelligence.Map import map_navigator as mn

# Import MQTT client for subscribing to hardware status
import paho.mqtt.client as mqtt

# Import Apriltag globals
from Tutorials.CameraStream import ApriltagQuick as aq

class DisplaySystem(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Display System")
        self.geometry("1400x900")
        self.after(0, lambda: self.state('zoomed')) # Fullscreen/zoomed on startup
        
        # Calculate ideal map width based on screen height to keep aspect ratio
        # so the right panel perfectly fits the map, giving the rest to the left panel
        screen_h = self.winfo_screenheight() - 100
        map_aspect = mn.DISP_W / mn.DISP_H
        ideal_map_w = int(screen_h * map_aspect)

        # Configure grid for main layout (1 row, 2 columns)
        self.grid_columnconfigure(0, weight=1) # Left side takes remaining space
        self.grid_columnconfigure(1, weight=0) # Right side fixed to map width
        self.grid_rowconfigure(0, weight=1)

        # ---- LEFT PANEL (Video + Terminal) ----
        self.left_frame = ctk.CTkFrame(self)
        self.left_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.left_frame.grid_propagate(False) # Stop content from stretching the frame further!
        self.left_frame.grid_columnconfigure(0, weight=1)
        self.left_frame.grid_rowconfigure(0, weight=0) # Camera top (fits to content)
        self.left_frame.grid_rowconfigure(1, weight=0) # Buttons frame
        self.left_frame.grid_rowconfigure(2, weight=1) # Terminal bottom (takes remaining)

        self.apriltag_label = ctk.CTkLabel(self.left_frame, text="Loading Camera...")
        self.apriltag_label.grid(row=0, column=0, padx=10, pady=10) # Removed sticky="nsew" so it wraps image tightly

        self.controls_frame = ctk.CTkFrame(self.left_frame, fg_color="transparent")
        self.controls_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        self.controls_frame.grid_columnconfigure(0, weight=1)  # Left: sliders
        self.controls_frame.grid_columnconfigure(1, weight=0)  # Right: button
        self.controls_frame.grid_rowconfigure(0, weight=0)
        self.controls_frame.grid_rowconfigure(1, weight=0)
        self.controls_frame.grid_rowconfigure(2, weight=0)

        # Light toggle buttons
        lights_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        lights_frame.grid(row=0, column=0, sticky="ew")

        self.chassis_light_on = False
        self.btn_chassis = ctk.CTkButton(
            lights_frame, 
            text="🔦 Chassis",
            fg_color="gray30",
            command=self.toggle_chassis_light
        )
        self.btn_chassis.pack(side="left", padx=5, expand=True, fill="x")

        self.camera_light_on = False
        self.btn_camera = ctk.CTkButton(
            lights_frame,
            text="💡 Camera",
            fg_color="gray30",
            command=self.toggle_camera_light
        )
        self.btn_camera.pack(side="right", padx=5, expand=True, fill="x")
        
        # Light status display (actual hardware state)
        status_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        status_frame.grid(row=0, column=1, sticky="ns", padx=5)
        
        self.chassis_status_label = ctk.CTkLabel(status_frame, text="Chassis: OFF", font=("Arial", 8), text_color="gray")
        self.chassis_status_label.pack(side="top", padx=2, pady=1)
        
        self.camera_status_label = ctk.CTkLabel(status_frame, text="Camera: OFF", font=("Arial", 8), text_color="gray")
        self.camera_status_label.pack(side="top", padx=2, pady=1)

        # Camera pan control
        pan_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        pan_frame.grid(row=1, column=0, sticky="ew", pady=3)
        pan_frame.grid_columnconfigure(1, weight=1)

        pan_label = ctk.CTkLabel(pan_frame, text="Pan", font=("Arial", 9), width=40)
        pan_label.grid(row=0, column=0, padx=5)

        self.pan_slider = ctk.CTkSlider(
            pan_frame,
            from_=-180,
            to=180,
            number_of_steps=360,
            command=self.on_pan_change,
            orientation="horizontal",
            height=20
        )
        self.pan_slider.set(0)
        self.pan_slider.grid(row=0, column=1, sticky="ew", padx=5)

        self.pan_value_label = ctk.CTkLabel(pan_frame, text="0°", font=("Arial", 9), width=40)
        self.pan_value_label.grid(row=0, column=2, padx=5)

        # Camera tilt control
        tilt_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        tilt_frame.grid(row=2, column=0, sticky="ew", pady=3)
        tilt_frame.grid_columnconfigure(1, weight=1)

        tilt_label = ctk.CTkLabel(tilt_frame, text="Tilt", font=("Arial", 9), width=40)
        tilt_label.grid(row=0, column=0, padx=5)

        self.tilt_slider = ctk.CTkSlider(
            tilt_frame,
            from_=-45,
            to=90,
            number_of_steps=135,
            command=self.on_tilt_change,
            orientation="horizontal",
            height=20
        )
        self.tilt_slider.set(0)
        self.tilt_slider.grid(row=0, column=1, sticky="ew", padx=5)

        self.tilt_value_label = ctk.CTkLabel(tilt_frame, text="0°", font=("Arial", 9), width=40)
        self.tilt_value_label.grid(row=0, column=2, padx=5)

        # Camera direction display (below sliders)
        direction_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        direction_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5)

        self.direction_label = ctk.CTkLabel(
            direction_frame,
            text="Direction: Forward",
            font=("Arial", 9),
            text_color="cyan"
        )
        self.direction_label.pack(side="left", padx=5)

        # Reset to center button (right side, spanning pan & tilt rows)
        reset_button_frame = ctk.CTkFrame(self.controls_frame, fg_color="transparent")
        reset_button_frame.grid(row=1, column=1, rowspan=2, sticky="ns", padx=5)

        reset_button = ctk.CTkButton(
            reset_button_frame,
            text="Reset to\nCenter",
            command=self.reset_camera_position,
            height=50,
            width=80
        )
        reset_button.pack(fill="both", expand=True)

        self.terminal = ctk.CTkTextbox(self.left_frame, fg_color="black", text_color="green", font=("Consolas", 14))
        self.terminal.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.terminal.insert("0.0", "System Initialized. Connected to UGV.\n")
        self.terminal.configure(state="disabled")

        # ---- RIGHT PANEL (Map Navigator) ----
        self.right_frame = ctk.CTkFrame(self, width=ideal_map_w)
        self.right_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        self.right_frame.grid_propagate(False) # Stop content from stretching the frame further!
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(0, weight=1)

        self.map_label = ctk.CTkLabel(self.right_frame, text="Loading Map...")
        self.map_label.grid(row=0, column=0, padx=10, pady=10) # Removed sticky="nsew" so it equals image size
        
        # Map Mouse Drag Binding
        self.map_label.bind("<ButtonPress-1>", self.on_map_press)
        self.map_label.bind("<B1-Motion>", self.on_map_drag)
        self.map_label.bind("<ButtonRelease-1>", self.on_map_release)

        # Background thread control
        self.stop_event = threading.Event()

        # Actual hardware state (from MQTT status messages)
        self.actual_chassis_light_value = 0
        self.actual_camera_light_value = 0
        self.actual_pan_rad = 0.0
        self.actual_tilt_rad = 0.0
        self.last_camera_cmd_time = 0.0
        
        # Initialize MQTT subscription using SDK's authenticated client
        self._setup_mqtt_subscription()

        # Key bindings
        self.bind("<Key>", self.on_keypress)

        # We rely on their background threads doing the updates, we just loop for UI drawing
        self.update_camera_direction()  # Initialize camera direction display
        self.update_views()

    def _setup_mqtt_subscription(self):
        """Setup MQTT subscriptions using the SDK's authenticated client"""
        try:
            # Use the SDK's already-authenticated MQTT client
            mqtt_client = mn.ugv.client.mqtt
            if not mqtt_client:
                print("[MQTT] SDK MQTT client not available")
                return
            
            # Set up our callback handlers
            mqtt_client.on_message = self._on_mqtt_message
            
            # Subscribe to status topics
            try:
                uuid = mn.ugv.uuid
                lights_topic = f"cyberwave/twin/{uuid}/lights/status"
                
                mqtt_client.subscribe(lights_topic)
                print(f"[MQTT] Subscribed to: {lights_topic}")
                print("[MQTT] Using SDK's authenticated MQTT client")
                
                # Use SDK native subscription for joint states (Pan and Tilt mechanisms)
                mn.ugv.subscribe_joints(self._on_joint_state)
                print("[SDK] Subscribed to joint states for camera orientation tracking")
            except Exception as e:
                print(f"[MQTT] Subscription error: {e}")
                import traceback
                traceback.print_exc()
        except Exception as e:
            print(f"[MQTT] Failed to setup subscription: {e}")
            import traceback
            traceback.print_exc()

    def _on_joint_state(self, payload, *args, **kwargs):
        """Process incoming joint state to capture camera pan and tilt"""
        try:
            # Robustly handle different SDK callback signatures
            if isinstance(payload, str) and len(args) > 0 and isinstance(args[0], dict):
                payload = args[0]
            elif isinstance(payload, (list, tuple)) and len(payload) == 2 and isinstance(payload[0], dict):
                payload = payload[0]

            if not isinstance(payload, dict):
                return
            
            joint_name = payload.get("joint_name")
            joint_state = payload.get("joint_state", {})
            position = joint_state.get("position")
            
            if position is None:
                return

            updated = False
            # Pan joint
            if joint_name == "pt_base_link_to_pt_link1":
                self.actual_pan_rad = float(position)
                updated = True
            # Tilt joint
            elif joint_name == "pt_link1_to_pt_link2":
                self.actual_tilt_rad = float(position)
                updated = True
                
            if updated:
                pan_deg = math.degrees(self.actual_pan_rad)
                tilt_deg = math.degrees(self.actual_tilt_rad)
                
                # Update map rendering with the camera's pan angle
                with mn._robot_lock:
                    mn._robot["camera_yaw"] = self.actual_pan_rad
                
                # Queue UI update on main thread - update sliders AND direction
                self.after(0, lambda pd=pan_deg, td=tilt_deg: self._update_camera_sliders_from_hardware(pd, td))
        except Exception as e:
            print(f"[Joints] Error processing joint state: {e}")

    def _on_mqtt_message(self, client, userdata, msg):
        """Process incoming MQTT status messages"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            
            # Handle lights status
            if "/lights/status" in topic:
                if "data" in payload:
                    data = payload["data"]
                    self.actual_chassis_light_value = int(data.get("io4", 0))
                    self.actual_camera_light_value = int(data.get("io5", 0))
                    print(f"[MQTT] Lights Status: chassis={self.actual_chassis_light_value}, camera={self.actual_camera_light_value}")
                    # Queue UI update on main thread
                    self.after(0, self._update_hardware_status_display)
        except Exception as e:
            print(f"[MQTT] Error processing message: {e}")

    def _update_camera_sliders_from_hardware(self, pan_deg, tilt_deg):
        """Update camera sliders to match actual hardware position"""
        try:
            # Debounce: don't overwrite user's drag if we recently sent a command
            if time.time() - getattr(self, 'last_camera_cmd_time', 0.0) >= 1.5:
                # Update slider values to show actual hardware position
                self.pan_slider.set(pan_deg)
                self.tilt_slider.set(tilt_deg)
                
                # Explicitly update textual labels because .set() does not trigger slider commands in CustomTkinter
                self.pan_value_label.configure(text=f"{int(pan_deg)}°")
                self.tilt_value_label.configure(text=f"{int(tilt_deg)}°")
            
            # Also update direction display (shows Cmd vs Actual while moving)
            self.update_camera_direction()
        except Exception as e:
            print(f"[MQTT] Error updating camera sliders: {e}")

    def toggle_chassis_light(self):
        self.chassis_light_on = not self.chassis_light_on
        val = 255 if self.chassis_light_on else 0
        status = "ON" if self.chassis_light_on else "OFF"
        
        try:
            topic = f"cyberwave/twin/{mn.ugv.uuid}/command"
            payload = {"command": "lights", "data": {"chassis_light": val}, "source_type": "tele"}
            
            # Get the Cyberwave client
            client = mn.ugv.client
            if not client:
                raise RuntimeError("mn.ugv.client is None")
            if not hasattr(client, 'mqtt'):
                raise RuntimeError("client doesn't have mqtt attribute")
            
            client.mqtt.connect()
            client.mqtt.publish(topic, payload)
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"> Command: Chassis Light {status} (awaiting hardware confirmation)\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")
        except Exception as e:
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"ERROR: Chassis Light toggle failed: {str(e)}\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")
            print(f"Error in toggle_chassis_light: {e}")
            import traceback
            traceback.print_exc()

    def toggle_camera_light(self):
        self.camera_light_on = not self.camera_light_on
        val = 255 if self.camera_light_on else 0
        status = "ON" if self.camera_light_on else "OFF"
        
        try:
            topic = f"cyberwave/twin/{mn.ugv.uuid}/command"
            payload = {"command": "lights", "data": {"camera_light": val}, "source_type": "tele"}
            
            # Get the Cyberwave client
            client = mn.ugv.client
            if not client:
                raise RuntimeError("mn.ugv.client is None")
            if not hasattr(client, 'mqtt'):
                raise RuntimeError("client doesn't have mqtt attribute")
            
            client.mqtt.connect()
            client.mqtt.publish(topic, payload)
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"> Command: Camera Light {status} (awaiting hardware confirmation)\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")
        except Exception as e:
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"ERROR: Camera Light toggle failed: {str(e)}\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")
            print(f"Error in toggle_camera_light: {e}")
            import traceback
            traceback.print_exc()

    def degrees_to_radians(self, degrees):
        """Convert degrees to radians"""
        return degrees * math.pi / 180.0

    def on_pan_change(self, value):
        """Handle pan slider change"""
        pan_deg = int(float(value))
        self.pan_value_label.configure(text=f"{pan_deg}°")
        self.update_camera_direction()
        self.send_camera_command(pan_deg, int(self.tilt_slider.get()))

    def on_tilt_change(self, value):
        """Handle tilt slider change"""
        tilt_deg = int(float(value))
        self.tilt_value_label.configure(text=f"{tilt_deg}°")
        self.update_camera_direction()
        self.send_camera_command(int(self.pan_slider.get()), tilt_deg)

    def send_camera_command(self, pan_deg, tilt_deg):
        """Send camera servo command via MQTT"""
        self.last_camera_cmd_time = time.time()
        try:
            pan_rad = self.degrees_to_radians(pan_deg)
            tilt_rad = self.degrees_to_radians(tilt_deg)
            
            topic = f"cyberwave/twin/{mn.ugv.uuid}/command"
            payload = {
                "command": "camera_servo",
                "data": {"pan": pan_rad, "tilt": tilt_rad},
                "source_type": "tele"
            }
            
            client = mn.ugv.client
            if not client or not hasattr(client, 'mqtt'):
                return
            
            client.mqtt.connect()
            client.mqtt.publish(topic, payload)
        except Exception as e:
            print(f"Error sending camera command: {e}")

    def reset_camera_position(self):
        """Reset camera to center position (pan 0°, tilt 0°)"""
        self.pan_slider.set(0)
        self.tilt_slider.set(0)
        self.send_camera_command(0, 0)
        self.update_camera_direction()

    def calculate_camera_direction(self, pan_deg, tilt_deg):
        """
        Calculate camera direction vector relative to robot base.
        
        Pan: horizontal rotation (0° = forward, 90° = left, -90° = right)
        Tilt: vertical rotation (0° = horizontal, 90° = up, -45° = down)
        
        Returns: (x, y, z) unit direction vector and descriptive string
        """
        # Convert to radians
        pan_rad = self.degrees_to_radians(pan_deg)
        tilt_rad = self.degrees_to_radians(tilt_deg)
        
        # Start with forward direction along robot's local X-axis
        # Apply pan rotation around Z-axis (vertical)
        # Apply tilt rotation around Y-axis (sideways, perpendicular to forward)
        
        x = math.cos(tilt_rad) * math.sin(pan_rad)
        y = math.cos(tilt_rad) * math.cos(pan_rad)  # Forward is +Y when pan=0
        z = math.sin(tilt_rad)
        
        # Determine direction description
        direction_parts = []
        
        # Vertical component
        if z > 0.3:
            direction_parts.append("Up")
        elif z < -0.3:
            direction_parts.append("Down")
        
        # Horizontal component (relative to robot forward)
        if abs(pan_deg) < 30:
            direction_parts.append("Forward")
        elif pan_deg > 30:
            direction_parts.append("Left")
        elif pan_deg < -30:
            direction_parts.append("Right")
        
        if not direction_parts:
            direction_parts = ["Horizontal"]
        
        direction_str = ", ".join(direction_parts)
        
        return (x, y, z), direction_str

    def update_camera_direction(self):
        """Update camera direction display"""
        try:
            pan_deg = int(float(self.pan_slider.get()))
            tilt_deg = int(float(self.tilt_slider.get()))
            
            (x, y, z), direction_str = self.calculate_camera_direction(pan_deg, tilt_deg)
            
            # Also show actual hardware position if different
            actual_pan_deg = math.degrees(self.actual_pan_rad)
            actual_tilt_deg = math.degrees(self.actual_tilt_rad)
            
            if abs(actual_pan_deg - pan_deg) > 2 or abs(actual_tilt_deg - tilt_deg) > 2:
                # Significant difference - show both commanded and actual
                actual_dir = self.calculate_camera_direction(actual_pan_deg, actual_tilt_deg)[1]
                self.direction_label.configure(
                    text=f"Cmd: {direction_str} | Actual: {actual_dir}",
                    text_color="yellow"
                )
            else:
                # Positions match
                self.direction_label.configure(
                    text=f"Direction: {direction_str}",
                    text_color="cyan"
                )
        except Exception as e:
            print(f"Error updating camera direction: {e}")

    def _update_hardware_status_display(self):
        """Update status labels to show actual hardware state"""
        try:
            # Update lights status
            chassis_status = "ON" if self.actual_chassis_light_value > 0 else "OFF"
            camera_status = "ON" if self.actual_camera_light_value > 0 else "OFF"
            
            self.chassis_status_label.configure(text=f"Chassis: {chassis_status}")
            self.camera_status_label.configure(text=f"Camera: {camera_status}")
            
            # Update button appearance based on ACTUAL hardware state
            chassis_color = "#22AA22" if self.actual_chassis_light_value > 0 else "gray30"
            camera_color = "#22AA22" if self.actual_camera_light_value > 0 else "gray30"
            chassis_label_color = "lime" if self.actual_chassis_light_value > 0 else "gray"
            camera_label_color = "lime" if self.actual_camera_light_value > 0 else "gray"
            
            # Only update if there's a change to avoid flicker
            if self.btn_chassis.cget("fg_color")[0] != chassis_color:
                self.btn_chassis.configure(fg_color=chassis_color, text_color="white")
            if self.chassis_status_label.cget("text_color") != chassis_label_color:
                self.chassis_status_label.configure(text_color=chassis_label_color)
                
            if self.btn_camera.cget("fg_color")[0] != camera_color:
                self.btn_camera.configure(fg_color=camera_color, text_color="white")
            if self.camera_status_label.cget("text_color") != camera_label_color:
                self.camera_status_label.configure(text_color=camera_label_color)
                
        except Exception as e:
            print(f"Error updating hardware status display: {e}")

    def update_views(self):
        """Update all display elements"""
        # Update hardware status displays
        self._update_hardware_status_display()
        
        try:
            # Update ApriltagQuick view using exact drawing logic adapted
            raw_bytes = aq.latest_frame["bytes"]
            if raw_bytes:
                arr = np.frombuffer(raw_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    # Basic display for the dashboard
                    display_img = img.copy()
                    if aq.cv2 is not None and getattr(aq, 'aruco_dict', None) is not None:
                        gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
                        
                        # Dynamic Brightness: Fixes tags washing out when the physical LED is ON
                        # Evaluate lighting based on the center of the image where the LED casts its beam
                        h, w = gray.shape
                        crop_y, crop_x = h // 4, w // 4
                        center_roi = gray[crop_y : h - crop_y, crop_x : w - crop_x]
                        center_brightness = cv2.mean(center_roi)[0]
                        
                        if center_brightness < 90:
                            # Dark (LED OFF): brighten midtones using Gamma Correction (gamma = 1.5)
                            table = np.array([((i / 255.0) ** (1.0 / 1.5)) * 255 for i in np.arange(0, 256)]).astype("uint8")
                            gray_adj = cv2.LUT(gray, table)
                        elif center_brightness > 150:
                            # Washed out (LED ON): heavily darken midtones to pull detail out of the glare (gamma = 0.3)
                            table = np.array([((i / 255.0) ** (1.0 / 0.3)) * 255 for i in np.arange(0, 256)]).astype("uint8")
                            gray_adj = cv2.LUT(gray, table)
                        else:
                            # Normal lighting
                            gray_adj = gray
                        
                        # Apply light CLAHE for local contrast balancing
                        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
                        processed_gray = clahe.apply(gray_adj)
                        
                        if getattr(aq, 'aruco_detector', None) is not None:
                            corners, ids, rejected = aq.aruco_detector.detectMarkers(processed_gray)
                        else:
                            corners, ids, rejected = cv2.aruco.detectMarkers(processed_gray, aq.aruco_dict, parameters=aq.aruco_params)
                            
                        if ids is not None and len(ids) > 0:
                            cv2.aruco.drawDetectedMarkers(display_img, corners)

                    # Resize to fit frame constraint nicely
                    frame_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
                    img_pil = Image.fromarray(frame_rgb)
                    
                    # Available space is defined by the left frame itself now, not the label
                    lbl_w = self.left_frame.winfo_width() - 20
                    max_h = self.left_frame.winfo_height() * 0.65 - 20 # Allow camera up to 65% of height
                    if lbl_w < 10 or max_h < 10:
                        lbl_w, max_h = 400, 300
                    
                    # maintain aspect ratio for camera
                    cam_h, cam_w = img.shape[:2]
                    cam_scale = min(lbl_w / cam_w, max_h / cam_h)
                    cam_new_w, cam_new_h = int(cam_w * cam_scale), int(cam_h * cam_scale)
                    
                    ctk_img = ctk.CTkImage(light_image=img_pil, dark_image=img_pil, size=(max(1, cam_new_w), max(1, cam_new_h)))
                    self.apriltag_label.configure(image=ctk_img, text="")

            # Update Map Navigator view using its own drawing functions
            frame = cv2.resize(mn._BASE_IMG, (mn.DISP_W, mn.DISP_H), interpolation=cv2.INTER_NEAREST)
            mn._draw_grid(frame)
            
            with mn._robot_lock:
                rx, ry = mn._robot["x"], mn._robot["y"]
            mn._current_zone = mn._check_zone(rx, ry)
            if mn._current_zone is not None:
                mn._draw_zone_mask(frame, mn._current_zone)
            
            mn._draw_last_goal(frame)
            mn._draw_drag_arrow(frame)
            mn._draw_lidar_scan(frame)
            mn._draw_apriltags(frame)
            mn._draw_robot(frame)
            mn._draw_logo(frame)
            mn._draw_hud(frame)

            # Preserve ratio to prevent coordinate stretching
            # Determine actual display size dynamically based on UI right_frame
            lbl_w = self.right_frame.winfo_width() - 20
            lbl_h = self.right_frame.winfo_height() - 20
            if lbl_w < 10 or lbl_h < 10:
                lbl_w, lbl_h = 800, 800

            mh, mw = frame.shape[:2]
            scale = min(lbl_w / mw, lbl_h / mh)
            new_w, new_h = max(1, int(mw * scale)), max(1, int(mh * scale))
            
            self.map_disp_w = new_w
            self.map_disp_h = new_h

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(frame_rgb)
            ctk_img = ctk.CTkImage(light_image=img_pil, dark_image=img_pil, size=(new_w, new_h))
            self.map_label.configure(image=ctk_img, text="")
        except Exception as e:
            import traceback
            traceback.print_exc()

        if not self.stop_event.is_set():
            self.after(50, self.update_views)
            
    # Passing drag events to Map Navigator logic mapped to original aspect ratio
    def _get_map_coords(self, x, y):
        # We scale the CTkImage's UI coordinates back to mn.DISP_W/DISP_H space
        if not hasattr(self, 'map_disp_w'):
            return x, y
        actual_x = int(x * (mn.DISP_W / self.map_disp_w))
        actual_y = int(y * (mn.DISP_H / self.map_disp_h))
        return actual_x, actual_y

    def _get_cv2_flags(self, event):
        flags = 0
        if hasattr(event, 'state') and event.state:
            if event.state & 0x0001:  # Shift
                flags |= cv2.EVENT_FLAG_SHIFTKEY
            if event.state & 0x0004:  # Control
                flags |= cv2.EVENT_FLAG_CTRLKEY
            if event.state & 0x20000 or event.state & 0x0008:  # Alt (Windows is 0x20000, others 0x0008)
                flags |= cv2.EVENT_FLAG_ALTKEY
        return flags

    def on_keypress(self, event):
        char = getattr(event, 'char', '').lower()
        keysym = getattr(event, 'keysym', '')
        if char == 'q' or keysym == 'Escape':
            self.destroy()
        elif char == 's':
            resp = mn.ugv.navigation.stop(source_type="tele")
            print(f"→ stop  |  resp: {resp}")

    def on_map_press(self, event):
        mx, my = self._get_map_coords(event.x, event.y)
        mn._on_mouse(cv2.EVENT_LBUTTONDOWN, mx, my, self._get_cv2_flags(event), None)
        
    def on_map_drag(self, event):
        mx, my = self._get_map_coords(event.x, event.y)
        mn._on_mouse(cv2.EVENT_MOUSEMOVE, mx, my, self._get_cv2_flags(event), None)
        
    def on_map_release(self, event):
        mx, my = self._get_map_coords(event.x, event.y)
        # Force the shift flag as active if the user holds shift while clicking
        mn._on_mouse(cv2.EVENT_LBUTTONUP, mx, my, self._get_cv2_flags(event), None)

    def destroy(self):
        self.stop_event.set()
        aq.stop_event.set()
        
        # Note: We don't disconnect the MQTT client as it's managed by the SDK
        # The SDK will handle cleanup
        
        time.sleep(0.5)
        super().destroy()

if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    app = DisplaySystem()
    app.mainloop()