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
import base64
import datetime
import re
from cyberwave import Cyberwave

# Import shared state/drawing functions from map_navigator
from Intelligence.Map import map_navigator as mn

# Import Apriltag globals
from Tutorials.CameraStream import ApriltagQuick as aq

# Import AprilTag localizer
from Project import apriltag_localizer as al


HOME_POSITION = [-0.14500000000000002, -0.25, 0.0]
# goto expects quaternion as [w, x, y, z]
HOME_ROTATION_WXYZ = [0.9796574969515479, 0.0, 0.0, 0.20067682643152376]

WORKFLOW_NAME = "VLA_CheckForHazards"
WORKFLOW_MODEL = "waveshare/ugv-beast"
HAZARD_PROMPT = (
    "Look for trailing cables or wires lying across the floor. "
    "Return ONLY strict JSON using this exact schema, no markdown or extra text."
)
HAZARD_SCHEMA_TEXT = (
    '{"trailing_cables_found":false,'
    '"region":"front|left|right|none",'
    '"evidence":"string"}'
)
QUESTION_TAG_PROMPT = (
    "This image has been pre-processed. Unknown AprilTag markers are highlighted with a yellow '?' label overlaid next to the black-and-white square tag. "
    "Look for any square AprilTag marker that has a yellow '?' label beside it. "
    "If found, identify the construction related(e.g. toolboxes) or construction material located near or attached to that marker. "
    "Return exactly one raw JSON object and no extra prose, code fences, or markdown using this schema: "
    '{"question_tag_found":false,"item":"string","material":"string","evidence":"string"}'
)


def _list_workflow_nodes(client, workflow_uuid: str):
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/nodes",
        path_params={"uuid": workflow_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "List[WorkflowNodeSchema]"},
    ).data


def _find_trigger_mission_node(nodes):
    for node in nodes:
        node_type = str(getattr(node, "node_type", "")).lower()
        trigger_type = str(getattr(node, "trigger_type", "")).lower()
        is_disabled = bool(getattr(node, "is_disabled", False))
        if node_type == "trigger" and trigger_type == "manual" and not is_disabled:
            return node
    return None


def _find_call_model_node(nodes):
    for node in nodes:
        node_type = str(getattr(node, "node_type", "")).lower()
        is_disabled = bool(getattr(node, "is_disabled", False))
        if is_disabled:
            continue
        if any(kw in node_type for kw in ("model", "action", "vla", "ai", "call")):
            return node
    return None


def _get_node(client, workflow_uuid: str, node_uuid: str):
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/nodes/{node_uuid}",
        path_params={"uuid": workflow_uuid, "node_uuid": node_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowNodeSchema"},
    ).data


def _patch_node_inputs(client, workflow_uuid: str, node_uuid: str, input_mappings: dict):
    existing = _get_node(client, workflow_uuid, node_uuid)
    existing_metadata = dict(getattr(existing, "metadata", None) or {})
    existing_input_mappings = dict(existing_metadata.get("input_mappings", {}))
    existing_input_mappings.update(input_mappings)
    existing_metadata["input_mappings"] = existing_input_mappings

    api = client.api.api_client
    body = {"metadata": existing_metadata}
    param = api.param_serialize(
        method="PUT",
        resource_path="/api/v1/workflows/{uuid}/nodes/{node_uuid}",
        path_params={"uuid": workflow_uuid, "node_uuid": node_uuid},
        body=body,
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowNodeSchema"},
    ).data


def _trigger_workflow(client, workflow_uuid: str, trigger_node_uuid: str):
    api = client.api.api_client
    body = {"inputs": {"node_uuid": trigger_node_uuid}}
    param = api.param_serialize(
        method="POST",
        resource_path="/api/v1/workflows/{uuid}/trigger",
        path_params={"uuid": workflow_uuid},
        body=body,
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowRunSchema"},
    ).data


def _get_execution(client, workflow_uuid: str, execution_uuid: str):
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/executions/{execution_uuid}",
        path_params={"uuid": workflow_uuid, "execution_uuid": execution_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowExecutionSchema"},
    ).data


def _wait_for_execution(client, workflow_uuid: str, run_uuid: str, timeout: float = 120.0):
    terminal = {"success", "error", "canceled", "failed", "completed"}
    deadline = time.monotonic() + timeout
    while True:
        execution = _get_execution(client, workflow_uuid, run_uuid)
        status = str(getattr(execution, "status", "")).lower()
        if status in terminal:
            return execution
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Execution {run_uuid} timed out")
        time.sleep(min(2.0, remaining))


def _extract_text_from_output_item(item):
    if isinstance(item, str):
        return item
    if isinstance(item, dict):
        preferred_keys = [
            "model_result",
            "result",
            "response",
            "output",
            "content",
            "answer",
            "text",
            "Result",
        ]
        for key in preferred_keys:
            value = item.get(key)
            if isinstance(value, str) and value.strip():
                return value
        for value in item.values():
            nested = _extract_text_from_output_item(value)
            if nested:
                return nested
    if isinstance(item, list):
        for value in item:
            nested = _extract_text_from_output_item(value)
            if nested:
                return nested
    return None


def _extract_json_from_text(text: str):
    if not text:
        return None
    try:
        return json.loads(text)
    except Exception:
        pass

    # Prefer fenced JSON blocks if present.
    fenced = re.search(r"```json\s*([\s\S]*?)\s*```", text, flags=re.IGNORECASE)
    if fenced:
        try:
            return json.loads(fenced.group(1))
        except Exception:
            pass

    # Fallback: extract first object-like block (non-greedy).
    match = re.search(r"\{[\s\S]*?\}", text)
    if match:
        try:
            return json.loads(match.group(0))
        except Exception:
            return None
    return None

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
        self._init_terminal_tags()

        # ---- RIGHT PANEL (Map Navigator) ----
        self.right_frame = ctk.CTkFrame(self, width=ideal_map_w)
        self.right_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        self.right_frame.grid_propagate(False) # Stop content from stretching the frame further!
        self.right_frame.grid_columnconfigure(0, weight=1)
        self.right_frame.grid_rowconfigure(0, weight=1)

        self.map_label = ctk.CTkLabel(self.right_frame, text="Loading Map...")
        self.map_label.grid(row=0, column=0, padx=10, pady=10) # Removed sticky="nsew" so it equals image size
        self._create_map_overlay_buttons()
        
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
        self.workflow_client = None
        self._pending_joint_ui = None
        self._joint_ui_lock = threading.Lock()
        
        # Initialize MQTT subscription using SDK's authenticated client
        self._setup_mqtt_subscription()

        # Key bindings
        self.bind("<Key>", self.on_keypress)

        # We rely on their background threads doing the updates, we just loop for UI drawing
        self.update_camera_direction()  # Initialize camera direction display
        self.update_views()

    def _init_terminal_tags(self):
        textbox = getattr(self.terminal, "_textbox", None)
        if textbox is None:
            return
        textbox.tag_config("query", foreground="#66d9ef")
        textbox.tag_config("cloud", foreground="#ffd866")
        textbox.tag_config("system", foreground="#a6e22e")
        textbox.tag_config("warn", foreground="#ff6188")

    def _terminal_log(self, channel: str, message: str):
        if threading.current_thread() is not threading.main_thread():
            self.after(0, lambda: self._terminal_log(channel, message))
            return
        tag_map = {
            "QUERY": "query",
            "CLOUD": "cloud",
            "SYSTEM": "system",
            "WARN": "warn",
        }
        prefix = f"[{channel}] "
        line = f"{prefix}{message}\n"
        self.terminal.configure(state="normal")
        textbox = getattr(self.terminal, "_textbox", None)
        if textbox is not None:
            textbox.insert("end", line, tag_map.get(channel, "system"))
        else:
            self.terminal.insert("end", line)
        self.terminal.see("end")
        self.terminal.configure(state="disabled")

    def _create_map_overlay_buttons(self):
        self.hazard_button = ctk.CTkButton(
            self.right_frame,
            text="VLA Hazard Sweep",
            command=self.on_vla_hazard_click,
            fg_color="#1f6aa5",
            hover_color="#2a83c4",
            width=220,
            height=38,
        )
        self.hazard_button.place(relx=0.98, rely=0.97, anchor="se")

        self.question_tag_button = ctk.CTkButton(
            self.right_frame,
            text="Find ? Tag Item",
            command=self.on_vla_question_tag_click,
            fg_color="#7a3ea1",
            hover_color="#9652bf",
            width=220,
            height=38,
        )
        self.question_tag_button.place(relx=0.98, rely=0.915, anchor="se")

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
                
                # Update map rendering with the camera's pan angle (inverted to match map coordinates)
                with mn._robot_lock:
                    mn._robot["camera_yaw"] = -self.actual_pan_rad

                # Queue UI update payload; apply from main thread in update_views.
                with self._joint_ui_lock:
                    self._pending_joint_ui = (pan_deg, tilt_deg)
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
            self.terminal.insert("end", f"> Command: Chassis Light {status}\n")
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
            self.terminal.insert("end", f"> Command: Camera Light {status}\n")
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

        # Apply pending joint-driven slider updates on the Tk main thread.
        pending = None
        with self._joint_ui_lock:
            if self._pending_joint_ui is not None:
                pending = self._pending_joint_ui
                self._pending_joint_ui = None
        if pending is not None:
            self._update_camera_sliders_from_hardware(*pending)
        
        # Frame skip for camera processing to improve UI performance
        if not hasattr(self, '_cam_process_count'):
            self._cam_process_count = 0
            self._last_cam_frame_pil = None
            self._last_cam_bytes = None
            
        try:
            # Update ApriltagQuick view using exact drawing logic adapted
            raw_bytes = aq.latest_frame["bytes"]
            if raw_bytes:
                # Only process if we have new bytes AND it's the right frame in the sequence
                # (Or if we haven't processed anything yet)
                if raw_bytes != self._last_cam_bytes and (self._cam_process_count % 3 == 0 or self._last_cam_frame_pil is None):
                    self._last_cam_bytes = raw_bytes
                    arr = np.frombuffer(raw_bytes, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        # Basic display for the dashboard
                        display_img = img.copy()
                        if aq.cv2 is not None and getattr(aq, 'aruco_dict', None) is not None:
                            gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
                            
                            # Dynamic Brightness: Fixes tags washing out when the physical LED is ON
                            h, w = gray.shape
                            crop_y, crop_x = h // 4, w // 4
                            center_roi = gray[crop_y : h - crop_y, crop_x : w - crop_x]
                            center_brightness = cv2.mean(center_roi)[0]
                            
                            # Optimize: skip LUT if normal lighting
                            if center_brightness < 90:
                                table = np.array([((i / 255.0) ** (1.0 / 1.5)) * 255 for i in np.arange(0, 256)]).astype("uint8")
                                processed_gray = cv2.LUT(gray, table)
                            elif center_brightness > 150:
                                table = np.array([((i / 255.0) ** (1.0 / 0.3)) * 255 for i in np.arange(0, 256)]).astype("uint8")
                                processed_gray = cv2.LUT(gray, table)
                            else:
                                processed_gray = gray
                            
                            # Apply light CLAHE for local contrast balancing - moved inside since LUT isn't needed for normal
                            if center_brightness < 90 or center_brightness > 150:
                                clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
                                processed_gray = clahe.apply(processed_gray)
                            
                            if getattr(aq, 'aruco_detector', None) is not None:
                                corners, ids, rejected = aq.aruco_detector.detectMarkers(processed_gray)
                            else:
                                corners, ids, rejected = cv2.aruco.detectMarkers(processed_gray, aq.aruco_dict, parameters=aq.aruco_params)
                                
                            if ids is not None and len(ids) > 0:
                                cv2.aruco.drawDetectedMarkers(display_img, corners)
                                
                                # Draw tag numbers in white
                                for i in range(len(ids)):
                                    tag_id = int(ids[i][0])
                                    known_tag = next((t for t in mn.APRILTAG_MARKERS if t.get("id") == tag_id), None)
                                    tag_label = str(tag_id) if known_tag else "?"
                                    corners_pts = corners[i][0].astype(int)
                                    center_x = int(np.mean(corners_pts[:, 0])+20)
                                    center_y = int(np.mean(corners_pts[:, 1]))
                                    if known_tag:
                                        cv2.putText(display_img, tag_label, (center_x - 10, center_y + 5),
                                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                                    else:
                                        # Unknown tags: draw '?' offset to the right with a filled background.
                                        side_x = int(np.max(corners_pts[:, 0])) + 14
                                        side_y = int(np.mean(corners_pts[:, 1]))
                                        font = cv2.FONT_HERSHEY_SIMPLEX
                                        scale = 0.9
                                        thickness = 2
                                        (tw, th), baseline = cv2.getTextSize(tag_label, font, scale, thickness)
                                        text_x = side_x
                                        text_y = side_y + th // 2

                                        pad = 5
                                        rect_x1 = text_x - pad
                                        rect_y1 = text_y - th - pad
                                        rect_x2 = text_x + tw + pad
                                        rect_y2 = text_y + baseline + pad

                                        # Clamp background box to image bounds.
                                        h_img, w_img = display_img.shape[:2]
                                        rect_x1 = max(0, min(rect_x1, w_img - 1))
                                        rect_y1 = max(0, min(rect_y1, h_img - 1))
                                        rect_x2 = max(0, min(rect_x2, w_img - 1))
                                        rect_y2 = max(0, min(rect_y2, h_img - 1))

                                        cv2.rectangle(display_img, (rect_x1, rect_y1), (rect_x2, rect_y2), (20, 20, 20), -1)
                                        cv2.rectangle(display_img, (rect_x1, rect_y1), (rect_x2, rect_y2), (255, 255, 255), 1)
                                        cv2.putText(display_img, tag_label, (text_x, text_y), font, scale, (0, 220, 255), thickness, cv2.LINE_AA)
                                
                                h, w = display_img.shape[:2]
                                fov_degrees = 118.0
                                fov_radians = math.radians(fov_degrees / 2.0)
                                focal_length = (w / 2.0) / math.tan(fov_radians)
                                center = (w / 2.0, h / 2.0)
                                camera_matrix = np.array([
                                    [focal_length, 0, center[0]],
                                    [0, focal_length, center[1]],
                                    [0, 0, 1]
                                ], dtype=np.float32)
                                
                                tag_size = 0.2
                                obj_pts = np.array([
                                    [-tag_size/2,  tag_size/2, 0],
                                    [ tag_size/2,  tag_size/2, 0],
                                    [ tag_size/2, -tag_size/2, 0],
                                    [-tag_size/2, -tag_size/2, 0]
                                ], dtype=np.float32)
                                
                                zero_dist = np.zeros((4, 1), dtype=np.float32)
                                
                                if not hasattr(self, 'tag_last_tagged'):
                                    self.tag_last_tagged = {}
                                    
                                for i in range(len(ids)):
                                    tag_id = int(ids[i][0])
                                    now = time.time()
                                    last_tagged = self.tag_last_tagged.get(tag_id, 0.0)
                                    
                                    # Only calculate pose and trigger initial pose if 10 seconds have passed
                                    if now - last_tagged > 10.0:
                                        self.tag_last_tagged[tag_id] = now
                                        
                                        success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i][0], camera_matrix, zero_dist)
                                        if success:
                                            axis_length = tag_size / 2
                                            axis_points_3d = np.array([
                                                [0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0]
                                            ], dtype=np.float32)
                                            axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, camera_matrix, zero_dist)
                                            axis_points_2d = axis_points_2d.astype(int)
                                            
                                            origin = tuple(axis_points_2d[0][0])
                                            x_end = tuple(axis_points_2d[1][0])
                                            y_end = tuple(axis_points_2d[2][0])
                                            cv2.line(display_img, origin, x_end, (0, 0, 255), 2)
                                            cv2.line(display_img, origin, y_end, (0, 255, 0), 2)
                                            
                                            dist = np.linalg.norm(tvec)
                                            cv2.putText(display_img, f"Dist: {dist:.2f}m", 
                                                        (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                                        
                                            known_tag = next((t for t in mn.APRILTAG_MARKERS if t.get("id") == tag_id), None)
                                            if known_tag:
                                                try:
                                                    payload, api_x, api_y, api_yaw = al.localise_from_tag(
                                                        rvec, tvec, known_tag,
                                                        pan_rad=self.actual_pan_rad,
                                                        debug=True,
                                                    )
                                                    topic = f"cyberwave/twin/{mn.TWIN_UUID}/navigate/initialpose"
                                                    client = mn.ugv.client
                                                    if client and hasattr(client, 'mqtt'):
                                                        # client.mqtt.publish(topic, json.dumps(payload))
                                                        log_msg1 = f"[AprilTag] Auto-localization via tag {tag_id} (Dist: {dist:.2f}m)."
                                                        log_msg2 = f"         → InitialPose sent: api_x={api_x:+.2f}, api_y={api_y:+.2f}, yaw={math.degrees(api_yaw):+.1f}°"
                                                        print(log_msg1)
                                                        print(log_msg2)
                                                        self.terminal.configure(state="normal")
                                                        self.terminal.insert("end", f"> {log_msg1}\n> {log_msg2}\n")
                                                        self.terminal.see("end")
                                                        self.terminal.configure(state="disabled")
                                                except Exception as e:
                                                    print(f"Error publishing initialpose from tag: {e}")

                        # Resize to fit frame constraint nicely
                        frame_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
                        self._last_cam_frame_pil = Image.fromarray(frame_rgb)
                        self._last_cam_shape = img.shape[:2]
                
                self._cam_process_count += 1
                
                # Apply the current or cached image to the UI (so UI still gets resized correctly if window size changes without recalculating CV maths)
                if self._last_cam_frame_pil is not None:
                    # Available space is defined by the left frame itself now, not the label
                    lbl_w = self.left_frame.winfo_width() - 20
                    max_h = self.left_frame.winfo_height() * 0.65 - 20 # Allow camera up to 65% of height
                    if lbl_w < 10 or max_h < 10:
                        lbl_w, max_h = 400, 300

                    # Flash stitched panorama for 2 seconds after hazard sweep.
                    flash_pil = getattr(self, '_flash_pil', None)
                    flash_until = getattr(self, '_flash_until', 0.0)
                    display_pil = flash_pil if (flash_pil is not None and time.monotonic() < flash_until) else self._last_cam_frame_pil
                    if display_pil is self._last_cam_frame_pil:
                        self._flash_pil = None  # Clear once expired

                    # maintain aspect ratio for camera
                    cam_h, cam_w = display_pil.size[1], display_pil.size[0]
                    cam_scale = min(lbl_w / cam_w, max_h / cam_h)
                    cam_new_w, cam_new_h = int(cam_w * cam_scale), int(cam_h * cam_scale)

                    ctk_img = ctk.CTkImage(light_image=display_pil, dark_image=display_pil, size=(max(1, cam_new_w), max(1, cam_new_h)))
                    self.apriltag_label.configure(image=ctk_img, text="")

            # Update Map Navigator view using its own drawing functions
            if not hasattr(self, '_map_process_count'):
                self._map_process_count = 0
                self._last_map_pil = None
                self._last_map_shape = None
                
            # Update map 10 times a second max (every 2nd tick at 50ms)
            if self._map_process_count % 2 == 0 or self._last_map_pil is None:
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

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self._last_map_pil = Image.fromarray(frame_rgb)
                self._last_map_shape = frame.shape[:2]
                
            self._map_process_count += 1

            if self._last_map_pil is not None:
                # Preserve ratio to prevent coordinate stretching
                # Determine actual display size dynamically based on UI right_frame
                lbl_w = self.right_frame.winfo_width() - 20
                lbl_h = self.right_frame.winfo_height() - 20
                if lbl_w < 10 or lbl_h < 10:
                    lbl_w, lbl_h = 800, 800

                mh, mw = self._last_map_shape
                scale = min(lbl_w / mw, lbl_h / mh)
                new_w, new_h = max(1, int(mw * scale)), max(1, int(mh * scale))
                
                self.map_disp_w = new_w
                self.map_disp_h = new_h

                ctk_img = ctk.CTkImage(light_image=self._last_map_pil, dark_image=self._last_map_pil, size=(new_w, new_h))
                self.map_label.configure(image=ctk_img, text="")
        except Exception:
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
        elif char == 'h':
            self.return_to_home()

    def return_to_home(self):
        """Send the robot to the fixed home pose using navigation.goto."""
        try:
            resp = mn.ugv.navigation.goto(
                HOME_POSITION,
                rotation=HOME_ROTATION_WXYZ,
                source_type="tele",
            )
            log_msg = (
                "[RTH] goto home sent: "
                f"pos=({HOME_POSITION[0]:+.3f}, {HOME_POSITION[1]:+.3f}, {HOME_POSITION[2]:+.3f}) "
                f"rot_wxyz=({HOME_ROTATION_WXYZ[0]:+.6f}, {HOME_ROTATION_WXYZ[1]:+.6f}, "
                f"{HOME_ROTATION_WXYZ[2]:+.6f}, {HOME_ROTATION_WXYZ[3]:+.6f})"
            )
            print(log_msg)
            print(f"[RTH] response: {resp}")
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"> {log_msg}\n")
            self.terminal.insert("end", f"> [RTH] response: {resp}\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")
        except Exception as e:
            err = f"[RTH] Failed to send home goto: {e}"
            print(err)
            self.terminal.configure(state="normal")
            self.terminal.insert("end", f"> {err}\n")
            self.terminal.see("end")
            self.terminal.configure(state="disabled")

    def on_vla_hazard_click(self):
        threading.Thread(target=self._run_vla_hazard_sweep, daemon=True).start()

    def on_vla_question_tag_click(self):
        threading.Thread(target=self._run_vla_question_tag_check, daemon=True).start()

    def _get_workflow_client(self):
        if self.workflow_client is None:
            self.workflow_client = Cyberwave()
            self.workflow_client.affect("live")
        return self.workflow_client

    def _capture_latest_frame(self, timeout_s: float = 4.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = aq.latest_frame.get("bytes")
            if raw and len(raw) > 200:
                return bytes(raw)
            time.sleep(0.1)
        return None

    def _capture_after_pan(self, pan_deg: int, settle_s: float, label: str):
        self.send_camera_command(pan_deg, 0)
        time.sleep(0.35)
        self.send_camera_command(pan_deg, 0)
        self._terminal_log("SYSTEM", f"Camera pan set to {pan_deg} deg for {label} view")
        time.sleep(settle_s)
        frame = self._capture_latest_frame(timeout_s=5.0)
        if frame is None:
            raise RuntimeError(f"Unable to capture {label} frame")
        return frame

    def _save_bytes_image(self, frame_bytes: bytes, stem: str):
        out_dir = os.path.join(_THIS_DIR, "..", "captures")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"{ts}_{stem}.jpg")
        with open(path, "wb") as fp:
            fp.write(frame_bytes)
        return path

    def _stitch_views(self, left_bytes: bytes, center_bytes: bytes, right_bytes: bytes):
        def _decode(b):
            arr = np.frombuffer(b, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)

        left = _decode(left_bytes)
        center = _decode(center_bytes)
        right = _decode(right_bytes)
        if left is None or center is None or right is None:
            raise RuntimeError("One or more camera images could not be decoded")

        target_h = min(left.shape[0], center.shape[0], right.shape[0])

        def _resize_to_h(img):
            scale = target_h / img.shape[0]
            return cv2.resize(img, (int(img.shape[1] * scale), target_h), interpolation=cv2.INTER_AREA)

        stitched = cv2.hconcat([_resize_to_h(left), _resize_to_h(center), _resize_to_h(right)])
        ok, encoded = cv2.imencode(".jpg", stitched, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            raise RuntimeError("Failed to encode stitched panorama")
        return encoded.tobytes()

    def _run_vla_hazard_sweep(self):
        try:
            self._terminal_log("SYSTEM", "Starting hazard sweep workflow")
            stop_resp = mn.ugv.navigation.stop(source_type="tele")
            self._terminal_log("SYSTEM", f"Robot stop sent: {stop_resp}")

            # Capture in left-center-right order with longer settle and duplicated commands.
            left = self._capture_after_pan(-55, settle_s=6.0, label="left")
            left_path = self._save_bytes_image(left, "left")
            self._terminal_log("SYSTEM", f"Left frame saved: {left_path}")

            center = self._capture_after_pan(0, settle_s=6.0, label="center")
            center_path = self._save_bytes_image(center, "center")
            self._terminal_log("SYSTEM", f"Center frame saved: {center_path}")

            right = self._capture_after_pan(55, settle_s=6.0, label="right")
            right_path = self._save_bytes_image(right, "right")
            self._terminal_log("SYSTEM", f"Right frame saved: {right_path}")

            self.send_camera_command(0, 0)
            time.sleep(0.35)
            self.send_camera_command(0, 0)
            pano = self._stitch_views(left, center, right)
            pano_path = self._save_bytes_image(pano, "stitched_panorama")
            self._terminal_log("SYSTEM", f"Panorama saved: {pano_path}")

            # Flash stitched panorama in the camera panel for 2 seconds.
            arr = np.frombuffer(pano, dtype=np.uint8)
            pano_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if pano_bgr is not None:
                pano_rgb = cv2.cvtColor(pano_bgr, cv2.COLOR_BGR2RGB)
                self._flash_pil = Image.fromarray(pano_rgb)
                self._flash_until = time.monotonic() + 2.0

            prompt = f"{HAZARD_PROMPT} {HAZARD_SCHEMA_TEXT}"
            self._terminal_log("QUERY", prompt)

            parsed, model_text = self._run_vla_query(prompt, pano)
            self._terminal_log("CLOUD", model_text or "No model output")

            if parsed is None:
                self._terminal_log("WARN", "Model response is not valid JSON; skipping auto-return-home")
                return

            pretty = json.dumps(parsed, indent=2)
            self._terminal_log("CLOUD", f"Parsed JSON:\n{pretty}")

            trailing_cables_found = bool(parsed.get("trailing_cables_found", False))
            region = str(parsed.get("region", "none")).strip()

            if trailing_cables_found:
                self._terminal_log("WARN", f"Trailing cables detected ({region}). Initiating return-to-home")
                self.after(0, self.return_to_home)
            else:
                self._terminal_log("SYSTEM", "No trailing cables detected")
        except Exception as exc:
            self._terminal_log("WARN", f"Hazard sweep failed: {exc}")

    def _run_vla_question_tag_check(self):
        try:
            # Use the processed frame (with ? overlay drawn) so the model can see the marker.
            frame = None
            pil_img = getattr(self, '_last_cam_frame_pil', None)
            if pil_img is not None:
                import io as _io
                buf = _io.BytesIO()
                pil_img.save(buf, format="JPEG", quality=90)
                frame = buf.getvalue()
            if frame is None:
                frame = self._capture_latest_frame(timeout_s=5.0)
            if frame is None:
                raise RuntimeError("Unable to capture frame for question-tag search")

            prompt = QUESTION_TAG_PROMPT
            self._terminal_log("QUERY", prompt)

            parsed, model_text = self._run_vla_query(prompt, frame)
            self._terminal_log("CLOUD", model_text or "No model output")

            if parsed is None:
                self._terminal_log("WARN", "Could not parse JSON response for question-tag check")
                return

            found = bool(parsed.get("question_tag_found", False))
            item = str(parsed.get("item", "")).strip()
            material = str(parsed.get("material", "")).strip()

            if found and (item or material):
                self._terminal_log("SYSTEM", f"MATERIAL : [{material or item or 'unknown'}] FOUND")
            else:
                self._terminal_log("SYSTEM", "No item found")
        except Exception as exc:
            self._terminal_log("WARN", f"Question-tag check failed: {exc}")

    def _run_vla_query(self, prompt: str, image_bytes: bytes):
        cw_live = self._get_workflow_client()
        workflows = cw_live.workflows.list()
        workflow = next((wf for wf in workflows if wf.is_active and wf.name == WORKFLOW_NAME), None)
        if workflow is None:
            raise RuntimeError(f"Active workflow '{WORKFLOW_NAME}' not found")

        nodes = _list_workflow_nodes(cw_live, workflow.uuid)
        trigger_node = _find_trigger_mission_node(nodes)
        model_node = _find_call_model_node(nodes)
        if trigger_node is None or model_node is None:
            raise RuntimeError("Missing trigger or model node in workflow")

        image_value = "data:image/jpeg;base64," + base64.b64encode(image_bytes).decode("ascii")
        model_inputs = {
            "prompt": {"mode": "value", "value": prompt},
            "text": {"mode": "value", "value": prompt},
            "image_bytes": {"mode": "value", "value": image_value},
            "image_url": {"mode": "value", "value": None},
        }
        _patch_node_inputs(cw_live, workflow.uuid, str(model_node.uuid), model_inputs)

        run = _trigger_workflow(cw_live, workflow.uuid, str(trigger_node.uuid))
        run_uuid = str(getattr(run, "execution_uuid", "") or getattr(run, "uuid", ""))
        if not run_uuid:
            raise RuntimeError("Workflow run did not return a UUID")

        execution = _wait_for_execution(cw_live, workflow.uuid, run_uuid, timeout=120.0)
        node_execs = getattr(execution, "node_executions", None) or []
        model_node_exec = next(
            (nx for nx in node_execs if str(getattr(nx, "node_uuid", "")) == str(model_node.uuid)),
            None,
        )

        if model_node_exec is None:
            raise RuntimeError("Model node execution not found")

        output_data = getattr(model_node_exec, "output_data", []) or []
        text_result = None
        for item in output_data:
            text_result = _extract_text_from_output_item(item)
            if text_result:
                break
        parsed = _extract_json_from_text(text_result) if text_result else None
        return parsed, (text_result or "")

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
        if self.workflow_client is not None:
            try:
                self.workflow_client.disconnect()
            except Exception:
                pass
        
        # Note: We don't disconnect the MQTT client as it's managed by the SDK
        # The SDK will handle cleanup
        
        time.sleep(0.5)
        super().destroy()

if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    app = DisplaySystem()
    app.mainloop()