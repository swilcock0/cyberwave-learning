"""
Command Handler Framework for MQTT Bridge Command Router

This module provides a scalable command routing system that maps MQTT commands
to ROS topics without using if-else chains. New commands can be added by:
1. Creating a handler class (for complex logic)
2. Using @register_command decorator
3. Or adding to YAML config (for simple mappings)

Design Pattern: Command Pattern + Registry Pattern
"""

import json
import base64
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Callable
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

# Import common ROS message types
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Import OpenCV for image encoding
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class CommandHandler(ABC):
    """
    Abstract base class for command handlers.
    
    Each command type should implement this interface.
    Handlers are responsible for:
    1. Validating command data
    2. Converting MQTT data to ROS messages
    3. Publishing to appropriate ROS topics
    """
    
    def __init__(self, node: Node):
        """
        Initialize handler with ROS node.
        
        Args:
            node: ROS2 node for creating publishers and logging
        """
        self.node = node
        self.logger = node.get_logger()
        self._publishers: Dict[str, Publisher] = {}
        self._mqtt_adapter: Any = None
        self._command_topic: Optional[str] = None
        self._setup_publishers()
    
    @abstractmethod
    def get_command_name(self) -> str:
        """Get the unique command name this handler handles."""
        pass

    @abstractmethod
    def handle(self, data: Dict[str, Any]) -> bool:
        """Process the command data and publish to ROS."""
        pass

    def _setup_publishers(self) -> None:
        """Create ROS publishers needed for this command."""
        pass
    
    def set_mqtt_context(self, adapter: Any, command_topic: str):
        """Set the MQTT adapter and topic for sending responses."""
        self._mqtt_adapter = adapter
        self._command_topic = command_topic

    def _get_status_topic(self) -> Optional[str]:
        """
        Get the status topic for this command's responses.
        
        Pattern: {prefix}cyberwave/twin/{twin_uuid}/{command}/status
        
        Returns:
            Status topic string or None if twin_uuid not available
        """
        try:
            twin_uuid = None
            if hasattr(self.node, '_mapping') and self.node._mapping:
                twin_uuid = getattr(self.node._mapping, 'twin_uuid', None)
            
            if not twin_uuid:
                return None
            
            prefix = getattr(self.node, 'ros_prefix', '')
            command_name = self.get_command_name()
            return f"{prefix}cyberwave/twin/{twin_uuid}/{command_name}/status"
        except Exception:
            return None

    def publish_response(self, response_data: Dict[str, Any]) -> bool:
        """
        Publish a response to the dedicated status topic for this command.
        
        Topic pattern: {prefix}cyberwave/twin/{twin_uuid}/{command}/status
        
        Args:
            response_data: Data to send back to the frontend
            
        Returns:
            True if published, False otherwise
        """
        if not self._mqtt_adapter:
            self.logger.warning("Cannot publish response: MQTT adapter not set")
            return False
        
        status_topic = self._get_status_topic()
        if not status_topic:
            self.logger.warning("Cannot publish response: twin_uuid not available")
            return False
            
        try:
            # Wrap response with metadata
            payload = {
                "command": self.get_command_name(),
                "type": "response",
                "source_type": "edge",
                "timestamp": self.node.get_clock().now().nanoseconds / 1e9,
                "data": response_data
            }
            # Serialize payload to JSON string before publishing
            json_payload = json.dumps(payload)
            self._mqtt_adapter.publish(status_topic, json_payload)
            self.logger.debug(f"Published response to {status_topic}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish response: {e}")
            return False
    
    def publish_simple_response(self, response_data: Dict[str, Any]) -> bool:
        """
        Publish a simple response (without command/type wrapper) for video commands.
        This matches the format expected by the frontend for start_video/stop_video.
        
        Args:
            response_data: Data to send back (e.g., {"status": "ok", "type": "video_started"})
            
        Returns:
            True if published, False otherwise
        """
        if not self._mqtt_adapter:
            self.logger.warning("Cannot publish simple response: MQTT adapter not set")
            return False
        
        status_topic = self._get_status_topic()
        if not status_topic:
            self.logger.warning("Cannot publish simple response: twin_uuid not available")
            return False
            
        try:
            # Add source_type and timestamp to simple response
            response_data["source_type"] = "edge"
            response_data["timestamp"] = self.node.get_clock().now().nanoseconds / 1e9
            json_payload = json.dumps(response_data)
            self._mqtt_adapter.publish(status_topic, json_payload)
            self.logger.debug(f"Published simple response to {status_topic}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish simple response: {e}")
            return False

    @abstractmethod
    def _setup_publishers(self) -> None:
        """Create ROS publishers needed for this command."""
        pass
    
    @abstractmethod
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle the command with given data.
        
        Args:
            data: Command data dictionary from MQTT message
            
        Returns:
            True if command was handled successfully, False otherwise
        """
        pass
    
    @abstractmethod
    def get_command_name(self) -> str:
        """Return the command name this handler processes."""
        pass
    
    def validate_data(self, data: Dict[str, Any], required_fields: list) -> bool:
        """
        Validate that required fields are present in data.
        
        Args:
            data: Data dictionary to validate
            required_fields: List of required field names
            
        Returns:
            True if all fields present, False otherwise
        """
        missing = [f for f in required_fields if f not in data]
        if missing:
            self.logger.warning(
                f"Command '{self.get_command_name()}' missing fields: {missing}"
            )
            return False
        return True


class GenericActuationHandler(CommandHandler):
    """
    GENERIC handler for actuation-based commands from frontend.
    
    This handler maps simple actuation strings (e.g., "move_forward", "turn_left")
    to appropriate ROS messages, enabling universal command-based control for
    ANY robot using "control_mode": "command".
    
    Supported actuations:
    - Locomotion: move_forward, move_backward, turn_left, turn_right, stop
    - Camera: camera_up, camera_down, camera_left, camera_right
    - Lights: chassis_light_toggle, camera_light_toggle, led_toggle
    - Utilities: take_photo, battery_check
    - Robot-specific: sit_down, stand_up, obstacle_avoidance_toggle
    
    Configuration:
    - Linear speed: Default 0.5 m/s (can be configured via ROS params)
    - Angular speed: Default 0.8 rad/s (can be configured via ROS params)
    """
    
    def __init__(self, node: Node):
        # Velocity configurations based on Waveshare UGV Beast specifications
        # 
        # Research findings:
        # - UGV Beast maximum speed: 0.35 m/s (official spec)
        # - Safe teleoperation speed: 0.3 m/s (85% of max for smooth control)
        # - Angular velocity: ~1.0 rad/s (based on differential drive kinematics)
        # 
        # Sources:
        # - https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2
        # - Differential drive formula: ω_max = (2 × v_wheel) / wheel_separation
        # - With wheel speed ≈0.35 m/s and typical separation ~0.4m: ω ≈ 1.0-1.25 rad/s
        self._linear_speed = 0.3   # m/s (safe teleoperation speed)
        self._angular_speed = 1.0  # rad/s (safe turning speed)
        self._camera_step = 0.1    # radians per key press
        
        # Track light states for toggles
        self._chassis_light_on = False
        self._camera_light_on = False
        self._led_on = False
        
        # Camera servo current position
        self._camera_pan = 0.0
        self._camera_tilt = 0.0
        
        super().__init__(node)
        self.logger.info(
            f"GenericActuationHandler initialized: "
            f"linear_speed={self._linear_speed} m/s, "
            f"angular_speed={self._angular_speed} rad/s"
        )
    
    def get_command_name(self) -> str:
        return "actuation"
    
    def _setup_publishers(self) -> None:
        """Create publishers for all possible actuation types."""
        # Movement
        self._publishers['cmd_vel'] = self.node.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Lights (UGV Beast specific, but safe to create)
        self._publishers['led_ctrl'] = self.node.create_publisher(
            Float32MultiArray, '/ugv/led_ctrl', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle generic actuation command.
        
        Expected data formats (from frontend):
        
        1. Simple commands (keyboard controller):
        {
            "command": "move_forward",
            "timestamp": 1706547890.123,
            "source_type": "tele"
        }
        
        2. Complex commands (video streaming):
        {
            "command": "start_video",
            "data": {"camera": "default", "recording": true},
            "timestamp": 1706547890.123,
            "source_type": "tele"
        }
        """
        try:
            self.logger.info(f"[DEBUG] GenericActuationHandler.handle() called with data: {data}")
            
            # Extract the actuation command
            actuation = data.get('command')
            if not actuation:
                self.logger.warning("Actuation handler requires 'command' field")
                return False
            
            self.logger.info(f"Processing actuation: {actuation}")
            
            # Extract additional data if present (for video commands, etc.)
            command_data = data.get('data', {})
            
            # Map actuation to ROS command
            result = self._process_actuation(actuation, command_data)
            self.logger.info(f"[DEBUG] _process_actuation returned: {result}")
            return result
            
        except Exception as e:
            self.logger.error(f"Failed to handle actuation: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def _process_actuation(self, actuation: str, data: Dict[str, Any] = None) -> bool:
        """
        Map actuation string to appropriate ROS message.
        
        This is the GENERIC mapping function that converts simple actuation
        commands to ROS messages.
        """
        # ============= LOCOMOTION ACTUATIONS =============
        if actuation == "move_forward":
            return self._send_twist(self._linear_speed, 0.0)
        
        elif actuation == "move_backward":
            return self._send_twist(-self._linear_speed, 0.0)
        
        elif actuation == "turn_left":
            return self._send_twist(0.0, self._angular_speed)
        
        elif actuation == "turn_right":
            return self._send_twist(0.0, -self._angular_speed)
        
        elif actuation == "stop":
            return self._send_twist(0.0, 0.0)
        
        # ============= CAMERA SERVO ACTUATIONS =============
        elif actuation == "camera_up":
            return self._send_camera_servo(tilt_delta=self._camera_step)
        
        elif actuation == "camera_down":
            return self._send_camera_servo(tilt_delta=-self._camera_step)
        
        elif actuation == "camera_left":
            return self._send_camera_servo(pan_delta=-self._camera_step)
        
        elif actuation == "camera_right":
            return self._send_camera_servo(pan_delta=self._camera_step)
        
        elif actuation == "camera_default":
            # Reset camera to default position (0.0, 0.0)
            return self._send_camera_servo_reset()
        
        # ============= LIGHT ACTUATIONS =============
        elif actuation == "chassis_light_toggle":
            self._chassis_light_on = not self._chassis_light_on
            return self._send_led_ctrl(chassis=255 if self._chassis_light_on else 0)
        
        elif actuation == "camera_light_toggle":
            self._camera_light_on = not self._camera_light_on
            return self._send_led_ctrl(camera=255 if self._camera_light_on else 0)
        
        elif actuation == "led_toggle":
            self._led_on = not self._led_on
            value = 255 if self._led_on else 0
            return self._send_led_ctrl(chassis=value, camera=value)
        
        # ============= UTILITY ACTUATIONS =============
        elif actuation == "take_photo":
            # Delegate to TakePhotoHandler via command registry
            if hasattr(self.node, '_command_registry'):
                return self.node._command_registry.handle_command("take_photo", {})
            self.logger.warning("take_photo: command registry not available")
            return False
        
        elif actuation == "battery_check":
            # Delegate to BatteryCheckHandler via command registry
            if hasattr(self.node, '_command_registry'):
                return self.node._command_registry.handle_command("battery_check", {})
            self.logger.warning("battery_check: command registry not available")
            return False
        
        # ============= VIDEO STREAM ACTUATIONS =============
        elif actuation == "start_video":
            # Start camera WebRTC streaming using SDK Twin API
            try:
                if hasattr(self.node, 'start_camera_stream'):
                    # Extract recording preference from data if provided
                    recording = True  # Default
                    if data and isinstance(data, dict):
                        recording = data.get('recording', True)
                    
                    self.logger.info(f"Starting video stream (recording={recording})")
                    self.node.start_camera_stream(recording=recording)
                    
                    # Send upstream response in simple format (FE expects this)
                    self.publish_simple_response({"status": "ok", "type": "video_started"})
                    return True
                else:
                    self.logger.error("start_video: node.start_camera_stream() not available")
                    self.publish_simple_response({"status": "error", "message": "Video streaming not supported"})
                    return False
            except Exception as e:
                self.logger.error(f"Failed to start video stream: {e}")
                self.publish_simple_response({"status": "error", "message": str(e)})
                return False
        
        elif actuation == "stop_video":
            # Stop camera WebRTC streaming using SDK Twin API
            try:
                if hasattr(self.node, 'stop_camera_stream'):
                    self.logger.info("Stopping video stream")
                    self.node.stop_camera_stream()
                    
                    # Send upstream response in simple format (FE expects this)
                    self.publish_simple_response({"status": "ok", "type": "video_stopped"})
                    return True
                else:
                    self.logger.error("stop_video: node.stop_camera_stream() not available")
                    self.publish_simple_response({"status": "error", "message": "Video streaming not supported"})
                    return False
            except Exception as e:
                self.logger.error(f"Failed to stop video stream: {e}")
                self.publish_simple_response({"status": "error", "message": str(e)})
                return False
        
        # ============= ROBOT-SPECIFIC ACTUATIONS =============
        # These are placeholders - implement based on specific robot capabilities
        elif actuation == "sit_down":
            self.logger.info("sit_down actuation (no ROS topic mapped yet)")
            self.publish_response({"status": "not_implemented", "actuation": actuation})
            return True
        
        elif actuation == "stand_up":
            self.logger.info("stand_up actuation (no ROS topic mapped yet)")
            self.publish_response({"status": "not_implemented", "actuation": actuation})
            return True
        
        elif actuation == "obstacle_avoidance_toggle":
            self.logger.info("obstacle_avoidance_toggle actuation (no ROS topic mapped yet)")
            self.publish_response({"status": "not_implemented", "actuation": actuation})
            return True
        
        # ============= UNKNOWN ACTUATION =============
        else:
            self.logger.warning(
                f"Unknown actuation: {actuation}. "
                f"Add mapping in GenericActuationHandler._process_actuation()"
            )
            self.publish_response({
                "status": "error",
                "message": f"Unknown actuation: {actuation}"
            })
            return False
    
    def _send_twist(self, linear_x: float, angular_z: float) -> bool:
        """Send a Twist message for locomotion."""
        try:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(angular_z)
            
            self._publishers['cmd_vel'].publish(msg)
            self.logger.debug(f"Published Twist: linear.x={linear_x}, angular.z={angular_z}")
            
            # Send confirmation
            self.publish_response({
                "status": "success",
                "linear_x": linear_x,
                "angular_z": angular_z
            })
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send Twist: {e}")
            return False
    
    def _send_camera_servo(self, pan_delta: float = 0.0, tilt_delta: float = 0.0) -> bool:
        """Send camera servo command via camera_servo handler."""
        try:
            # Delegate to CameraServoHandler via command registry
            if hasattr(self.node, '_command_registry'):
                servo_data = {}
                if pan_delta != 0.0:
                    servo_data['pan_delta'] = pan_delta
                if tilt_delta != 0.0:
                    servo_data['tilt_delta'] = tilt_delta
                
                return self.node._command_registry.handle_command("camera_servo", servo_data)
            
            self.logger.warning("camera_servo: command registry not available")
            return False
            
        except Exception as e:
            self.logger.error(f"Failed to send camera servo: {e}")
            return False
    
    def _send_camera_servo_reset(self) -> bool:
        """Reset camera servo to default position (0.0, 0.0)."""
        try:
            self.logger.info("[DEBUG] _send_camera_servo_reset() called")
            
            # Delegate to CameraServoHandler via command registry with absolute positions
            if hasattr(self.node, '_command_registry'):
                servo_data = {
                    'pan': 0.0,
                    'tilt': 0.0
                }
                self.logger.info(f"[DEBUG] Calling command registry with camera_servo command: {servo_data}")
                result = self.node._command_registry.handle_command("camera_servo", servo_data)
                self.logger.info(f"[DEBUG] camera_servo handler returned: {result}")
                return result
            
            self.logger.warning("camera_servo: command registry not available")
            return False
            
        except Exception as e:
            self.logger.error(f"Failed to reset camera servo: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def _send_led_ctrl(self, chassis: Optional[float] = None, camera: Optional[float] = None) -> bool:
        """Send LED control command via lights handler."""
        try:
            # Delegate to LightsHandler via command registry
            if hasattr(self.node, '_command_registry'):
                led_data = {}
                if chassis is not None:
                    led_data['chassis_light'] = chassis
                if camera is not None:
                    led_data['camera_light'] = camera
                
                return self.node._command_registry.handle_command("lights", led_data)
            
            self.logger.warning("lights: command registry not available")
            return False
            
        except Exception as e:
            self.logger.error(f"Failed to send LED control: {e}")
            return False


class LightsHandler(CommandHandler):
    """
    Handler for UGV Beast LED/headlight control commands.
    
    This is the ONLY handler for light control (led_ctrl is deprecated).
    
    Controls:
    - IO4: Chassis headlight (near OKA camera)
    - IO5: Camera pan-tilt headlight (USB camera)
    
    Values: 0 = off, 255 = on (PWM for dimming)
    
    Status published to: {prefix}cyberwave/twin/{twin_uuid}/lights/status
    """
    
    def __init__(self, node: Node):
        super().__init__(node)
        # Track current LED states
        self._chassis_light_value = 0.0  # IO4
        self._camera_light_value = 0.0   # IO5
    
    def get_command_name(self) -> str:
        return "lights"
    
    def _setup_publishers(self) -> None:
        self._publishers['led_ctrl'] = self.node.create_publisher(
            Float32MultiArray, '/ugv/led_ctrl', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle lights command with PWM support.
        
        Supported formats:
        
        1. PWM format (recommended):
           {"pwm": 128}  // 0-255, applies to both lights
        
        2. ROS-like format:
           {"data": [255, 255]}
        
        3. Array helper format:
           {"leds": [255, 255]}
           - leds[0] = IO4 (chassis headlight)
           - leds[1] = IO5 (camera headlight)
        
        4. Named IO format:
           {"io4": 255, "io5": 0}
           {"chassis_light": 255, "camera_light": 0}
        
        5. Convenience format:
           {"all": 255}  // Turn all lights on/off
        
        Values: 0 = off, 255 = on (or 0-255 for dimming if supported)
        """
        try:
            msg = Float32MultiArray()
            
            # Format 1: PWM format (most common from frontend)
            if 'pwm' in data:
                pwm_value = data.get('pwm', 255)
                # Validate PWM range
                if not 0 <= pwm_value <= 255:
                    self.logger.warning(f"PWM value {pwm_value} out of range [0-255], clamping")
                    pwm_value = max(0, min(255, pwm_value))
                msg.data = [float(pwm_value), float(pwm_value)]
            
            # Format 2 & 3: Array formats
            elif 'data' in data:
                msg.data = [float(v) for v in data['data']]
            elif 'leds' in data:
                msg.data = [float(v) for v in data['leds']]
            
            # Format 4: Named IO format
            elif 'io4' in data or 'io5' in data:
                io4 = float(data.get('io4', self._chassis_light_value))
                io5 = float(data.get('io5', self._camera_light_value))
                msg.data = [io4, io5]
            
            # Format 5: Named descriptive format
            elif 'chassis_light' in data or 'camera_light' in data:
                chassis = float(data.get('chassis_light', self._chassis_light_value))
                camera = float(data.get('camera_light', self._camera_light_value))
                msg.data = [chassis, camera]
            
            # Format 6: Convenience - all lights same value
            elif 'all' in data:
                value = float(data['all'])
                msg.data = [value, value]
            
            else:
                self.logger.warning(
                    "lights command requires one of: 'pwm', 'data', 'leds', 'io4/io5', "
                    "'chassis_light/camera_light', or 'all'"
                )
                return False
            
            # Ensure we have exactly 2 elements as expected by UGV Beast driver
            if len(msg.data) < 2:
                msg.data.extend([0.0] * (2 - len(msg.data)))
            
            # Track current LED states
            self._chassis_light_value = msg.data[0]
            self._camera_light_value = msg.data[1]
            
            self._publishers['led_ctrl'].publish(msg)
            
            # Send response to dedicated status topic
            self.publish_response({
                "status": "success",
                "io4": msg.data[0],
                "io5": msg.data[1],
                "chassis_light": "on" if msg.data[0] > 0 else "off",
                "camera_light": "on" if msg.data[1] > 0 else "off",
                "chassis_light_value": int(self._chassis_light_value),
                "camera_light_value": int(self._camera_light_value)
            })
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle lights: {e}")
            return False


class EmergencyStopHandler(CommandHandler):
    """Handler for emergency stop commands."""
    
    def get_command_name(self) -> str:
        return "estop"
    
    def _setup_publishers(self) -> None:
        self._publishers['estop'] = self.node.create_publisher(
            Bool, '/emergency_stop', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle emergency stop command.
        
        Expected data format:
        {
            "activate": true
        }
        """
        try:
            if not self.validate_data(data, ['activate']):
                return False
            
            msg = Bool()
            msg.data = bool(data['activate'])
            
            self._publishers['estop'].publish(msg)
            self.logger.warn(f"Emergency stop {'ACTIVATED' if msg.data else 'DEACTIVATED'}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle estop: {e}")
            return False


class OledCtrlHandler(CommandHandler):
    """Handler for UGV Beast OLED display commands."""
    
    def get_command_name(self) -> str:
        return "oled_ctrl"
    
    def _setup_publishers(self) -> None:
        self._publishers['oled_ctrl'] = self.node.create_publisher(
            String, '/ugv/oled_ctrl', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle OLED control command.
        
        Expected data format:
        {"text": "Hello"} or {"data": "Hello"} or just a string "Hello"
        """
        try:
            msg = String()
            if isinstance(data, str):
                msg.data = data
            elif 'text' in data:
                msg.data = str(data['text'])
            elif 'data' in data:
                msg.data = str(data['data'])
            else:
                self.logger.warning("oled_ctrl requires 'text' or 'data' field")
                return False
            
            self._publishers['oled_ctrl'].publish(msg)
            
            # Send confirmation response
            self.publish_response({"status": "displayed", "text": msg.data})
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle oled_ctrl: {e}")
            return False


class JointTrajectoryHandler(CommandHandler):
    """Handler for sending joint trajectories (individual wheel/joint control)."""
    
    def get_command_name(self) -> str:
        return "trajectory"
    
    def _setup_publishers(self) -> None:
        self._publishers['trajectory'] = self.node.create_publisher(
            JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle trajectory command.
        
        Expected data format:
        {
            "joint_names": ["left_up_wheel_link_joint", ...],
            "points": [{
                "positions": [0.0, ...],
                "velocities": [1.0, ...],
                "time_from_start": {"sec": 1, "nanosec": 0}
            }]
        }
        """
        try:
            if not self.validate_data(data, ['joint_names', 'points']):
                return False
            
            msg = JointTrajectory()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = data['joint_names']
            
            for pt_data in data['points']:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in pt_data.get('positions', [])]
                point.velocities = [float(v) for v in pt_data.get('velocities', [])]
                
                tfs = pt_data.get('time_from_start', {})
                point.time_from_start.sec = int(tfs.get('sec', 0))
                point.time_from_start.nanosec = int(tfs.get('nanosec', 0))
                
                msg.points.append(point)
            
            self._publishers['trajectory'].publish(msg)
            
            self.publish_response({
                "status": "success",
                "joints": msg.joint_names,
                "points_count": len(msg.points)
            })
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle trajectory: {e}")
            return False


class GripperHandler(CommandHandler):
    """Handler for gripper control commands."""
    
    def get_command_name(self) -> str:
        return "gripper"
    
    def _setup_publishers(self) -> None:
        # Using String for simplicity - can be upgraded to service call
        self._publishers['gripper'] = self.node.create_publisher(
            String, '/gripper/command', 10
        )
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle gripper command.
        
        Expected data format:
        {
            "action": "grip"  // or "release", "reset"
        }
        """
        try:
            if not self.validate_data(data, ['action']):
                return False
            
            action = data['action']
            if action not in ['grip', 'release', 'reset']:
                self.logger.warning(f"Invalid gripper action: {action}")
                return False
            
            msg = String()
            msg.data = action
            
            self._publishers['gripper'].publish(msg)
            
            # Send confirmation response
            self.publish_response({"status": "executed", "action": action})
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle gripper: {e}")
            return False


class CameraServoHandler(CommandHandler):
    """
    Handler for UGV Beast pan-tilt camera servo control with smooth interpolation.
    
    Controls the pan-tilt mechanism via joint trajectory commands.
    Pan joint: pt_base_link_to_pt_link1 (horizontal rotation)
    Tilt joint: pt_link1_to_pt_link2 (vertical movement)
    """
    
    def __init__(self, node: Node):
        super().__init__(node)
        # Track current servo positions
        self._pan_position = 0.0   # radians
        self._tilt_position = 0.0  # radians
        
        # Servo limits (in radians)
        self._pan_min = -3.14159   # -180 degrees
        self._pan_max = 3.14159    # +180 degrees
        self._tilt_min = -0.785    # -45 degrees
        self._tilt_max = 1.57      # +90 degrees
        
        # Interpolation settings - optimized for fast & smooth
        self._interpolation_steps = 4  # Number of intermediate steps
        self._interpolation_interval = 0.02  # seconds between steps (20ms)
        self._interpolation_timer = None
        self._target_pan = 0.0
        self._target_tilt = 0.0
        self._current_step = 0
        
        self.logger.info("CameraServoHandler initialized with smooth interpolation")
    
    def get_command_name(self) -> str:
        return "camera_servo"
    
    def _setup_publishers(self) -> None:
        # UGV Beast uses /ugv/joint_states for servo control, not trajectory controller
        self._publishers['joint_states'] = self.node.create_publisher(
            JointState, '/ugv/joint_states', 10
        )
    
    def _interpolate_step(self) -> None:
        """Execute one step of the interpolation."""
        if self._current_step >= self._interpolation_steps:
            # Interpolation complete
            if self._interpolation_timer is not None:
                self._interpolation_timer.cancel()
                self._interpolation_timer = None
            return
        
        # Calculate interpolation progress (0.0 to 1.0)
        progress = self._current_step / self._interpolation_steps
        
        # Ease-in-out for smooth acceleration/deceleration (smoothstep function)
        progress = progress * progress * (3.0 - 2.0 * progress)
        
        # Calculate intermediate position
        start_pan = self._pan_position
        start_tilt = self._tilt_position
        
        self._pan_position = start_pan + (self._target_pan - start_pan) * progress
        self._tilt_position = start_tilt + (self._target_tilt - start_tilt) * progress
        
        # Publish intermediate position
        self._publish_position()
        
        # Move to next step
        self._current_step += 1
    
    def _publish_position(self) -> None:
        """Publish current servo position to ROS topic."""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'tele'  # CRITICAL: Must be 'tele' for ugv_integrated_driver to accept
        msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
        msg.position = [self._pan_position, self._tilt_position]
        msg.velocity = []
        msg.effort = []
        
        self._publishers['joint_states'].publish(msg)
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle camera servo command with smooth interpolation.
        
        Expected data format:
        {
            "pan_delta": 0.1,    # Optional: change in pan (radians)
            "tilt_delta": -0.1,  # Optional: change in tilt (radians)
            "pan": 0.5,          # Optional: absolute pan position (radians)
            "tilt": 0.3          # Optional: absolute tilt position (radians)
        }
        """
        try:
            self.logger.info(f"[DEBUG] CameraServoHandler.handle() called with data: {data}")
            
            # Cancel any ongoing interpolation
            if self._interpolation_timer is not None:
                self._interpolation_timer.cancel()
                self._interpolation_timer = None
            
            # Calculate target position
            pan_delta = data.get('pan_delta', 0.0)
            tilt_delta = data.get('tilt_delta', 0.0)
            
            # Check if absolute positions are provided
            if 'pan' in data:
                self._target_pan = float(data['pan'])
                self.logger.info(f"[DEBUG] Setting absolute pan position: {self._target_pan}")
            else:
                self._target_pan = self._pan_position + float(pan_delta)
                self.logger.info(f"[DEBUG] Setting relative pan position: {self._target_pan} (delta: {pan_delta})")
            
            if 'tilt' in data:
                self._target_tilt = float(data['tilt'])
                self.logger.info(f"[DEBUG] Setting absolute tilt position: {self._target_tilt}")
            else:
                self._target_tilt = self._tilt_position + float(tilt_delta)
                self.logger.info(f"[DEBUG] Setting relative tilt position: {self._target_tilt} (delta: {tilt_delta})")
            
            # Clamp target to limits
            self._target_pan = max(self._pan_min, min(self._pan_max, self._target_pan))
            self._target_tilt = max(self._tilt_min, min(self._tilt_max, self._target_tilt))
            
            # Start interpolation
            self._current_step = 1  # Start from step 1 (0 would be current position)
            
            # Create timer for interpolation steps
            self._interpolation_timer = self.node.create_timer(
                self._interpolation_interval,
                self._interpolate_step
            )
            
            self.logger.debug(
                f"Camera servo: target pan={self._target_pan:.3f}, tilt={self._target_tilt:.3f}"
            )
            
            import math
            # Convert radians to degrees for display
            pan_deg = math.degrees(self._target_pan)
            tilt_deg = math.degrees(self._target_tilt)
            
            # Send confirmation response with target position (publishes to camera_servo/status)
            self.publish_response({
                "status": "success",
                "pan": self._target_pan,
                "tilt": self._target_tilt,
                "pan_degrees": round(pan_deg, 1),
                "tilt_degrees": round(tilt_deg, 1),
                "pan_radians": round(self._target_pan, 3),
                "tilt_radians": round(self._target_tilt, 3)
            })
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle camera servo: {e}")
            return False


class BatteryCheckHandler(CommandHandler):
    """
    Handler for explicit battery level check.
    
    Publishes to the same topic as automatic battery updates:
    cyberwave/twin/{twin_uuid}/battery/status
    """
    
    def get_command_name(self) -> str:
        return "battery_check"
    
    def _setup_publishers(self) -> None:
        pass
    
    def get_status_topic(self) -> Optional[str]:
        """
        Override to publish to battery/status instead of battery_check/status.
        This ensures command responses use the same topic as automatic telemetry.
        """
        try:
            mapping = getattr(self.node, '_mapping', None)
            if mapping and hasattr(mapping, 'twin_uuid'):
                twin_uuid = mapping.twin_uuid
                prefix = getattr(self.node, 'ros_prefix', '')
                # Publish to battery/status (same as automatic updates)
                return f"{prefix}cyberwave/twin/{twin_uuid}/battery/status"
        except Exception:
            return None
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle battery check command.
        
        Publishes battery status to: {prefix}cyberwave/twin/{twin_uuid}/battery/status
        Same format as automatic updates.
        """
        try:
            import time
            
            self.logger.info("Battery check command received")
            
            # Try to get battery data from the node's cache
            battery_msg = None
            if hasattr(self.node, '_last_battery_msg') and self.node._last_battery_msg:
                battery_msg = self.node._last_battery_msg
                self.logger.info(f"Found cached battery message: {type(battery_msg).__name__}")
            else:
                self.logger.warning("No battery message cached yet")
            
            if battery_msg:
                # Extract battery info
                # Check if it's a BatteryState or Float32 message
                if hasattr(battery_msg, 'voltage'):
                    # BatteryState message
                    voltage = float(battery_msg.voltage)
                    percentage = float(battery_msg.percentage) if hasattr(battery_msg, 'percentage') else None
                    self.logger.info(f"BatteryState: voltage={voltage}V, percentage={percentage}")
                elif hasattr(battery_msg, 'data'):
                    # Float32 message - calculate percentage
                    voltage = float(battery_msg.data)
                    percentage = (voltage - 9.0) / (12.6 - 9.0)
                    percentage = float(max(0.0, min(1.0, percentage)))
                    self.logger.info(f"Float32: voltage={voltage}V, calculated percentage={percentage}")
                else:
                    voltage = None
                    percentage = None
                    self.logger.error("Unknown battery message format")
                
                if voltage is not None:
                    # Get the status topic (battery/status)
                    status_topic = self.get_status_topic()
                    if status_topic:
                        # Publish directly to the topic (not through publish_response)
                        # to use the same format as automatic updates
                        payload = {
                            "source_type": "edge",
                            "voltage": voltage,
                            "percentage": percentage if percentage is not None else 0.0,
                            "timestamp": time.time()
                        }
                        
                        self.logger.info(f"Publishing battery status to {status_topic}: {payload}")
                        
                        # Use the adapter or client to publish
                        if hasattr(self.node, '_mqtt_adapter') and self.node._mqtt_adapter:
                            self.node._mqtt_adapter.publish(status_topic, payload)
                            self.logger.info("Published via adapter")
                        elif hasattr(self.node, '_mqtt_client') and self.node._mqtt_client:
                            import json
                            self.node._mqtt_client.publish(status_topic, json.dumps(payload))
                            self.logger.info("Published via paho client")
                        else:
                            self.logger.error("No MQTT client available")
                        
                        return True
                    else:
                        self.logger.error("Could not determine status topic")
                
                # Fallback if we can't extract voltage
                self.logger.error("Could not extract voltage from battery message")
                self.publish_response({
                    "status": "error",
                    "message": "Could not extract voltage from battery message"
                })
            else:
                self.logger.warning("Battery data not available yet - no cached message")
                self.publish_response({
                    "status": "error",
                    "message": "Battery data not available yet"
                })
            return True
        except Exception as e:
            self.logger.error(f"Failed to handle battery check: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False


class TakePhotoHandler(CommandHandler):
    """Handler for taking a photo/snapshot from the camera."""
    
    def get_command_name(self) -> str:
        return "take_photo"
    
    def _setup_publishers(self) -> None:
        pass
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """Handle take_photo command."""
        try:
            if not CV2_AVAILABLE:
                self.publish_response({
                    "status": "error",
                    "command": "take_photo",
                    "message": "OpenCV not available for image encoding"
                })
                return False
            
            # Try to get the latest frame from the camera streamer
            frame = None
            if hasattr(self.node, '_ros_streamer') and self.node._ros_streamer:
                if hasattr(self.node._ros_streamer, 'streamer') and self.node._ros_streamer.streamer:
                    if hasattr(self.node._ros_streamer.streamer, 'latest_frame'):
                        frame = self.node._ros_streamer.streamer.latest_frame
            
            if frame is None:
                self.publish_response({
                    "status": "error",
                    "command": "take_photo",
                    "message": "No camera frame available"
                })
                return False
            
            # Encode frame to JPEG
            success, encoded_image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            if not success:
                self.publish_response({
                    "status": "error",
                    "command": "take_photo",
                    "message": "Failed to encode image"
                })
                return False
            
            # Convert to base64
            image_base64 = base64.b64encode(encoded_image.tobytes()).decode('utf-8')
            
            # Get twin UUID for topic
            twin_uuid = None
            if hasattr(self.node, '_mapping') and self.node._mapping:
                twin_uuid = getattr(self.node._mapping, 'twin_uuid', None)
            
            if twin_uuid:
                prefix = getattr(self.node, 'ros_prefix', '')
                
                # Publish the captured image to a dedicated photo topic
                photo_topic = f"{prefix}cyberwave/twin/{twin_uuid}/camera/photo"
                photo_payload = {
                    "source_type": "edge",
                    "timestamp": self.node.get_clock().now().nanoseconds / 1e9,
                    "image": image_base64,
                    "format": "jpeg",
                    "width": frame.shape[1],
                    "height": frame.shape[0]
                }
                # Use node.publish() which properly serializes dict to JSON
                # and handles both CyberwaveAdapter and paho-mqtt fallback
                self.node.publish(photo_topic, photo_payload)
                self.logger.info(f"Published photo to {photo_topic} ({len(image_base64)} bytes)")
            
            # Send success response
            self.publish_response({
                "status": "success",
                "command": "take_photo",
                "message": "Photo captured successfully",
                "image_size": len(image_base64)
            })
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle take_photo: {e}")
            self.publish_response({
                "status": "error",
                "command": "take_photo",
                "message": f"Exception: {str(e)}"
            })
            return False


class CameraCommandHandler(CommandHandler):
    """Handles frontend video streaming commands."""
    def __init__(self, node: Node, command_name: str):
        super().__init__(node)
        self._command_name = command_name

    def get_command_name(self) -> str:
        return self._command_name
        
    def _setup_publishers(self) -> None:
        pass

    def handle(self, data: Any) -> bool:
        # Commands from FE: "start_video" or "stop_video"
        if self._command_name == "start_video":
            self.node.start_camera_stream()
            self.publish_response({"status": "ok", "type": "video_started"})
        elif self._command_name == "stop_video":
            self.node.stop_camera_stream()
            self.publish_response({"status": "ok", "type": "video_stopped"})
        return True

class StatusQueryHandler(CommandHandler):
    """Handler for querying robot status (IMU, Battery, Odom)."""
    
    def get_command_name(self) -> str:
        return "get_status"
    
    def _setup_publishers(self) -> None:
        # No publishers needed, we just read from the node's state
        pass
    
    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle status query command.
        
        Expected data format:
        {
            "target": "battery"  // or "imu", "odom", "joint_states", "all"
        }
        """
        try:
            target = data.get('target', 'all')
            
            # Map simple target names to actual ROS topics
            topic_map = {
                "battery": "/ugv/battery_status",
                "imu": "/ugv/imu",
                "odom": "/odom",
                "joint_states": "/ugv/joint_states",
                "cmd_vel": "/cmd_vel",
                "led_ctrl": "/ugv/led_ctrl",
                "oled_ctrl": "/ugv/oled_ctrl"
            }
            
            # Get the cache from the node
            cache = getattr(self.node, '_ros_state_cache', {})
            
            if target == 'all':
                response_data = {}
                for name, topic in topic_map.items():
                    msg = cache.get(topic)
                    if msg:
                        response_data[name] = self._serialize_msg(msg)
                self.publish_response({
                    "status": "success",
                    "data": response_data
                })
            elif target in topic_map:
                topic = topic_map[target]
                msg = cache.get(topic)
                if msg:
                    self.publish_response({
                        "status": "success",
                        "target": target,
                        "data": self._serialize_msg(msg)
                    })
                else:
                    self.publish_response({
                        "status": "error",
                        "message": f"No data cached for {target} ({topic})"
                    })
            else:
                self.publish_response({
                    "status": "error",
                    "message": f"Unknown status target: {target}. Available: {list(topic_map.keys())}"
                })
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to handle status query: {e}")
            return False

    def _serialize_msg(self, msg) -> Any:
        """Helper to serialize ROS message to dict."""
        try:
            # Try to use the node's encoder if available (it handles JointState and battery status specially)
            if hasattr(self.node, '_encode_msg_for_mqtt'):
                import json
                # Determine context topic if possible
                context_topic = None
                if 'battery' in str(type(msg)).lower() or 'float' in str(type(msg)).lower():
                    # For battery status, we need to pass a topic containing 'status/battery' to trigger special encoding
                    # if the type is Float32.
                    context_topic = 'cyberwave/twin/unknown/status/battery'
                
                encoded = self.node._encode_msg_for_mqtt(msg, type(msg), mqtt_topic=context_topic)
                return json.loads(encoded)
            
            # Fallback to direct conversion
            from rosidl_runtime_py.convert import message_to_ordereddict
            return message_to_ordereddict(msg)
        except Exception:
            return str(msg)


class CommandRegistry:
    """
    Registry for command handlers.
    
    Maintains a mapping of command names to handler instances.
    Provides methods to register handlers and route commands.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the command registry.
        
        Args:
            node: ROS2 node for creating handlers
        """
        self.node = node
        self.logger = node.get_logger()
        self._handlers: Dict[str, CommandHandler] = {}
        self._mqtt_adapter: Any = None
        self._command_topic: Optional[str] = None
        self._register_default_handlers()
    
    def set_mqtt_context(self, adapter: Any, command_topic: str):
        """Pass MQTT context to all registered handlers."""
        self._mqtt_adapter = adapter
        self._command_topic = command_topic
        for handler in self._handlers.values():
            handler.set_mqtt_context(adapter, command_topic)

    def _register_default_handlers(self) -> None:
        """Register all default command handlers."""
        # NOTE: cmd_vel and led_ctrl handlers are deprecated
        # Use 'actuation' for movement and 'lights' for LED control
        default_handlers = [
            GenericActuationHandler,  # Handles all movement commands (move_forward, turn_left, stop, etc.)
            LightsHandler,            # Handles lights command (replaces led_ctrl)
            EmergencyStopHandler,
            OledCtrlHandler,
            JointTrajectoryHandler,
            GripperHandler,
            StatusQueryHandler,
            BatteryCheckHandler,
            TakePhotoHandler,
            CameraServoHandler,
        ]
        
        for handler_class in default_handlers:
            self.register_handler(handler_class)
            
        # Register specialized camera handlers for start/stop to match FE button
        self.register_handler_instance(CameraCommandHandler(self.node, "start_video"))
        self.register_handler_instance(CameraCommandHandler(self.node, "stop_video"))
    
    def register_handler(self, handler_class: type) -> None:
        """
        Register a command handler class.
        
        Args:
            handler_class: CommandHandler subclass to register
        """
        try:
            handler = handler_class(self.node)
            command_name = handler.get_command_name()
            self._handlers[command_name] = handler
            self.logger.info(f"Registered command handler: {command_name}")
        except Exception as e:
            self.logger.error(f"Failed to register handler {handler_class.__name__}: {e}")
    
    def register_handler_instance(self, handler: CommandHandler) -> None:
        """
        Register an already-instantiated command handler.
        
        Args:
            handler: CommandHandler instance to register
        """
        command_name = handler.get_command_name()
        self._handlers[command_name] = handler
        self.logger.info(f"Registered command handler: {command_name}")
    
    def handle_command(self, command: str, data: Dict[str, Any]) -> bool:
        """
        Route a command to the appropriate handler.
        
        Args:
            command: Command name/type
            data: Command data dictionary
            
        Returns:
            True if command was handled successfully, False otherwise
        """
        if command not in self._handlers:
            self.logger.warning(
                f"No handler registered for command: {command}. "
                f"Available: {list(self._handlers.keys())}"
            )
            return False
        
        handler = self._handlers[command]
        return handler.handle(data)
    
    def get_registered_commands(self) -> list:
        """Get list of all registered command names."""
        return list(self._handlers.keys())
    
    def unregister_handler(self, command_name: str) -> bool:
        """
        Unregister a command handler.
        
        Args:
            command_name: Name of command to unregister
            
        Returns:
            True if handler was found and removed, False otherwise
        """
        if command_name in self._handlers:
            del self._handlers[command_name]
            self.logger.info(f"Unregistered command handler: {command_name}")
            return True
        return False


class NavigationProxyHandler(CommandHandler):
    """
    A proxy handler that forwards MQTT commands to a navigation-specific plugin.
    
    This is used when 'plugin' is specified in the YAML configuration to route
    commands to more specialized ROS-Nav2 components like NavigationBridge.
    """
    
    def __init__(self, node: Node, command_name: str, plugin_instance: Any):
        """
        Initialize proxy handler.
        
        Args:
            node: The bridge node
            command_name: The command this handler will register as (e.g. 'goto')
            plugin_instance: The object instance to call when command arrives
        """
        # We must set simple attributes before calling super().__init__ 
        # because super() calls _setup_publishers()
        self._cmd_name = command_name
        self._plugin = plugin_instance
        super().__init__(node)

    def get_command_name(self) -> str:
        return self._cmd_name

    def _setup_publishers(self) -> None:
        """
        No publishers needed here as we delegate to a plugin.
        """
        pass

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Delegate command handling to the plugin.
        
        Tries calling handle_command(command, data) or similar method on plugin.
        """
        self.logger.info(f"Proxying command '{self._cmd_name}' to {type(self._plugin).__name__}")
        
        # Priority 1: Generic handle_command method
        if hasattr(self._plugin, 'handle_command'):
            return self._plugin.handle_command(self._cmd_name, data)
            
        # Priority 2: Direct method name match (e.g. plugin.goto(data))
        if hasattr(self._plugin, self._cmd_name):
            method = getattr(self._plugin, self._cmd_name)
            if callable(method):
                return method(data)
                
        self.logger.error(f"Plugin {type(self._plugin).__name__} lacks handler for '{self._cmd_name}'")
        return False


class NavigationBridge:
    """
    Bridge between MQTT navigation commands and ROS2 Nav2 actions.
    """
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self._setup_publishers()

    def _setup_publishers(self) -> None:
        """Initialize any publishers or action clients needed for navigation."""
        # The 'goto' command usually maps to a goal pose for Nav2
        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        # The 'path' command maps to a planned path for visualization or tracking
        self._path_pub = self.node.create_publisher(
            Path,
            '/plan',
            10
        )

    def handle_command(self, command: str, data: Dict[str, Any]) -> bool:
        """Handle incoming navigation commands."""
        if command == 'goto':
            return self.goto(data)
        elif command == 'path':
            return self.path(data)
        
        self.logger.warning(f"NavigationBridge received unhandled command: {command}")
        return False

    def goto(self, data: Dict[str, Any]) -> bool:
        """
        Process 'goto' command and publish to Nav2.
        
        Expected data format:
        {
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0,  # optional
            "frame_id": "map"  # optional
        }
        """
        try:
            pose = PoseStamped()
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.frame_id = data.get('frame_id', 'map')
            
            pose.pose.position.x = float(data.get('x', 0.0))
            pose.pose.position.y = float(data.get('y', 0.0))
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion if provided
            yaw = float(data.get('yaw', 0.0))
            import math
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self._goal_pub.publish(pose)
            self.logger.info(f"Published goal pose: x={pose.pose.position.x}, y={pose.pose.position.y}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to handle 'goto' command: {e}")
            return False

    def path(self, data: Dict[str, Any]) -> bool:
        """
        Process 'path' command and publish to Nav2.
        
        Expected data format:
        {
            "points": [{"x": 1.0, "y": 2.0}, ...],
            "frame_id": "map"  # optional
        }
        """
        try:
            path_msg = Path()
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            path_msg.header.frame_id = data.get('frame_id', 'map')
            
            for pt in data.get('points', []):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(pt.get('x', 0.0))
                pose.pose.position.y = float(pt.get('y', 0.0))
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            self._path_pub.publish(path_msg)
            self.logger.info(f"Published path with {len(path_msg.poses)} points")
            return True
        except Exception as e:
            self.logger.error(f"Failed to handle 'path' command: {e}")
            return False


# Decorator for easy handler registration
def command_handler(command_name: str):
    """
    Decorator for registering command handlers.
    
    Usage:
        @command_handler("my_command")
        class MyCommandHandler(CommandHandler):
            ...
    """
    def decorator(cls):
        cls._command_name = command_name
        return cls
    return decorator

