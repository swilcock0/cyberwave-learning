"""ROS 2 node: forwards /odometry/filtered -> MQTT.

Runs inside the ros2_ekf container alongside the EKF node.
MQTT credentials are resolved from the /mqtt_bridge_node ROS 2 parameter
server first, then fall back to environment variables.

Env vars (all optional when /mqtt_bridge_node is reachable):
    CYBERWAVE_MQTT_HOST  - MQTT broker hostname
    MQTT_PORT            - broker port (default: 8883)
    MQTT_TLS             - set "0" to disable TLS (default: enabled for 8883)
    CYBERWAVE_TWIN_UUID  - twin UUID (falls back to robot_id param)
    CYBERWAVE_TOKEN      - Cyberwave API token (used as MQTT password when set)
    MQTT_USERNAME        - explicit MQTT username
    MQTT_PASSWORD        - explicit MQTT password
    ODOM_ROS_TOPIC       - ROS topic to subscribe (default: /odometry/filtered)
    ODOM_MQTT_TOPIC      - MQTT topic to publish
                           (default: cyberwave/twin/{twin_uuid}/odometry_filtered)
    PUBLISH_RATE_HZ      - max publish rate in Hz (default: 10.0)
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry

import paho.mqtt.client as mqtt_client_lib


def _env(key: str, default: str = "") -> str:
    return os.environ.get(key, default)


class OdometryFilteredBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("odometry_filtered_bridge")

        cfg = self._resolve_mqtt_config()

        ros_topic = _env("ODOM_ROS_TOPIC", "/odometry/filtered")
        mqtt_topic_tmpl = _env(
            "ODOM_MQTT_TOPIC",
            f"cyberwave/twin/{cfg['twin_uuid']}/odometry_filtered",
        )
        self._mqtt_topic = mqtt_topic_tmpl.replace("{twin_uuid}", cfg["twin_uuid"])

        rate_hz = float(_env("PUBLISH_RATE_HZ", "5.0"))
        self._min_interval = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._last_publish = 0.0

        # ── MQTT setup ───────────────────────────────────────────────────────
        use_tls = _env("MQTT_TLS", "1") != "0"

        self._mqtt = mqtt_client_lib.Client()
        if cfg["username"]:
            self._mqtt.username_pw_set(cfg["username"], cfg["password"])
        if use_tls and cfg["port"] == 8883:
            self._mqtt.tls_set()

        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect

        try:
            self._mqtt.connect(cfg["host"], cfg["port"], keepalive=60)
            self._mqtt.loop_start()
            self.get_logger().info(f"MQTT connecting to {cfg['host']}:{cfg['port']}")
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")

        # ── ROS subscriber ───────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Odometry, ros_topic, self._odom_cb, 10
        )
        self.get_logger().info(
            f"Subscribed to {ros_topic} (nav_msgs/Odometry) "
            f"-> MQTT {self._mqtt_topic} @ {rate_hz} Hz"
        )

    # ── Credential resolution ─────────────────────────────────────────────────
    def _resolve_mqtt_config(self) -> dict:
        """Read credentials from /mqtt_bridge_node via GetParameters service.

        Works on rclpy 3.x (Humble) which lacks rclpy.parameter_client.
        Falls back to env vars if the service is unavailable.
        """
        cfg = {
            "host":      _env("CYBERWAVE_MQTT_HOST") or _env("CYBERWAVE_MQTT_BROKER", "mqtt.cyberwave.com"),
            "port":      int(_env("MQTT_PORT", "8883")),
            "username":  _env("MQTT_USERNAME"),
            "password":  _env("MQTT_PASSWORD"),
            "token":     _env("CYBERWAVE_TOKEN") or _env("CYBERWAVE_API_KEY"),
            "twin_uuid": _env("CYBERWAVE_TWIN_UUID", ""),
        }
        if not cfg["username"] and cfg["token"]:
            cfg["username"] = cfg["token"]
            cfg["password"] = cfg["token"]

        _PARAMS = ["broker.host", "broker.port", "broker.username",
                   "broker.password", "cyberwave_token", "robot_id"]
        try:
            cli = self.create_client(
                GetParameters, "/mqtt_bridge_node/get_parameters"
            )
            if cli.wait_for_service(timeout_sec=3.0):
                req = GetParameters.Request()
                req.names = _PARAMS
                future = cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                result = future.result()
                if result:
                    # ParameterValue: type 0=unset,1=bool,2=int,3=float,4=string
                    def _str(pv):
                        if pv.type == 4:
                            return pv.string_value
                        if pv.type == 2:
                            return str(pv.integer_value)
                        return ""
                    pmap = {name: _str(pv)
                            for name, pv in zip(_PARAMS, result.values)}
                    if pmap.get("broker.host"):
                        cfg["host"] = pmap["broker.host"]
                    if pmap.get("broker.port"):
                        cfg["port"] = int(pmap["broker.port"])
                    if pmap.get("broker.username"):
                        cfg["username"] = pmap["broker.username"]
                    if pmap.get("broker.password"):
                        cfg["password"] = pmap["broker.password"]
                    if pmap.get("cyberwave_token") and not _env("MQTT_USERNAME"):
                        cfg["token"] = pmap["cyberwave_token"]
                        cfg["username"] = cfg["username"] or cfg["token"]
                        cfg["password"] = cfg["password"] or cfg["token"]
                    if not cfg["twin_uuid"] and pmap.get("robot_id"):
                        cfg["twin_uuid"] = pmap["robot_id"]
                    self.get_logger().info(
                        "MQTT credentials loaded from /mqtt_bridge_node param server"
                    )
            else:
                self.get_logger().warning(
                    "/mqtt_bridge_node param service unavailable, using env vars"
                )
        except Exception as e:
            self.get_logger().warning(
                f"Could not read param server: {e} — using env vars"
            )
        finally:
            self.destroy_client(cli) if "cli" in dir() else None

        if not cfg["twin_uuid"]:
            cfg["twin_uuid"] = "unknown"
        return cfg

    # ── MQTT callbacks ────────────────────────────────────────────────────────
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected")
        else:
            self.get_logger().error(f"MQTT connect failed rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warning(f"MQTT disconnected rc={rc}")

    # ── ROS callback ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self._min_interval > 0 and (now - self._last_publish) < self._min_interval:
            return
        self._last_publish = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        payload = json.dumps({
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "position":         {"x": p.x, "y": p.y, "z": p.z},
            "orientation":      {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
            "linear_velocity":  {"x": v.x, "y": v.y, "z": v.z},
            "angular_velocity": {"x": w.x, "y": w.y, "z": w.z},
        })

        try:
            self._mqtt.publish(self._mqtt_topic, payload, qos=1)
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def destroy_node(self):
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = OdometryFilteredBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
