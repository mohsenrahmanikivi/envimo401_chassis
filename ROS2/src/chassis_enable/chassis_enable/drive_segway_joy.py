#!/usr/bin/env python3
"""
drive_segway_joy.py

ROS2 Python node to control Segway RMP 401 with a gamepad (joy topic).
- Subscribes to /joy (sensor_msgs/msg/Joy)
- Publishes geometry_msgs/msg/Twist on /cmd_vel
- Calls segway enable service (segway_msgs/srv/RosSetChassisEnableCmd) with True/False

Features:
- Button-driven increments for linear and angular velocity (default step 0.1)
- Holding a button continuously changes velocity by step (default)
- Rising-edge detection for enable/disable service calls to avoid spamming
- Velocities clamped to robot limits (default: max_linear=2.0 m/s, max_angular=0.87 rad/s)
- All important values (button indices, steps, topics, rate) are ROS parameters for easy reconfiguration

Place this file in a ROS2 Python package (e.g. <your_pkg>/scripts/) and make executable.
Add an entry point in setup.cfg/setup.py or run directly with `ros2 run <pkg> drive_segway_joy` after packaging.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd


class DriveSegwayJoy(Node):
    def __init__(self):
        super().__init__('drive_segway_joy')

        # --- Parameters (declare with defaults so they can be overridden via launch/YAML) ---
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service_name', 'RosSetChassisEnableCmd')

        # Button mapping (defaults) - change to match your controller or override via parameters
        self.declare_parameter('inc_linear_btn', 4)
        self.declare_parameter('dec_linear_btn', 6)
        self.declare_parameter('inc_angular_btn', 7)
        self.declare_parameter('dec_angular_btn', 5)
        self.declare_parameter('enable_btn', 0)
        self.declare_parameter('disable_btn', 1)

        # Steps and limits
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('max_angular', 0.87)

        # Behavior flags and rate
        self.declare_parameter('hold_to_change', True)
        self.declare_parameter('publish_rate', 20.0)  # Hz

        # Read params
        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.enable_service_name = self.get_parameter('enable_service_name').get_parameter_value().string_value

        self.inc_linear_btn = self.get_parameter('inc_linear_btn').get_parameter_value().integer_value
        self.dec_linear_btn = self.get_parameter('dec_linear_btn').get_parameter_value().integer_value
        self.inc_angular_btn = self.get_parameter('inc_angular_btn').get_parameter_value().integer_value
        self.dec_angular_btn = self.get_parameter('dec_angular_btn').get_parameter_value().integer_value
        self.enable_btn = self.get_parameter('enable_btn').get_parameter_value().integer_value
        self.disable_btn = self.get_parameter('disable_btn').get_parameter_value().integer_value

        self.linear_step = self.get_parameter('linear_step').get_parameter_value().double_value
        self.angular_step = self.get_parameter('angular_step').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value

        self.hold_to_change = self.get_parameter('hold_to_change').get_parameter_value().bool_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        if publish_rate <= 0.0:
            publish_rate = 20.0
        self.publish_period = 1.0 / publish_rate

        # Internal state
        self.last_joy = None
        self.prev_buttons = []
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Publisher and subscriber
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, qos)

        # Service client (enable/disable)
        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service_name)

        # Timer to publish at fixed rate and process hold behaviour
        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info(f"drive_segway_joy started: publishing '{self.cmd_vel_topic}' at {publish_rate} Hz")
        self.get_logger().info(
            f"Button mapping: inc_lin={self.inc_linear_btn} dec_lin={self.dec_linear_btn} inc_ang={self.inc_angular_btn} "
            f"dec_ang={self.dec_angular_btn} enable={self.enable_btn} disable={self.disable_btn}")

    def joy_callback(self, msg: Joy):
        # store latest joy and ensure prev_buttons sized
        self.last_joy = msg
        if not self.prev_buttons:
            # initialize prev_buttons
            self.prev_buttons = list(msg.buttons)
        elif len(self.prev_buttons) != len(msg.buttons):
            # re-initialize if sizes changed
            self.prev_buttons = list(msg.buttons)

    def timer_callback(self):
        if self.last_joy is not None:
            buttons = self.last_joy.buttons

            def is_pressed(idx: int) -> bool:
                return 0 <= idx < len(buttons) and buttons[idx] != 0

            def is_rising(idx: int) -> bool:
                cur = 0 <= idx < len(buttons) and buttons[idx] != 0
                prev = 0 <= idx < len(self.prev_buttons) and self.prev_buttons[idx] != 0
                return cur and not prev

            # Linear adjustments
            if self.hold_to_change:
                if is_pressed(self.inc_linear_btn):
                    self.linear_vel += self.linear_step
                if is_pressed(self.dec_linear_btn):
                    self.linear_vel -= self.linear_step
            else:
                if is_rising(self.inc_linear_btn):
                    self.linear_vel += self.linear_step
                if is_rising(self.dec_linear_btn):
                    self.linear_vel -= self.linear_step

            # Angular adjustments
            if self.hold_to_change:
                if is_pressed(self.inc_angular_btn):
                    self.angular_vel += self.angular_step
                if is_pressed(self.dec_angular_btn):
                    self.angular_vel -= self.angular_step
            else:
                if is_rising(self.inc_angular_btn):
                    self.angular_vel += self.angular_step
                if is_rising(self.dec_angular_btn):
                    self.angular_vel -= self.angular_step

            # Clamp speeds
            self.linear_vel = max(-self.max_linear, min(self.max_linear, self.linear_vel))
            self.angular_vel = max(-self.max_angular, min(self.max_angular, self.angular_vel))

            # Publish Twist
            twist = Twist()
            twist.linear.x = float(self.linear_vel)
            twist.angular.z = float(self.angular_vel)
            self.cmd_vel_pub.publish(twist)

            # Handle enable/disable on rising edge only to avoid spamming
            if is_rising(self.enable_btn):
                self.call_enable_service(True)
            if is_rising(self.disable_btn):
                self.call_enable_service(False)

            # save current buttons for next loop
            self.prev_buttons = list(buttons)

        else:
            # no joy yet: publish zero twist periodically
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Enable service '{self.enable_service_name}' not available")
            return

        req = RosSetChassisEnableCmd.Request()
        # NOTE: field name may differ depending on segway_msgs generation.
        # The C++ example used 'ros_set_chassis_enable_cmd' — if your generated Request
        # uses another name, change the next line accordingly.
        try:
            req.ros_set_chassis_enable_cmd = bool(enable)
        except Exception:
            # fallback attempt: try setting first available boolean-like field
            for field in req.get_fields_and_field_types().keys():
                try:
                    setattr(req, field, bool(enable))
                    break
                except Exception:
                    continue

        future = self.enable_client.call_async(req)
        # Do not block; log the call
        self.get_logger().info(f"Sent {'enable' if enable else 'disable'} request to {self.enable_service_name}")


def main(args=None):
    rclpy.init(args=args)
    node = DriveSegwayJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
