#!/usr/bin/env python3
"""
drive_segway_joy.py

ROS2 Python node to control Segway RMP 401 with a gamepad (joy topic) using analog sticks.
- Subscribes to /joy (sensor_msgs/msg/Joy)
- Publishes geometry_msgs/msg/Twist on /cmd_vel
- Calls segway enable service (segway_msgs/srv/RosSetChassisEnableCmd) with True/False

Behavior changes from previous version:
- Linear and angular velocities are driven directly by analog stick axes (not by buttons and increments).
  * Linear velocity <- (axis value) * max_linear
  * Angular velocity <- (axis value) * max_angular
- No "hold to increase" behavior. Pushing the stick further increases the commanded value.
- Buttons are only used for enable / disable (rising-edge detection to avoid spamming).

DEFAULT AXIS & BUTTON MAPPING (changeable via ROS params):
  linear_axis   (default 1) : Left stick vertical (push forward -> negative on many controllers; `invert_linear_axis` param exists)
  angular_axis  (default 3) : Right stick horizontal
  enable_btn    (default 0) : Call enable service with `true` on rising edge (enable robot)
  disable_btn   (default 1) : Call enable service with `false` on rising edge (disable robot)

Tuneable parameters:
  max_linear        (default 2.0)  # m/s
  max_angular       (default 0.87) # rad/s (~50 deg/s)
  deadzone          (default 0.05) # axis values within +/- deadzone are treated as zero
  invert_linear_axis (default True) # invert axis sign if forward maps to negative
  invert_angular_axis (default False)
  publish_rate      (default 20.0)  # Hz

Notes:
- Axis indices vary by OS/driver; typical mappings (may differ):
    Xbox/Generic: left vertical = 1, right horizontal = 3
    PS4/DS4 (via some drivers): left vertical = 1, right horizontal = 2
  Please confirm your `joy` messages (ros2 topic echo /joy) and adjust `linear_axis`/`angular_axis` params accordingly.

Usage:
- Put this file into your ROS2 Python package (scripts/ or package module), make executable, add entry point in setup.cfg/setup.py as previously provided.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd


class DriveSegwayJoy(Node):
    def __init__(self):
        super().__init__('drive_segway_joy')

        # Parameters (defaults)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service_name', 'RosSetChassisEnableCmd')

        # Axis mapping (analog sticks)
        self.declare_parameter('linear_axis', 1)    # default: left stick vertical
        self.declare_parameter('angular_axis', 3)   # default: right stick horizontal
        self.declare_parameter('invert_linear_axis', True)
        self.declare_parameter('invert_angular_axis', False)

        # Buttons for enable/disable
        self.declare_parameter('enable_btn', 0)
        self.declare_parameter('disable_btn', 1)

        # Limits and tuning
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('max_angular', 0.87)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('publish_rate', 20.0)

        # Read parameters
        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.enable_service_name = self.get_parameter('enable_service_name').get_parameter_value().string_value

        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.invert_linear = self.get_parameter('invert_linear_axis').get_parameter_value().bool_value
        self.invert_angular = self.get_parameter('invert_angular_axis').get_parameter_value().bool_value

        self.enable_btn = self.get_parameter('enable_btn').get_parameter_value().integer_value
        self.disable_btn = self.get_parameter('disable_btn').get_parameter_value().integer_value

        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        if publish_rate <= 0.0:
            publish_rate = 20.0
        self.publish_period = 1.0 / publish_rate

        # Internal state
        self.last_joy = None
        self.prev_buttons = []

        # Publisher & subscriber
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, qos)

        # Service client for enabling/disabling the chassis
        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service_name)

        # Timer to publish at steady rate
        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info(f"drive_segway_joy (axes) started: publishing '{self.cmd_vel_topic}' at {publish_rate} Hz")
        self.get_logger().info(
            f"Axes mapping: linear_axis={self.linear_axis} (invert={self.invert_linear}), "
            f"angular_axis={self.angular_axis} (invert={self.invert_angular})")
        self.get_logger().info(f"Enable/disable buttons: enable={self.enable_btn}, disable={self.disable_btn}")

    def joy_callback(self, msg: Joy):
        # store latest joy and keep prev_buttons sized for rising-edge detection
        self.last_joy = msg
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
        elif len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = list(msg.buttons)

    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.deadzone:
            return 0.0
        return v

    def get_axis_value(self, axes, idx: int, invert: bool) -> float:
        if idx < 0 or idx >= len(axes):
            return 0.0
        val = float(axes[idx])
        if invert:
            val = -val
        return val

    def timer_callback(self):
        twist = Twist()

        if self.last_joy is not None:
            axes = self.last_joy.axes
            buttons = self.last_joy.buttons

            # Read axis values (direct mapping)
            raw_linear = self.get_axis_value(axes, self.linear_axis, self.invert_linear)
            raw_angular = self.get_axis_value(axes, self.angular_axis, self.invert_angular)

            # Apply deadzone
            linear_axis_val = self.apply_deadzone(raw_linear)
            angular_axis_val = self.apply_deadzone(raw_angular)

            # Map to speeds
            linear_cmd = linear_axis_val * self.max_linear
            angular_cmd = angular_axis_val * self.max_angular

            # Clamp (just in case)
            linear_cmd = max(-self.max_linear, min(self.max_linear, linear_cmd))
            angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))

            twist.linear.x = linear_cmd
            twist.angular.z = angular_cmd

            # Publish
            self.cmd_vel_pub.publish(twist)

            # Enable/disable on rising edge
            def is_rising(idx: int) -> bool:
                cur = 0 <= idx < len(buttons) and buttons[idx] != 0
                prev = 0 <= idx < len(self.prev_buttons) and self.prev_buttons[idx] != 0
                return cur and not prev

            if is_rising(self.enable_btn):
                self.call_enable_service(True)
            if is_rising(self.disable_btn):
                self.call_enable_service(False)

            # save current buttons for next loop
            self.prev_buttons = list(buttons)

        else:
            # No joy yet — publish zero to be safe
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Enable service '{self.enable_service_name}' not available")
            return

        req = RosSetChassisEnableCmd.Request()
        try:
            req.ros_set_chassis_enable_cmd = bool(enable)
        except Exception:
            # fallback attempt: set first boolean-like field
            for field in req.get_fields_and_field_types().keys():
                try:
                    setattr(req, field, bool(enable))
                    break
                except Exception:
                    continue

        self.enable_client.call_async(req)
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
