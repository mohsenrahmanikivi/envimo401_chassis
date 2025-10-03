#!/usr/bin/env python3
"""
drive_segway_joy.py

Controls Segway RMP401 using:
 - Left stick left/right -> angular velocity (proportional)
 - D-pad left/right -> fixed minimum angular (when stick near center)
 - D-pad up/down -> fixed minimum positive/negative linear
 - R1 -> fixed positive linear
 - R2 (analog trigger) -> positive linear proportional to trigger
 - L1 -> fixed negative linear
 - L2 (analog trigger) -> negative linear proportional to trigger
 - Enable/Disable: calls service /set_chassis_enable (segway_msgs/srv/RosSetChassisEnableCmd)
   with parameter 'ros_set_chassis_enable_cmd' set to True/False on rising edge.

Tune everything via ROS2 parameters (axis/button indices, min speeds, scales, deadzones).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd

from typing import List


class DriveSegwayJoy(Node):
    def __init__(self):
        super().__init__('drive_segway_joy')

        # --- Topics & service name ---
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        # exact enable service topic you provided:
        self.declare_parameter('enable_service_name', '/set_chassis_enable')

        # --- Axes mapping (defaults — update to match your controller) ---
        # Left stick horizontal -> angular control
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('invert_angular_axis', False)

        # Triggers axes (analog). Many controllers use values in [-1,1] or [0,1].
        self.declare_parameter('r2_axis', 5)  # right trigger (analog, positive -> press)
        self.declare_parameter('l2_axis', 2)  # left trigger (analog)
        self.declare_parameter('invert_r2', False)
        self.declare_parameter('invert_l2', False)

        # D-pad (buttons) for fixed angular and linear
        self.declare_parameter('dpad_left_btn', 6)
        self.declare_parameter('dpad_right_btn', 7)
        self.declare_parameter('dpad_up_btn', 11)
        self.declare_parameter('dpad_down_btn', 12)

        # Buttons for fixed linear
        self.declare_parameter('r1_btn', 5)   # fixed positive linear
        self.declare_parameter('l1_btn', 4)   # fixed negative linear

        # Enable / disable buttons
        self.declare_parameter('enable_btn', 0)
        self.declare_parameter('disable_btn', 1)

        # --- Scaling / limits / deadzone ---
        self.declare_parameter('max_linear', 2.0)     # m/s
        self.declare_parameter('max_angular', 0.87)   # rad/s (~50 deg/s)
        self.declare_parameter('dpad_angular_min', 0.1)  # minimum angular speed when dpad pressed
        self.declare_parameter('dpad_linear_min', 0.1)   # minimum linear speed for dpad up/down
        self.declare_parameter('r1_linear_min', 0.1)  # fixed linear speed for R1 (m/s)
        self.declare_parameter('l1_linear_min', 0.1)  # fixed negative linear for L1 (m/s)
        self.declare_parameter('trigger_scale', 1.0)  # multiplier for trigger -> linear mapping
        self.declare_parameter('deadzone', 0.05)      # axis deadzone
        self.declare_parameter('publish_rate', 20.0)  # Hz

        # --- Read params ---
        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.enable_service_name = self.get_parameter('enable_service_name').get_parameter_value().string_value

        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.invert_angular = self.get_parameter('invert_angular_axis').get_parameter_value().bool_value

        self.r2_axis = self.get_parameter('r2_axis').get_parameter_value().integer_value
        self.l2_axis = self.get_parameter('l2_axis').get_parameter_value().integer_value
        self.invert_r2 = self.get_parameter('invert_r2').get_parameter_value().bool_value
        self.invert_l2 = self.get_parameter('invert_l2').get_parameter_value().bool_value

        self.dpad_left_btn = self.get_parameter('dpad_left_btn').get_parameter_value().integer_value
        self.dpad_right_btn = self.get_parameter('dpad_right_btn').get_parameter_value().integer_value
        self.dpad_up_btn = self.get_parameter('dpad_up_btn').get_parameter_value().integer_value
        self.dpad_down_btn = self.get_parameter('dpad_down_btn').get_parameter_value().integer_value

        self.r1_btn = self.get_parameter('r1_btn').get_parameter_value().integer_value
        self.l1_btn = self.get_parameter('l1_btn').get_parameter_value().integer_value

        self.enable_btn = self.get_parameter('enable_btn').get_parameter_value().integer_value
        self.disable_btn = self.get_parameter('disable_btn').get_parameter_value().integer_value

        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.dpad_angular_min = self.get_parameter('dpad_angular_min').get_parameter_value().double_value
        self.dpad_linear_min = self.get_parameter('dpad_linear_min').get_parameter_value().double_value
        self.r1_linear_min = self.get_parameter('r1_linear_min').get_parameter_value().double_value
        self.l1_linear_min = self.get_parameter('l1_linear_min').get_parameter_value().double_value
        self.trigger_scale = self.get_parameter('trigger_scale').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        if publish_rate <= 0.0:
            publish_rate = 20.0
        self.publish_period = 1.0 / publish_rate

        # --- Internal state ---
        self.last_joy = None
        self.prev_buttons: List[int] = []

        # --- ROS interfaces ---
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, qos)

        # service client for enabling / disabling: use exact topic /set_chassis_enable by default
        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service_name)

        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info(
            f"drive_segway_joy started, publishing '{self.cmd_vel_topic}' @ {publish_rate:.1f} Hz")
        self.get_logger().info(
            f"angular_axis={self.angular_axis}, r2_axis={self.r2_axis}, l2_axis={self.l2_axis}, "
            f"dpad_up={self.dpad_up_btn}, dpad_down={self.dpad_down_btn}")
        self.get_logger().info(
            f"dpad_left={self.dpad_left_btn}, dpad_right={self.dpad_right_btn}, r1={self.r1_btn}, l1={self.l1_btn}")
        self.get_logger().info(
            f"enable_btn={self.enable_btn}, disable_btn={self.disable_btn}, service={self.enable_service_name}")

    # --- helpers ---
    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
        elif len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = list(msg.buttons)

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) <= self.deadzone else v

    def axis_to_unit_positive(self, raw: float, invert: bool) -> float:
        """
        Convert trigger axis raw input to 0..1 scale (positive when pressed).
        Handles two common schemes:
         - If raw in [-1,1] where -1 is released and +1 is fully pressed -> convert: (raw+1)/2
         - If raw already in [0,1] -> use as-is.
        Use 'invert' to flip sign if needed.
        """
        val = float(raw)
        if -1.0 <= val <= 1.0:
            scaled = (val + 1.0) / 2.0
        else:
            scaled = max(0.0, float(val))
        if invert:
            scaled = 1.0 - scaled
        return max(0.0, min(1.0, scaled))

    def get_axis_val(self, axes: List[float], idx: int, invert: bool) -> float:
        if idx < 0 or idx >= len(axes):
            return 0.0
        v = float(axes[idx])
        return -v if invert else v

    def is_pressed(self, buttons: List[int], idx: int) -> bool:
        return 0 <= idx < len(buttons) and buttons[idx] != 0

    def is_rising(self, buttons: List[int], idx: int) -> bool:
        cur = 0 <= idx < len(buttons) and buttons[idx] != 0
        prev = 0 <= idx < len(self.prev_buttons) and self.prev_buttons[idx] != 0
        return cur and not prev

    # --- main loop ---
    def timer_callback(self):
        twist = Twist()
        if self.last_joy is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # --- Angular command (axis takes precedence) ---
        raw_ang = self.get_axis_val(axes, self.angular_axis, self.invert_angular)
        raw_ang = self.apply_deadzone(raw_ang)

        if abs(raw_ang) > 0.0:
            # axis -> proportional angular [-max_angular, max_angular]
            angular_cmd = raw_ang * self.max_angular
        else:
            # axis centered -> check dpad left/right
            if self.is_pressed(buttons, self.dpad_left_btn):
                angular_cmd = -abs(self.dpad_angular_min)
            elif self.is_pressed(buttons, self.dpad_right_btn):
                angular_cmd = abs(self.dpad_angular_min)
            else:
                angular_cmd = 0.0

        # --- Linear command: consider D-PAD up/down, R1, R2 (positive) and L1, L2 (negative) ---
        candidates = []

        # D-pad up/down fixed linear
        if self.is_pressed(buttons, self.dpad_up_btn):
            candidates.append(abs(self.dpad_linear_min))
        if self.is_pressed(buttons, self.dpad_down_btn):
            candidates.append(-abs(self.dpad_linear_min))

        # R1 fixed positive
        if self.is_pressed(buttons, self.r1_btn):
            candidates.append(self.r1_linear_min)

        # R2 analog trigger positive
        if 0 <= self.r2_axis < len(axes):
            raw_r2 = axes[self.r2_axis]
            r2_val = self.axis_to_unit_positive(raw_r2, self.invert_r2)
            if r2_val > 0.0 + self.deadzone:
                candidates.append(r2_val * self.trigger_scale * self.max_linear)

        # L1 fixed negative
        if self.is_pressed(buttons, self.l1_btn):
            candidates.append(-abs(self.l1_linear_min))

        # L2 analog trigger negative
        if 0 <= self.l2_axis < len(axes):
            raw_l2 = axes[self.l2_axis]
            l2_val = self.axis_to_unit_positive(raw_l2, self.invert_l2)
            if l2_val > 0.0 + self.deadzone:
                candidates.append(-l2_val * self.trigger_scale * self.max_linear)

        # Choose linear command:
        linear_cmd = 0.0
        if candidates:
            # pick candidate with largest absolute magnitude
            linear_cmd = max(candidates, key=lambda x: abs(x))

        # clamp safety
        linear_cmd = max(-self.max_linear, min(self.max_linear, linear_cmd))
        angular_cmd = max(-self.max_angular, min(self.max_angular, angular_cmd))

        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        self.cmd_vel_pub.publish(twist)

        # enable/disable on rising edge
        if self.is_rising(buttons, self.enable_btn):
            self.call_enable_service(True)
        if self.is_rising(buttons, self.disable_btn):
            self.call_enable_service(False)

        # store prev buttons
        self.prev_buttons = list(buttons)

    def call_enable_service(self, enable: bool):
        # wait briefly for the service to exist
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Enable service '{self.enable_service_name}' not available")
            return
        req = RosSetChassisEnableCmd.Request()
        # use the exact field name you provided:
        req.ros_set_chassis_enable_cmd = bool(enable)
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
