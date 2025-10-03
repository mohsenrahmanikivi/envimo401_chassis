#!/usr/bin/env python3
"""
drive_segway_joy.py

ROS2 Python node to control Segway RMP401 via joystick.
- Linear: R2 (+), L2 (-) proportional [±2.0 m/s]
- Angular: left stick left/right proportional [±0.87 rad/s]
- D-pad left/right: fixed ±0.1 rad/s
- D-pad up/down: fixed ±0.1 m/s
- R1: fixed +0.1 m/s
- L1: fixed -0.1 m/s
- Enable: calls /set_chassis_enable with True
- Disable: calls /set_chassis_enable with False

All fixed velocities = 0.1 by default.
Includes velocity ramping to prevent jumps.
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

        # Parameters
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service_name', '/set_chassis_enable')

        # Axis mapping (adjust for your joystick)
        self.declare_parameter('angular_axis', 0)   # left stick horizontal
        self.declare_parameter('r2_axis', 5)        # right trigger
        self.declare_parameter('l2_axis', 2)        # left trigger

        # Button mapping (adjust for your joystick)
        self.declare_parameter('dpad_left_btn', 6)
        self.declare_parameter('dpad_right_btn', 7)
        self.declare_parameter('dpad_up_btn', 11)
        self.declare_parameter('dpad_down_btn', 12)
        self.declare_parameter('r1_btn', 5)
        self.declare_parameter('l1_btn', 4)
        self.declare_parameter('enable_btn', 0)
        self.declare_parameter('disable_btn', 1)

        # Limits
        self.max_linear = 2.0       # m/s
        self.max_angular = 0.87     # rad/s (~50 deg/s)
        self.min_linear = 0.1
        self.min_angular = 0.1
        self.deadzone = 0.05
        self.publish_rate = 20.0    # Hz

        # Internal state
        self.last_joy = None
        self.prev_buttons: List[int] = []
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # ROS setup
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.joy_sub = self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_callback, qos)
        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.get_parameter('enable_service_name').value)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info("drive_segway_joy started.")

    # --- Helper functions ---
    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)

    def is_pressed(self, buttons, idx):
        return 0 <= idx < len(buttons) and buttons[idx] != 0

    def is_rising(self, buttons, idx):
        cur = self.is_pressed(buttons, idx)
        prev = 0 <= idx < len(self.prev_buttons) and self.prev_buttons[idx] != 0
        return cur and not prev

    def axis_value(self, axes, idx):
        if 0 <= idx < len(axes):
            return axes[idx]
        return 0.0

    def trigger_to_unit(self, raw):
        """Convert trigger raw value to 0..1 (works for [-1,1] or [0,1])."""
        val = float(raw)
        if -1.0 <= val <= 1.0:
            return (val + 1.0) / 2.0
        return max(0.0, min(1.0, val))

    def ramp(self, prev, target, max_accel):
        dt = 1.0 / self.publish_rate
        delta = target - prev
        step = max_accel * dt
        if abs(delta) > step:
            delta = step if delta > 0 else -step
        return prev + delta

    # --- Main loop ---
    def timer_callback(self):
        twist = Twist()
        if self.last_joy is None:
            self.cmd_vel_pub.publish(twist)
            return

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # Angular control
        angular = self.axis_value(axes, self.get_parameter('angular_axis').value)
        if abs(angular) > self.deadzone:
            angular_cmd = angular * self.max_angular
            # enforce min angular step
            if 0 < abs(angular_cmd) < self.min_angular:
                angular_cmd = self.min_angular if angular_cmd > 0 else -self.min_angular
        elif self.is_pressed(buttons, self.get_parameter('dpad_left_btn').value):
            angular_cmd = -self.min_angular
        elif self.is_pressed(buttons, self.get_parameter('dpad_right_btn').value):
            angular_cmd = self.min_angular
        else:
            angular_cmd = 0.0

        # Linear control
        linear_cmd = 0.0
        candidates = []

        # D-pad up/down fixed
        if self.is_pressed(buttons, self.get_parameter('dpad_up_btn').value):
            candidates.append(self.min_linear)
        if self.is_pressed(buttons, self.get_parameter('dpad_down_btn').value):
            candidates.append(-self.min_linear)

        # R1/L1 fixed
        if self.is_pressed(buttons, self.get_parameter('r1_btn').value):
            candidates.append(self.min_linear)
        if self.is_pressed(buttons, self.get_parameter('l1_btn').value):
            candidates.append(-self.min_linear)

        # R2/L2 proportional
        r2_val = self.trigger_to_unit(self.axis_value(axes, self.get_parameter('r2_axis').value))
        l2_val = self.trigger_to_unit(self.axis_value(axes, self.get_parameter('l2_axis').value))
        if r2_val > self.deadzone:
            candidates.append(max(self.min_linear, r2_val * self.max_linear))
        if l2_val > self.deadzone:
            candidates.append(-max(self.min_linear, l2_val * self.max_linear))

        if candidates:
            linear_cmd = max(candidates, key=lambda x: abs(x))

        # --- Apply smoothing (accel limits) ---
        linear_cmd = self.ramp(self.prev_linear, linear_cmd, max_accel=0.5)   # m/s²
        angular_cmd = self.ramp(self.prev_angular, angular_cmd, max_accel=0.5) # rad/s²

        twist.linear.x = linear_cmd
        twist.angular.z = angular_cmd
        self.cmd_vel_pub.publish(twist)

        # Save state
        self.prev_linear = linear_cmd
        self.prev_angular = angular_cmd
        self.prev_buttons = list(buttons)

        # Handle enable/disable service
        if self.is_rising(buttons, self.get_parameter('enable_btn').value):
            self.call_enable_service(True)
        if self.is_rising(buttons, self.get_parameter('disable_btn').value):
            self.call_enable_service(False)

    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Enable service not available")
            return
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = enable
        self.enable_client.call_async(req)
        self.get_logger().info(f"Sent {'ENABLE' if enable else 'DISABLE'} to /set_chassis_enable")


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
