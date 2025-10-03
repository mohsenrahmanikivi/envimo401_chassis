#!/usr/bin/env python3
"""
drive_segway_joy.py

ROS2 node: Use a joystick (PS-style) to teleop the Segway RMP401.

Behavior (all fixed minimums = 0.1):
 - Left stick horizontal → angular velocity [-max..+max], min = 0.1 if moved
 - D-pad left/right → fixed angular ±0.1
 - D-pad up/down → fixed linear ±0.1
 - R1 → fixed linear +0.1
 - L1 → fixed linear -0.1
 - R2 trigger → proportional forward linear, idle=1.0 → pressed=-1.0
 - L2 trigger → proportional backward linear, idle=1.0 → pressed=-1.0
 - Enable button → call /set_chassis_enable true
 - Disable button → call /set_chassis_enable false
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

        # --- Topics and service ---
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service_name', '/set_chassis_enable')

        # --- Axis / Button mapping (adjust as needed) ---
        self.declare_parameter('angular_axis', 0)   # left stick horizontal
        self.declare_parameter('r2_axis', 5)        # right trigger
        self.declare_parameter('l2_axis', 2)        # left trigger

        self.declare_parameter('dpad_left_btn', 6)
        self.declare_parameter('dpad_right_btn', 7)
        self.declare_parameter('dpad_up_btn', 11)
        self.declare_parameter('dpad_down_btn', 12)

        self.declare_parameter('r1_btn', 5)
        self.declare_parameter('l1_btn', 4)

        self.declare_parameter('enable_btn', 0)
        self.declare_parameter('disable_btn', 1)

        # --- Limits ---
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('max_angular', 0.87)      # ~50 deg/s
        self.declare_parameter('fixed_min_linear', 0.1)
        self.declare_parameter('fixed_min_angular', 0.1)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('debug', False)

        # --- Read params ---
        self.joy_topic = self.get_parameter('joy_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enable_service_name = self.get_parameter('enable_service_name').value

        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.r2_axis = int(self.get_parameter('r2_axis').value)
        self.l2_axis = int(self.get_parameter('l2_axis').value)

        self.dpad_left_btn = int(self.get_parameter('dpad_left_btn').value)
        self.dpad_right_btn = int(self.get_parameter('dpad_right_btn').value)
        self.dpad_up_btn = int(self.get_parameter('dpad_up_btn').value)
        self.dpad_down_btn = int(self.get_parameter('dpad_down_btn').value)

        self.r1_btn = int(self.get_parameter('r1_btn').value)
        self.l1_btn = int(self.get_parameter('l1_btn').value)

        self.enable_btn = int(self.get_parameter('enable_btn').value)
        self.disable_btn = int(self.get_parameter('disable_btn').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.fixed_min_linear = float(self.get_parameter('fixed_min_linear').value)
        self.fixed_min_angular = float(self.get_parameter('fixed_min_angular').value)
        self.deadzone = float(self.get_parameter('deadzone').value)

        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.debug = bool(self.get_parameter('debug').value)

        # --- State ---
        self.last_joy: Joy = None
        self.prev_buttons: List[int] = []
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, qos)

        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service_name)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("drive_segway_joy started with trigger mapping (idle=1, pressed=-1)")

    # ---------------- Helpers ----------------
    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        if not self.prev_buttons or len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = list(msg.buttons)

    def is_pressed(self, buttons: List[int], idx: int) -> bool:
        # Only treat as pressed if value > 0 (ignore -1)
        return 0 <= idx < len(buttons) and buttons[idx] > 0

    def is_rising(self, buttons: List[int], idx: int) -> bool:
        cur = self.is_pressed(buttons, idx)
        prev = 0 <= idx < len(self.prev_buttons) and self.prev_buttons[idx] > 0
        return cur and not prev

    def axis_safe(self, axes: List[float], idx: int) -> float:
        return float(axes[idx]) if 0 <= idx < len(axes) else 0.0

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) <= self.deadzone else v

    def ramp_towards(self, prev: float, target: float, max_accel: float, dt: float) -> float:
        delta = target - prev
        step = max_accel * dt
        if abs(delta) <= step:
            return target
        return prev + (step if delta > 0 else -step)

    def trigger_to_unit(self, raw: float) -> float:
        """
        Convert trigger raw where idle=1.0, pressed=-1.0 into [0..1].
        """
        return (1.0 - raw) / 2.0

    # ---------------- Main loop ----------------
    def timer_callback(self):
        twist = Twist()
        if self.last_joy is None:
            self.cmd_vel_pub.publish(twist)
            return

        axes = self.last_joy.axes
        buttons = self.last_joy.buttons

        # Angular
        raw_ang = self.apply_deadzone(self.axis_safe(axes, self.angular_axis))
        if raw_ang != 0.0:
            target_angular = raw_ang * self.max_angular
            if 0 < abs(target_angular) < self.fixed_min_angular:
                target_angular = self.fixed_min_angular if target_angular > 0 else -self.fixed_min_angular
        elif self.is_pressed(buttons, self.dpad_left_btn):
            target_angular = -self.fixed_min_angular
        elif self.is_pressed(buttons, self.dpad_right_btn):
            target_angular = self.fixed_min_angular
        else:
            target_angular = 0.0

        # Linear candidates
        candidates = []
        if self.is_pressed(buttons, self.dpad_up_btn):
            candidates.append(self.fixed_min_linear)
        if self.is_pressed(buttons, self.dpad_down_btn):
            candidates.append(-self.fixed_min_linear)
        if self.is_pressed(buttons, self.r1_btn):
            candidates.append(self.fixed_min_linear)
        if self.is_pressed(buttons, self.l1_btn):
            candidates.append(-self.fixed_min_linear)

        r2_val = self.trigger_to_unit(self.axis_safe(axes, self.r2_axis))
        if r2_val > 0.0:
            candidates.append(max(self.fixed_min_linear, r2_val * self.max_linear))
        l2_val = self.trigger_to_unit(self.axis_safe(axes, self.l2_axis))
        if l2_val > 0.0:
            candidates.append(-max(self.fixed_min_linear, l2_val * self.max_linear))

        target_linear = max(candidates, key=lambda x: abs(x)) if candidates else 0.0

        # Smooth ramp
        dt = 1.0 / self.publish_rate
        linear_smoothed = self.ramp_towards(self.prev_linear, target_linear, self.max_linear_accel, dt)
        angular_smoothed = self.ramp_towards(self.prev_angular, target_angular, self.max_angular_accel, dt)

        twist.linear.x = float(linear_smoothed)
        twist.angular.z = float(angular_smoothed)
        self.cmd_vel_pub.publish(twist)

        self.prev_linear = linear_smoothed
        self.prev_angular = angular_smoothed
        self.prev_buttons = list(buttons)

        # Enable/Disable
        if self.is_rising(buttons, self.enable_btn):
            self.call_enable_service(True)
        if self.is_rising(buttons, self.disable_btn):
            self.call_enable_service(False)

    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Service {self.enable_service_name} not available")
            return
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = bool(enable)
        self.enable_client.call_async(req)
        self.get_logger().info(f"Sent {'ENABLE' if enable else 'DISABLE'}")


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
