#!/usr/bin/env python3
"""
drive_segway_joy.py - robust D-pad & bumper handling + manual R2/L2

Behavior implemented:
 - Axis 0 (Left stick horizontal): angular velocity [-max..+max], deadzone applied
 - Axis 1 unused for linear (triggers control linear)
 - D-pad left/right: fixed angular ±0.1 (handles both button-style and hat-axis-style D-pad)
 - D-pad up/down: fixed linear ±0.1
 - R1 (button 5 or fallback 6): fixed linear +0.1
 - L1 (button 4 or fallback 7): fixed linear -0.1
 - R2 (manual axis): proportional forward linear, idle=1 -> pressed=-1 -> [0..max_lin]
 - L2 (manual axis): proportional backward linear, idle=1 -> pressed=-1 -> [0..max_lin]
 - Enable (button 0) / Disable (button 1): service calls
 - QoS: cmd_vel publisher uses RELIABLE/TRANSIENT_LOCAL by default (param)
 - Lots of debug logs to verify mappings and values
"""
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd
import time

def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) <= deadzone else value

class DriveSegwayJoy(Node):
    def __init__(self):
        super().__init__('drive_segway_joy')

        # --- parameters ---
        self.declare_parameter('max_lin', 2.0)
        self.declare_parameter('min_turn_radius', 1.36)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service', 'set_chassis_enable')

        # triggers (you said axes 4 & 5)
        self.declare_parameter('trigger_mode', 'manual')
        self.declare_parameter('l2_axis', 4)   # default from your device
        self.declare_parameter('r2_axis', 5)

        # D-pad/hat options (if your D-pad is an axis, set hat_x/y, otherwise defaults to -1)
        self.declare_parameter('dpad_hat_x', -1)
        self.declare_parameter('dpad_hat_y', -1)

        # QoS & debug
        self.declare_parameter('cmd_vel_qos_transient_local', True)
        self.declare_parameter('debug_joy', False)

        # --- read params ---
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.min_turn_radius = float(self.get_parameter('min_turn_radius').value) or 1.0
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.enable_service = str(self.get_parameter('enable_service').value)

        self.trigger_mode = str(self.get_parameter('trigger_mode').value)
        self.l2_axis = int(self.get_parameter('l2_axis').value)
        self.r2_axis = int(self.get_parameter('r2_axis').value)
        self.dpad_hat_x = int(self.get_parameter('dpad_hat_x').value)
        self.dpad_hat_y = int(self.get_parameter('dpad_hat_y').value)

        self.use_transient_local = bool(self.get_parameter('cmd_vel_qos_transient_local').value)
        self.debug_joy = bool(self.get_parameter('debug_joy').value)

        # derived
        self.max_ang = self.max_lin / self.min_turn_radius

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL if self.use_transient_local else QoSDurabilityPolicy.VOLATILE

        # pub/sub/service
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service)
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.enable_service} not available yet; will retry on call')

        # state
        self.prev_twist = Twist()
        self.last_buttons: List[int] = []
        self._last_axes: Optional[List[float]] = None
        self._last_debug_time = time.time()
        self.get_logger().info(f'drive_segway_joy ready (manual triggers l2={self.l2_axis}, r2={self.r2_axis})')

    # PS trigger mapping: axis in [+1..-1] -> fraction [0..1]
    def _ps_trigger_fraction(self, axis_value: float) -> float:
        v = float(axis_value)
        frac = (1.0 - v) / 2.0
        # clamp
        if frac <= 1e-6:
            return 0.0
        if frac >= 1.0:
            return 1.0
        return frac

    def _dpad_from_buttons(self, buttons: List[int]) -> Tuple[int,int]:
        """
        Return (up/down, left/right) as integers in {-1,0,1}.
        Buttons mapping (common): up=12, down=13, left=14, right=15
        """
        up = down = left = right = 0
        if len(buttons) > 12:
            up = 1 if buttons[12] == 1 else 0
            down = 1 if buttons[13] == 1 else 0
            left = 1 if buttons[14] == 1 else 0
            right = 1 if buttons[15] == 1 else 0
            y = up - down
            x = right - left
            return (y, x)
        return (0, 0)

    def _dpad_from_hat_axes(self, axes: List[float]) -> Tuple[int,int]:
        """
        If your controller presents D-pad as hat axes, those axes typically return:
        -1, 0, +1 for left/center/right or up/center/down.
        Parameters dpad_hat_x/dpad_hat_y are set to axis indices (or -1).
        Returns (y, x) ints in {-1,0,1}.
        """
        x = 0; y = 0
        if 0 <= self.dpad_hat_x < len(axes):
            try:
                ax = float(axes[self.dpad_hat_x])
                if ax > 0.5: x = 1
                elif ax < -0.5: x = -1
            except Exception:
                x = 0
        if 0 <= self.dpad_hat_y < len(axes):
            try:
                ay = float(axes[self.dpad_hat_y])
                # note: some drivers use -1 for up; we treat signs symmetrically
                if ay > 0.5: y = 1
                elif ay < -0.5: y = -1
            except Exception:
                y = 0
        return (y, x)

    def _bumpers_from_buttons(self, buttons: List[int]) -> Tuple[bool,bool]:
        """
        Return (r1_pressed, l1_pressed).
        We check common mappings: R1=5, L1=4, and fallback pair (6,7).
        """
        r1 = False; l1 = False
        if len(buttons) > 5:
            r1 = buttons[5] == 1
            l1 = buttons[4] == 1
        # fallback try (6,7)
        if not (r1 or l1):
            if len(buttons) > 7:
                r1 = buttons[7] == 1 or r1
                l1 = buttons[6] == 1 or l1
        return (r1, l1)

    def joy_callback(self, msg: Joy):
        axes = list(msg.axes) if msg.axes is not None else []
        buttons = list(msg.buttons) if msg.buttons is not None else []

        # debug raw frames when they change
        if self.debug_joy:
            if self._last_axes is None or axes != self._last_axes:
                self.get_logger().info(f'JOY received axes(len={len(axes)}): {axes}')
                self.get_logger().info(f'JOY received buttons(len={len(buttons)}): {buttons}')
                self._last_axes = list(axes)

        # ensure last_buttons initialised
        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)

        desired = Twist()

        # --- ANGULAR from left stick horizontal (axis 0) ---
        if len(axes) > 0:
            raw_ang = apply_deadzone(axes[0], self.deadzone)
            desired.angular.z = float(raw_ang) * self.max_ang

        # --- DPAD: try hat axes first (if configured), else buttons ---
        dpad_y, dpad_x = 0, 0
        if 0 <= self.dpad_hat_x < len(axes) or 0 <= self.dpad_hat_y < len(axes):
            dpad_y, dpad_x = self._dpad_from_hat_axes(axes)
            if self.debug_joy:
                self.get_logger().info(f'Using D-PAD from hat axes: y={dpad_y}, x={dpad_x}')
        else:
            dpad_y, dpad_x = self._dpad_from_buttons(buttons)
            if self.debug_joy:
                self.get_logger().info(f'Using D-PAD from buttons: y={dpad_y}, x={dpad_x}')

        # D-pad left/right -> angular increments
        if dpad_x == 1:
            desired.angular.z += 0.1
        elif dpad_x == -1:
            desired.angular.z -= 0.1

        # D-pad up/down -> fixed linear increments
        if dpad_y == 1:
            desired.linear.x += 0.1
        elif dpad_y == -1:
            desired.linear.x -= 0.1

        # --- BUMPERS R1/L1 fixed increments ---
        r1, l1 = self._bumpers_from_buttons(buttons)
        if self.debug_joy:
            self.get_logger().info(f'BUMPERS read: R1={r1}, L1={l1}')

        if r1:
            desired.linear.x += 0.1
        if l1:
            desired.linear.x -= 0.1

        # --- TRIGGERS (manual axes) ---
        fwd = 0.0; bwd = 0.0
        if self.trigger_mode == 'manual':
            if 0 <= self.r2_axis < len(axes):
                r_val = axes[self.r2_axis]
                r_frac = self._ps_trigger_fraction(r_val)
                fwd = r_frac * self.max_lin
            else:
                r_frac = 0.0
            if 0 <= self.l2_axis < len(axes):
                l_val = axes[self.l2_axis]
                l_frac = self._ps_trigger_fraction(l_val)
                bwd = l_frac * self.max_lin
            else:
                l_frac = 0.0

            # debug triggers
            if self.debug_joy and (time.time() - self._last_debug_time) > 0.2:
                self.get_logger().info(f'TRIGGERS read: r_idx={self.r2_axis} r_val={r_val if 0<=self.r2_axis<len(axes) else "N/A"} r_frac={r_frac:.3f} -> fwd={fwd:.3f}; '
                                       f'l_idx={self.l2_axis} l_val={l_val if 0<=self.l2_axis<len(axes) else "N/A"} l_frac={l_frac:.3f} -> bwd={bwd:.3f}')
                self._last_debug_time = time.time()
        else:
            # simple safe default: no triggers if not manual
            pass

        # Triggers add to linear (forward positive, backward negative)
        desired.linear.x += (fwd - bwd)

        # clamp linear and angular before smoothing
        desired.linear.x = max(min(desired.linear.x, self.max_lin), -self.max_lin)
        desired.angular.z = max(min(desired.angular.z, self.max_ang), -self.max_ang)

        # smoothing (EMA)
        a = float(self.smoothing_alpha)
        smoothed = Twist()
        smoothed.linear.x = a * desired.linear.x + (1.0 - a) * float(self.prev_twist.linear.x)
        smoothed.angular.z = a * desired.angular.z + (1.0 - a) * float(self.prev_twist.angular.z)
        self.prev_twist = smoothed

        # publish and debug-publish
        self.cmd_pub.publish(smoothed)
        if self.debug_joy:
            self.get_logger().info(f'PUBLISH cmd_vel linear={smoothed.linear.x:.3f} ang={smoothed.angular.z:.3f}')

        # enable/disable rising-edge detection
        if len(buttons) != len(self.last_buttons):
            self.last_buttons = [0] * len(buttons)
        if len(buttons) > 0 and buttons[0] == 1 and self.last_buttons[0] == 0:
            self.get_logger().info('Enable pressed -> calling set_chassis_enable(True)')
            self.call_enable_service(True)
        if len(buttons) > 1 and buttons[1] == 1 and self.last_buttons[1] == 0:
            self.get_logger().info('Disable pressed -> calling set_chassis_enable(False)')
            self.call_enable_service(False)
        self.last_buttons = list(buttons)

    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Enable service not available; cannot set')
            return
        req = RosSetChassisEnableCmd.Request()
        try:
            req.ros_set_chassis_enable_cmd = bool(enable)
        except AttributeError:
            self.get_logger().error('RosSetChassisEnableCmd.Request has no field "ros_set_chassis_enable_cmd"')
            return
        fut = self.enable_client.call_async(req)
        fut.add_done_callback(lambda f: self.get_logger().info(f'set_chassis_enable({enable}) called'))

def main(args=None):
    rclpy.init(args=args)
    node = DriveSegwayJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
