#!/usr/bin/env python3
"""
drive_segway_joy_with_grace_and_ramp.py

Publishes cmd_vel at a fixed rate. If Joy stops:
 - hold the last valid command for `miss_grace_count` ticks,
 - then ramp last_valid -> zero over `ramp_down_time` seconds,
 - continue publishing zeros at the same fixed rate afterwards.

Features:
 - In-place rotation mode controlled by /enable_chassis_rotate (segway_msgs/srv/RosEnableChassisRotateCmd).
 - Button indices: default enable=2, disable=3 (configurable via ROS params).
 - When rotate mode is enabled: linear.x is forced to 0 (and smoothing state zeroed).
 - When disabling rotate mode (via button or robot-disable), an immediate zero angular Twist
   is published and smoothing state cleared before calling the rotate-disable service.
 - Robust async service handling with done-callbacks and fallbacks.
"""

from typing import List, Tuple, Optional
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# service types (must be available in your workspace)
from segway_msgs.srv import RosSetChassisEnableCmd, RosEnableChassisRotateCmd


def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) <= deadzone else value


class DriveSegwayJoy(Node):
    def __init__(self):
        super().__init__('drive_segway_joy')

        # ----- Parameters (declare defaults) -----
        self.declare_parameter('max_lin', 2.0)
        self.declare_parameter('min_turn_radius', 1.36)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('enable_service', 'set_chassis_enable')

        # rotate-specific params
        self.declare_parameter('rotate_service', '/enable_chassis_rotate')
        self.declare_parameter('rotate_enable_button', 2)
        self.declare_parameter('rotate_disable_button', 3)

        self.declare_parameter('trigger_mode', 'manual')
        self.declare_parameter('l2_axis', 4)
        self.declare_parameter('r2_axis', 5)

        self.declare_parameter('dpad_hat_x', -1)
        self.declare_parameter('dpad_hat_y', -1)

        self.declare_parameter('cmd_vel_qos_transient_local', True)
        self.declare_parameter('debug_joy', False)

        self.declare_parameter('cmd_publish_hz', 50.0)
        self.declare_parameter('watchdog_timeout', 0.04)
        self.declare_parameter('miss_grace_count', 2)
        self.declare_parameter('ramp_down_time', 0.25)

        # ----- Read params -----
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.min_turn_radius = float(self.get_parameter('min_turn_radius').value) or 1.0
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.enable_service = str(self.get_parameter('enable_service').value)

        self.rotate_service = str(self.get_parameter('rotate_service').value)
        self.rotate_enable_button = int(self.get_parameter('rotate_enable_button').value)
        self.rotate_disable_button = int(self.get_parameter('rotate_disable_button').value)

        self.trigger_mode = str(self.get_parameter('trigger_mode').value)
        self.l2_axis = int(self.get_parameter('l2_axis').value)
        self.r2_axis = int(self.get_parameter('r2_axis').value)

        self.dpad_hat_x = int(self.get_parameter('dpad_hat_x').value)
        self.dpad_hat_y = int(self.get_parameter('dpad_hat_y').value)

        self.use_transient_local = bool(self.get_parameter('cmd_vel_qos_transient_local').value)
        self.debug_joy = bool(self.get_parameter('debug_joy').value)

        self.cmd_publish_hz = float(self.get_parameter('cmd_publish_hz').value)
        if self.cmd_publish_hz <= 0.0:
            self.get_logger().warn('cmd_publish_hz must be > 0; defaulting to 50 Hz')
            self.cmd_publish_hz = 50.0

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.miss_grace_count = int(self.get_parameter('miss_grace_count').value)
        self.ramp_down_time = float(self.get_parameter('ramp_down_time').value)

        # derived
        self.max_ang = self.max_lin / self.min_turn_radius

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL if self.use_transient_local else QoSDurabilityPolicy.VOLATILE

        # ----- Pub/Sub/Clients -----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)

        self.enable_client = self.create_client(RosSetChassisEnableCmd, self.enable_service)
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.enable_service} not available yet; will retry on call')

        self.rotate_client = self.create_client(RosEnableChassisRotateCmd, self.rotate_service)
        if not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{self.rotate_service} not available yet; will retry on call')

        # Try to ensure rotate disabled at startup (best-effort)
        try:
            self.call_rotate_service(False)
        except Exception:
            pass

        # ----- Internal state -----
        self.prev_twist = Twist()            # smoothed twist
        self._last_valid_twist = Twist()     # used for hold/ramp
        self._last_joy_time: Optional[float] = None
        self.last_buttons: List[int] = []
        self._last_axes: Optional[List[float]] = None
        self._last_debug_time = time.time()

        self._rotate_mode_enabled = False    # local rotate-only flag

        self._consecutive_misses = 0
        self._ramp_start_time: Optional[float] = None

        # Timer for fixed-rate publishing
        timer_period = 1.0 / float(self.cmd_publish_hz)
        self._publish_timer = self.create_timer(timer_period, self._publish_timer_cb)

        self.get_logger().info(
            f'drive_segway_joy ready: publish {self.cmd_publish_hz}Hz, watchdog={self.watchdog_timeout}s, '
            f'miss_grace={self.miss_grace_count}, ramp_down={self.ramp_down_time}s, rotate_service={self.rotate_service}'
        )

    # ---------------- helper functions ----------------
    def _ps_trigger_fraction(self, axis_value: float) -> float:
        v = float(axis_value)
        frac = (1.0 - v) / 2.0
        if frac <= 1e-6:
            return 0.0
        if frac >= 1.0:
            return 1.0
        return frac

    def _dpad_from_buttons(self, buttons: List[int]) -> Tuple[int, int]:
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

    def _dpad_from_hat_axes(self, axes: List[float]) -> Tuple[int, int]:
        x = 0; y = 0
        if 0 <= self.dpad_hat_x < len(axes):
            try:
                ax = float(axes[self.dpad_hat_x])
                if ax > 0.5:
                    x = 1
                elif ax < -0.5:
                    x = -1
            except Exception:
                x = 0
        if 0 <= self.dpad_hat_y < len(axes):
            try:
                ay = float(axes[self.dpad_hat_y])
                if ay > 0.5:
                    y = 1
                elif ay < -0.5:
                    y = -1
            except Exception:
                y = 0
        return (y, x)

    def _bumpers_from_buttons(self, buttons: List[int]) -> Tuple[bool, bool]:
        r1 = False; l1 = False
        if len(buttons) > 5:
            r1 = buttons[5] == 1
            l1 = buttons[4] == 1
        if not (r1 or l1):
            if len(buttons) > 7:
                r1 = buttons[7] == 1 or r1
                l1 = buttons[6] == 1 or l1
        return (r1, l1)

    def _rotate_call_done(self, future, requested_enable: bool):
        """
        Called when rotate service future completes. If the call failed we revert optimistic state.
        """
        try:
            res = future.result()  # may raise if service threw
            self.get_logger().info(f'Rotate service response for enable={requested_enable}: {res}')
            # assume success if no exception; if your response contains an explicit success boolean
            # you can inspect it here and revert if it's False.
        except Exception as e:
            self.get_logger().error(f'Rotate service call failed for enable={requested_enable}: {e}')
            if requested_enable and self._rotate_mode_enabled:
                self.get_logger().warn('Reverting local rotate flag to False due to failed rotate-enable service call.')
                self._rotate_mode_enabled = False

    def _publish_instant_zero_angular(self):
        """
        Immediately publish a Twist with angular.z = 0 (and linear.x = 0 if in rotate mode).
        Also clears smoothed/state angular and linear as needed to avoid EMA reintroduction.
        """
        try:
            # Clear smoothed angular so EMA won't reintroduce rotation
            self.prev_twist.angular.z = 0.0
            self._last_valid_twist.angular.z = 0.0

            zero_twist = Twist()
            zero_twist.angular.z = 0.0

            # If rotate mode was on, ensure linear is zeroed as well
            if self._rotate_mode_enabled:
                zero_twist.linear.x = 0.0
                self.prev_twist.linear.x = 0.0
                self._last_valid_twist.linear.x = 0.0

            self.cmd_pub.publish(zero_twist)
            self.get_logger().info('Published immediate zero angular before disabling rotate mode.')
        except Exception as e:
            self.get_logger().error(f'Failed to publish immediate zero angular: {e}')

    # ---------------- Joy callback ----------------
    def joy_callback(self, msg: Joy):
        """Compute desired & smoothed twist from incoming Joy and update last-joy timestamp."""
        now = time.time()
        self._last_joy_time = now

        axes = list(msg.axes) if msg.axes is not None else []
        buttons = list(msg.buttons) if msg.buttons is not None else []

        if self.debug_joy:
            if self._last_axes is None or axes != self._last_axes:
                self.get_logger().info(f'JOY received axes(len={len(axes)}): {axes}')
                self.get_logger().info(f'JOY received buttons(len={len(buttons)}): {buttons}')
                self._last_axes = list(axes)

        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)

        desired = Twist()

        # Angular from left stick horizontal (axis 0)
        if len(axes) > 0:
            raw_ang = apply_deadzone(axes[0], self.deadzone)
            desired.angular.z = -float(raw_ang) * self.max_ang

        # Right stick vertical (axis 3) -> linear (half range)
        if len(axes) > 3:
            raw_lin = apply_deadzone(-axes[3], self.deadzone)  # invert so up=forward
            desired.linear.x += raw_lin * (self.max_lin / 2.0)

        # D-PAD
        dpad_y, dpad_x = 0, 0
        if 0 <= self.dpad_hat_x < len(axes) or 0 <= self.dpad_hat_y < len(axes):
            dpad_y, dpad_x = self._dpad_from_hat_axes(axes)
            if self.debug_joy:
                self.get_logger().info(f'Using D-PAD from hat axes: y={dpad_y}, x={dpad_x}')
        else:
            dpad_y, dpad_x = self._dpad_from_buttons(buttons)
            if self.debug_joy:
                self.get_logger().info(f'Using D-PAD from buttons: y={dpad_y}, x={dpad_x}')

        if dpad_x == 1:
            desired.angular.z -= 1
        elif dpad_x == -1:
            desired.angular.z += 1

        if dpad_y == 1:
            desired.linear.x += 0.1
        elif dpad_y == -1:
            desired.linear.x -= 0.1

        # Bumpers
        r1, l1 = self._bumpers_from_buttons(buttons)
        if self.debug_joy:
            self.get_logger().info(f'BUMPERS read: R1={r1}, L1={l1}')
        if r1:
            desired.linear.x += 0.1
        if l1:
            desired.linear.x -= 0.1

        # Triggers (manual)
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

            if self.debug_joy and (time.time() - self._last_debug_time) > 0.2:
                self.get_logger().info(
                    f'TRIGGERS read: r_idx={self.r2_axis} r_val={r_val if 0<=self.r2_axis<len(axes) else "N/A"} r_frac={r_frac:.3f} -> fwd={fwd:.3f}; '
                    f'l_idx={self.l2_axis} l_val={l_val if 0<=self.l2_axis<len(axes) else "N/A"} l_frac={l_frac:.3f} -> bwd={bwd:.3f}'
                )
                self._last_debug_time = time.time()

        desired.linear.x += (fwd - bwd)

        # If rotate mode enabled, force linear to zero
        if self._rotate_mode_enabled:
            desired.linear.x = 0.0

        # Clamp
        desired.linear.x = max(min(desired.linear.x, self.max_lin), -self.max_lin)
        desired.angular.z = max(min(desired.angular.z, self.max_ang), -self.max_ang)

        # Smoothing (EMA)
        a = float(self.smoothing_alpha)
        smoothed = Twist()
        smoothed.linear.x = a * desired.linear.x + (1.0 - a) * float(self.prev_twist.linear.x)
        smoothed.angular.z = a * desired.angular.z + (1.0 - a) * float(self.prev_twist.angular.z)

        # Update state
        self.prev_twist = smoothed
        self._last_valid_twist = Twist()
        self._last_valid_twist.linear.x = float(self.prev_twist.linear.x)
        self._last_valid_twist.angular.z = float(self.prev_twist.angular.z)

        if self.debug_joy:
            self.get_logger().info(f'Computed smoothed twist linear={smoothed.linear.x:.3f} ang={smoothed.angular.z:.3f}')

        # --- Enable/Disable chassis handling ---
        if len(buttons) != len(self.last_buttons):
            # reinitialize last_buttons array if controller changed layout
            self.last_buttons = [0] * len(buttons)

        # Button 0 -> enable chassis
        if len(buttons) > 0 and buttons[0] == 1 and self.last_buttons[0] == 0:
            self.get_logger().info('Enable pressed -> calling set_chassis_enable(True)')
            self.call_enable_service(True)

        # Button 1 -> disable chassis (also disable rotate mode)
        if len(buttons) > 1 and buttons[1] == 1 and self.last_buttons[1] == 0:
            self.get_logger().info('Disable pressed -> calling set_chassis_enable(False)')
            self.call_enable_service(False)

            # publish immediate zero angular then call rotate disable service and clear local flag
            self.get_logger().info('Robot disable requested -> publishing zero angular and disabling rotate mode as well.')
            self._publish_instant_zero_angular()
            called = self.call_rotate_service(False)
            if self._rotate_mode_enabled:
                self._rotate_mode_enabled = False
            if not called:
                self.get_logger().warn('Rotate-service disable call could not be started; rotate mode cleared locally.')

        # --- Rotate-mode enable/disable buttons ---
        # Enable rotate (edge)
        if 0 <= self.rotate_enable_button < len(buttons) and \
           buttons[self.rotate_enable_button] == 1 and \
           self.last_buttons[self.rotate_enable_button] == 0:
            self.get_logger().info('Rotate-enable pressed -> calling rotate service(True)')
            started = self.call_rotate_service(True)
            if started:
                # optimistic immediate enable for UX; done-callback may revert on failure
                self._rotate_mode_enabled = True
                # zero linear smoothing state to avoid leakage
                self.prev_twist.linear.x = 0.0
                self._last_valid_twist.linear.x = 0.0
            else:
                self.get_logger().warn('Rotate-service enable call could not be started; staying in normal mode')

        # Disable rotate (edge)
        if 0 <= self.rotate_disable_button < len(buttons) and \
           buttons[self.rotate_disable_button] == 1 and \
           self.last_buttons[self.rotate_disable_button] == 0:
            self.get_logger().info('Rotate-disable pressed -> publishing zero angular then calling rotate service(False)')
            self._publish_instant_zero_angular()
            time.sleep(0.5)  # allow zero angular to take effect

            called = self.call_rotate_service(False)
            if self._rotate_mode_enabled:
                self.get_logger().info('Clearing local rotate flag (disable requested).')
            self._rotate_mode_enabled = False
            if not called:
                self.get_logger().warn('Rotate-service disable call could not be started; rotate mode cleared locally.')

        # update last_buttons snapshot
        self.last_buttons = list(buttons)

    # ---------------- publish timer ----------------
    def _publish_timer_cb(self):
        """Publish at fixed rate. Uses grace + ramp to avoid sudden zero on single missed frames."""
        now = time.time()

        # if rotate mode enabled, ensure rotate service is still available; disable if gone
        try:
            if self._rotate_mode_enabled and not self.rotate_client.wait_for_service(timeout_sec=0.01):
                self.get_logger().warn('Rotate service unavailable -> disabling rotate mode (fallback to normal mode)')
                self._rotate_mode_enabled = False
        except Exception as e:
            if self._rotate_mode_enabled:
                self.get_logger().warn(f'Error checking rotate service ({e}) -> disabling rotate mode')
            self._rotate_mode_enabled = False

        # determine if joy is recent
        joy_recent = False
        if self._last_joy_time is not None:
            joy_recent = (now - self._last_joy_time) <= self.watchdog_timeout

        publish_twist = Twist()

        if joy_recent:
            # fresh joy -> reset counters, publish latest smoothed twist (subject to rotate mode linear zero)
            self._consecutive_misses = 0
            self._ramp_start_time = None

            if self._rotate_mode_enabled:
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = float(self.prev_twist.angular.z)
                publish_twist = t
            else:
                publish_twist = self.prev_twist

            # refresh last valid twist
            self._last_valid_twist = Twist()
            self._last_valid_twist.linear.x = float(publish_twist.linear.x)
            self._last_valid_twist.angular.z = float(publish_twist.angular.z)
        else:
            # no fresh joy this tick
            self._consecutive_misses += 1

            if self._consecutive_misses <= self.miss_grace_count:
                # hold last valid twist (but if rotate mode -> zero linear)
                if self._rotate_mode_enabled:
                    publish_twist = Twist()
                    publish_twist.linear.x = 0.0
                    publish_twist.angular.z = float(self._last_valid_twist.angular.z)
                else:
                    publish_twist = self._last_valid_twist
            else:
                # ramp down from last valid twist to zero
                if self._ramp_start_time is None:
                    self._ramp_start_time = now
                elapsed = now - self._ramp_start_time

                if self.ramp_down_time <= 0.0:
                    factor = 0.0
                else:
                    factor = max(0.0, 1.0 - (elapsed / self.ramp_down_time))

                publish_twist = Twist()
                if self._rotate_mode_enabled:
                    publish_twist.linear.x = 0.0
                    publish_twist.angular.z = self._last_valid_twist.angular.z * factor
                else:
                    publish_twist.linear.x = self._last_valid_twist.linear.x * factor
                    publish_twist.angular.z = self._last_valid_twist.angular.z * factor

                if factor == 0.0:
                    self._last_valid_twist = Twist()
                    self.prev_twist = Twist()

        # publish every tick
        self.cmd_pub.publish(publish_twist)

        if self.debug_joy:
            self.get_logger().info(f'PUBLISH cmd_vel linear={publish_twist.linear.x:.3f} ang={publish_twist.angular.z:.3f}')

    # ---------------- service callers ----------------
    def call_enable_service(self, enable: bool):
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Enable service not available; cannot set')
            return
        req = RosSetChassisEnableCmd.Request()
        try:
            if hasattr(req, 'ros_set_chassis_enable_cmd'):
                setattr(req, 'ros_set_chassis_enable_cmd', bool(enable))
            else:
                # best-effort: try to set a similarly named field, otherwise log and return
                # (adjust if your message field name differs)
                self.get_logger().error('RosSetChassisEnableCmd.Request has no field "ros_set_chassis_enable_cmd"')
                return
        except Exception as e:
            self.get_logger().error(f'Error preparing enable request: {e}')
            return

        try:
            fut = self.enable_client.call_async(req)
            fut.add_done_callback(lambda f: self.get_logger().info(f'set_chassis_enable({enable}) called'))
        except Exception as e:
            self.get_logger().error(f'Failed to call enable service: {e}')

    def call_rotate_service(self, enable: bool) -> bool:
        """
        Calls the rotate enable service. Returns True if call started, False if service not available or request malformed.
        Attaches a done-callback to inspect result and revert optimistic state if needed.
        """
        if not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Rotate service not available; cannot set rotate mode')
            return False

        req = RosEnableChassisRotateCmd.Request()
        try:
            if hasattr(req, 'ros_enable_chassis_rotate_cmd'):
                setattr(req, 'ros_enable_chassis_rotate_cmd', bool(enable))
            else:
                self.get_logger().error('RosEnableChassisRotateCmd.Request does not have field "ros_enable_chassis_rotate_cmd"')
                return False
        except Exception as e:
            self.get_logger().error(f'Error preparing rotate request: {e}')
            return False

        try:
            fut = self.rotate_client.call_async(req)
            # attach callback to handle failures/success
            fut.add_done_callback(lambda f, en=enable: self._rotate_call_done(f, en))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to call rotate service: {e}')
            return False


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
