#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd
from rclpy.qos import QoSProfile
import time

class ChassisEnable(Node):
    def __init__(self):
        super().__init__('chassis_enable')

        # QoS to ensure reliable delivery
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_const', qos)
        self.subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos)

        self.cmd_vel_msg = Twist()
        self.cmd_vel_received = False

        # Timer to publish at 100 Hz
        self.timer_ = self.create_timer(0.01, self.publish_cmd_vel_const)

        # Delay chassis enable service call by 1 second
        self.create_timer(1.0, self.enable_chassis_once, callback_group=None)
        self.service_called = False

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg
        self.cmd_vel_received = True

    def publish_cmd_vel_const(self):
        if not self.cmd_vel_received:
            self.cmd_vel_msg = Twist()
        self.publisher_.publish(self.cmd_vel_msg)

    def enable_chassis_once(self):
        if self.service_called:
            return
        self.service_called = True
        self.call_enable_service()

    def call_enable_service(self):
        client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /set_chassis_enable not available.')
            rclpy.shutdown()
            return

        request = RosSetChassisEnableCmd.Request()
        request.ros_set_chassis_enable_cmd = True

        attempt = 0
        max_attempts = 5
        success = False

        while attempt < max_attempts and rclpy.ok():
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is not None:
                if future.result().ros_set_chassis_enable_cmd == True:
                    self.get_logger().info('Chassis enabled successfully.')
                    success = True
                    break
                else:
                    self.get_logger().error('Chassis enable failed (response false).')
            else:
                self.get_logger().error('Service call failed or timed out.')

            attempt += 1
            time.sleep(0.5)

        if not success:
            self.get_logger().fatal('Failed to enable chassis after retries.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnable()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
