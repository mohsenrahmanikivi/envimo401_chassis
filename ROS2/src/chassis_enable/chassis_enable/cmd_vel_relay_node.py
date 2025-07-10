import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from time import time


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_const', qos)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos)
        self.timer = self.create_timer(0.01, self.publish_cmd_vel_const)  # 100 Hz

        self.last_msg = Twist()
        self.last_time = time()

    def cmd_vel_callback(self, msg):
        self.last_msg = msg
        self.last_time = time()

    def publish_cmd_vel_const(self):
        if time() - self.last_time > 0.1:  # No msg in last 0.1s
            zero = Twist()
            self.publisher.publish(zero)
        else:
            self.publisher.publish(self.last_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
