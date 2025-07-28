import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_const', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.01, self.publish_loop)  # 100 Hz

        self.last_msg = Twist()
        self.last_msg_time = self.get_clock().now()

        self.no_cmd_vel_state = False  # Track if we are currently in silent mode (no cmd_vel)

        self.get_logger().info('CmdVelRelay node started. Listening to /cmd_vel and publishing to /cmd_vel_const at 100Hz.')

    def cmd_vel_callback(self, msg):
        self.last_msg = msg
        self.last_msg_time = self.get_clock().now()

        if self.no_cmd_vel_state:
            self.get_logger().info('Received /cmd_vel messages again.')
            self.no_cmd_vel_state = False

        self.get_logger().debug(f'Received /cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

    def publish_loop(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if elapsed > 0.1:
            # Silent: Publish zero velocity
            self.publisher.publish(Twist())
            if not self.no_cmd_vel_state:
                self.get_logger().warn('No /cmd_vel received in last 100ms. Publishing zero velocity.')
                self.no_cmd_vel_state = True
        else:
            self.publisher.publish(self.last_msg)
            self.get_logger().debug(f'Publishing to /cmd_vel_const: linear={self.last_msg.linear.x:.2f}, angular={self.last_msg.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CmdVelRelay node stopped by user.')
    finally:
        node.destroy_node()
        # Avoid double shutdown error by checking if rclpy is still OK
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
