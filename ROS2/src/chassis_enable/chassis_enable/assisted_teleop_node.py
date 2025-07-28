import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator


class AssistedTeleopNode(Node):
    def __init__(self):
        super().__init__('assisted_teleop_node')
        self.get_logger().info('Starting Assisted Teleop Node (lifetime mode)...')

        # Initialize navigator and wait for Nav2 stack to be active
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active.')

        # Start Assisted Teleop with no time limit
        self.navigator.assistedTeleop(time_allowance=0)
        self.get_logger().info('AssistedTeleop action started with indefinite duration.')

        # No timers, no subscriptions — just keep the node alive
        self.get_logger().info('Assisted Teleop Node will keep running.')


def main(args=None):
    rclpy.init(args=args)
    node = AssistedTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Assisted Teleop Node stopped by user.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

