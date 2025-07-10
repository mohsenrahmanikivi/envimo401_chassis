import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd
import time


class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')
        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_chassis_enable service...')

        request = RosSetChassisEnableCmd.Request()
        request.ros_set_chassis_enable_cmd = True

        max_retries = 5
        for i in range(max_retries):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info('Chassis enable command succeeded.')
                break
            else:
                self.get_logger().warn(f'Attempt {i + 1} failed, retrying...')
                time.sleep(0.5)
        else:
            self.get_logger().error('Failed to enable chassis after 5 attempts.')


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()
    node.destroy_node()
    rclpy.shutdown()
