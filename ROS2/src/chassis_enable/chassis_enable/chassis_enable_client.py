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
        for attempt in range(max_retries):
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info('Service call succeeded')
                break
            else:
                self.get_logger().warn(f'Service call failed. Retrying ({attempt + 1}/{max_retries})...')
                time.sleep(0.5)

        self.get_logger().info('Exiting client node.')


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()
    node.destroy_node()
    rclpy.shutdown()
