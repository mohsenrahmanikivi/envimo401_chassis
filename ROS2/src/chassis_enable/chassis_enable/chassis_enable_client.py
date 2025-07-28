import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd
import time


class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')
        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        self.get_logger().info('Waiting for /set_chassis_enable service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_chassis_enable service...')

        # Prepare the request
        self.request = RosSetChassisEnableCmd.Request()
        self.request.ros_set_chassis_enable_cmd = True

        # Retry settings
        self.start_time = self.get_clock().now()
        self.max_duration_sec = 12.0
        self.retry_interval_sec = 3
        self.enabled = False

    def try_enable_chassis(self):
        if self.enabled:
            return True  # Already enabled, nothing to do

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.max_duration_sec:
            self.get_logger().error(f'❌ Failed to enable chassis after {self.max_duration_sec} seconds. Exiting.')
            return True  # Stop trying

        self.get_logger().info(f'🔄 Attempting to enable chassis (elapsed {elapsed:.2f}s)...')
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is not None:
            self.get_logger().info(f'📨 Service response: {response}')
            # Correct field name
            if hasattr(response, 'chassis_set_chassis_enable_result'):
                if response.chassis_set_chassis_enable_result == 0:
                    self.get_logger().info('✅ Chassis enabled successfully!')
                    self.enabled = True
                    return True  # Stop retrying
                else:
                    self.get_logger().warn(
                        f'⚠️ Enable failed (result={response.chassis_set_chassis_enable_result}), retrying...')
            else:
                self.get_logger().warn('⚠️ Unknown response structure, retrying...')
        else:
            self.get_logger().warn('⚠️ No response from service, retrying...')

        return False  # Retry again


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()

    try:
        while rclpy.ok():
            stop = node.try_enable_chassis()
            if stop:
                break
            time.sleep(node.retry_interval_sec)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Chassis enable client interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
