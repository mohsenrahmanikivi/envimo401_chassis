import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd
import time


class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')

        # Declare and create the service client
        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        self.get_logger().info('🔌 Waiting for /set_chassis_enable service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ Waiting for service...')

        # Prepare the request
        self.request = RosSetChassisEnableCmd.Request()
        self.request.ros_set_chassis_enable_cmd = True  # Set the request field properly

        # Time-based retry control
        self.start_time = self.get_clock().now()
        self.max_duration_sec = 12.0  # Fail after 12 seconds
        self.retry_interval_sec = 3
        self.enabled = False

    def try_enable_chassis(self):
        if self.enabled:
            return True  # Already done

        # Compute elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.max_duration_sec:
            self.get_logger().error(f'❌ Timeout after {self.max_duration_sec}s — failed to enable.')
            return True  # Give up, exit

        self.get_logger().info(f'🔄 Attempting to enable chassis (elapsed {elapsed:.1f}s)...')

        # Send async service call
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()

            # Validate field existence (matches .srv file)
            result = getattr(response, 'chassis_set_chassis_enable_result', None)
            if result is None:
                self.get_logger().error('❌ Response missing expected field.')
                return True  # Unexpected error

            if result == 0:
                self.get_logger().info('✅ Chassis enabled successfully.')
                self.enabled = True
                return True  # Done
            else:
                self.get_logger().warn(f'⚠️ Chassis enable failed with result={result}, retrying...')
        else:
            self.get_logger().warn('⚠️ No response from service, retrying...')

        return False  # Retry again


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()

    try:
        # Retry until success or timeout
        while rclpy.ok():
            done = node.try_enable_chassis()
            if done:
                break
            time.sleep(node.retry_interval_sec)

        if node.enabled:
            node.get_logger().info('🚗 Spinning — chassis enabled and running.')
            rclpy.spin(node)  # Stay alive
        else:
            node.get_logger().error('❌ Chassis not enabled. Exiting.')

    except KeyboardInterrupt:
        # Avoid logging if rclpy already shutdown (might cause publisher invalid error)
        if rclpy.ok():
            node.get_logger().info('🛑 Interrupted by user.')

    finally:
        node.destroy_node()
        # Only shutdown if not already shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
