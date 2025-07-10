import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd

class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')
        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        self.get_logger().info('Waiting for /set_chassis_enable service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_chassis_enable service...')

        self.request = RosSetChassisEnableCmd.Request()
        self.request.ros_set_chassis_enable_cmd = True

        self.start_time = self.get_clock().now()
        self.max_duration_sec = 5.0
        self.retry_interval_sec = 0.5
        self.enabled = False

    def try_enable_chassis(self):
        if self.enabled:
            # Already enabled, keep running, no more retries
            return False

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.max_duration_sec:
            self.get_logger().error(f'Failed to enable chassis after {self.max_duration_sec} seconds. Exiting.')
            rclpy.shutdown()
            return True  # Stop spinning

        self.get_logger().info(f'Attempting to enable chassis (elapsed {elapsed:.2f}s)...')
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is not None:
            self.get_logger().info(f'Service response: {response}')
            # Check common response fields for success
            if hasattr(response, 'result'):
                if response.result == 1:
                    self.get_logger().info('Chassis enabled successfully! Continuing to run.')
                    self.enabled = True
                    return False
                else:
                    self.get_logger().warn(f'Chassis enable command failed with result={response.result}, retrying...')
            elif hasattr(response, 'success'):
                if response.success:
                    self.get_logger().info('Chassis enabled successfully! Continuing to run.')
                    self.enabled = True
                    return False
                else:
                    self.get_logger().warn('Chassis enable command failed (success=False), retrying...')
            else:
                self.get_logger().warn('Unknown service response structure, retrying...')
        else:
            self.get_logger().warn('Service call failed or no response, retrying...')

        return False  # Continue retrying

def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()

    try:
        while rclpy.ok():
            stop = node.try_enable_chassis()
            if stop:
                break
            node.get_clock().sleep_for(rclpy.duration.Duration(seconds=node.retry_interval_sec))
    except KeyboardInterrupt:
        node.get_logger().info('Chassis enable client stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
