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
            # Already enabled, just keep running
            return False

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.max_duration_sec:
            self.get_logger().error(f'Failed to enable chassis after {self.max_duration_sec} seconds. Exiting.')
            rclpy.shutdown()
            return True  # Stop spinning

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if getattr(future.result(), 'result', 0) == 1:
                self.get_logger().info('Chassis enabled successfully! Continuing to run.')
                self.enabled = True
                return False  # Keep spinning
            else:
                self.get_logger().warn(f'Chassis enable command failed with result={future.result().result}, retrying...')
        else:
            self.get_logger().warn('Service call failed or no response, retrying...')
        return False  # Keep spinning

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
