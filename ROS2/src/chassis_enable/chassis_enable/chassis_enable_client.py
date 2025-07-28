import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd
import time


class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')

        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        self.get_logger().info('🔌 Waiting for /set_chassis_enable service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ Waiting for service...')

        self.retry_interval_sec = 3
        self.max_duration_sec = 12.0
        self.start_time = self.get_clock().now()

    def call_chassis_enable_service(self, enable: bool) -> (bool, int):
        """
        Call the /set_chassis_enable service with True (enable) or False (disable).
        Returns (success_bool, result_code_int)
        """
        request = RosSetChassisEnableCmd.Request()
        request.ros_set_chassis_enable_cmd = enable

        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} chassis...")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().warn(f"⚠️ No response received when trying to {'enable' if enable else 'disable'} chassis.")
            return False, -1  # Failure with no response

        response = future.result()
        result_code = getattr(response, 'chassis_set_chassis_enable_result', None)

        if result_code is None:
            self.get_logger().error("❌ Response missing expected field 'chassis_set_chassis_enable_result'.")
            return False, -1

        if result_code == 0:
            self.get_logger().info(f"✅ Chassis {'enabled' if enable else 'disabled'} successfully.")
            return True, 0
        else:
            self.get_logger().warn(f"⚠️ Chassis {'enable' if enable else 'disable'} returned result code {result_code}.")
            return True, result_code  # Treated as success with warning code

    def run_enable_sequence(self):
        """Run disable then enable sequence with retries and timeout."""

        start_time = self.get_clock().now()
        elapsed = 0.0

        # Step 1: Disable chassis first
        while elapsed < self.max_duration_sec:
            success, code = self.call_chassis_enable_service(False)
            if success:
                break
            self.get_logger().info(f"Retrying disable in {self.retry_interval_sec}s...")
            time.sleep(self.retry_interval_sec)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
        else:
            self.get_logger().error(f"❌ Timeout disabling chassis after {self.max_duration_sec}s.")
            return False

        # Step 2: Enable chassis
        start_time = self.get_clock().now()
        elapsed = 0.0

        while elapsed < self.max_duration_sec:
            success, code = self.call_chassis_enable_service(True)
            if success:
                return True
            self.get_logger().info(f"Retrying enable in {self.retry_interval_sec}s...")
            time.sleep(self.retry_interval_sec)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

        self.get_logger().error(f"❌ Timeout enabling chassis after {self.max_duration_sec}s.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()

    try:
        if node.run_enable_sequence():
            if rclpy.ok():
                node.get_logger().info("🚗 Chassis is enabled and node is spinning.")
                rclpy.spin(node)
        else:
            node.get_logger().error("❌ Failed to enable chassis after retries. Exiting.")

    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("🛑 Interrupted by user.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
