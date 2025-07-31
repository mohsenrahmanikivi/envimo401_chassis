import rclpy
from rclpy.node import Node
from segway_msgs.srv import RosSetChassisEnableCmd
from geometry_msgs.msg import Twist
import time
import threading

class ChassisEnableClient(Node):
    def __init__(self):
        super().__init__('chassis_enable_client')
        self.client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        self.get_logger().info('🔌 Waiting for /set_chassis_enable service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ Waiting for service...')

        self.retry_interval_sec = 3
        self.max_duration_sec = 12.0
        self.cmd_vel_received = False
        self.shutdown_requested = False

        self.create_subscription(
            Twist,
            '/cmd_vel_const',
            self.cmd_vel_callback,
            10
        )

        # Use a timer to check shutdown_requested flag and exit cleanly
        self.create_timer(0.5, self.check_shutdown)

    def cmd_vel_callback(self, msg):
        if not self.cmd_vel_received:
            self.get_logger().info('📡 Received first /cmd_vel_const message!')
        self.cmd_vel_received = True

    def call_chassis_enable_service(self, enable: bool) -> (bool, int):
        request = RosSetChassisEnableCmd.Request()
        request.ros_set_chassis_enable_cmd = enable

        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} chassis...")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().warn(f"⚠️ No response received when trying to {'enable' if enable else 'disable'} chassis.")
            return False, -1

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
            return True, result_code

    def run_enable_sequence(self):
        # Disable chassis first with retries
        start_time = self.get_clock().now()
        elapsed = 0.0

        while elapsed < self.max_duration_sec:
            success, code = self.call_chassis_enable_service(False)
            if success:
                self.get_logger().info(f"⏳ Sleeping 2 seconds after disable command...")
                time.sleep(2)  # real delay
                break
            self.get_logger().info(f"Retrying disable in {self.retry_interval_sec}s...")
            time.sleep(self.retry_interval_sec)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
        else:
            self.get_logger().error(f"❌ Timeout disabling chassis after {self.max_duration_sec}s.")
            return False

        # Enable chassis with retries
        start_time = self.get_clock().now()
        elapsed = 0.0

        while elapsed < self.max_duration_sec:
            success, code = self.call_chassis_enable_service(True)
            if success:
                self.get_logger().info(f"⏳ Sleeping 2 seconds after enable command...")
                time.sleep(2)
                return True
            self.get_logger().info(f"Retrying enable in {self.retry_interval_sec}s...")
            time.sleep(self.retry_interval_sec)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

        self.get_logger().error(f"❌ Timeout enabling chassis after {self.max_duration_sec}s.")
        return False

    def wait_for_cmd_vel_const(self, timeout_sec=20):
        self.get_logger().info(f"⏳ Waiting for /cmd_vel_const message up to {timeout_sec}s...")
        start_time = self.get_clock().now()
        while not self.cmd_vel_received:
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                self.get_logger().error(f"❌ Timeout waiting for /cmd_vel_const message.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("✅ /cmd_vel_const message received.")
        return True

    def check_shutdown(self):
        if self.shutdown_requested:
            self.get_logger().info('🛑 Shutdown requested: disabling chassis and shutting down...')
            self.call_chassis_enable_service(False)
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnableClient()

    # Use threading event to handle shutdown signal safely
    def on_shutdown(*args):
        node.shutdown_requested = True

    import signal
    signal.signal(signal.SIGINT, on_shutdown)
    signal.signal(signal.SIGTERM, on_shutdown)

    try:
        if node.run_enable_sequence():
            if node.wait_for_cmd_vel_const():
                node.get_logger().info("🚗 Chassis enabled and received /cmd_vel_const. Spinning node...")
                while rclpy.ok() and not node.shutdown_requested:
                    rclpy.spin_once(node, timeout_sec=0.1)
            else:
                node.get_logger().error("❌ Did not receive /cmd_vel_const message in time. Exiting.")
        else:
            node.get_logger().error("❌ Failed to enable chassis after retries. Exiting.")

    except Exception as e:
        node.get_logger().error(f"❌ Exception in main: {e}")

    finally:
        if rclpy.ok():
            node.get_logger().info("🧹 Cleaning up: disabling chassis before exit...")
            node.call_chassis_enable_service(False)
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
