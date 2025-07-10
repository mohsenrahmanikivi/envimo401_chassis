import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from segway_msgs.srv import RosSetChassisEnableCmd
import time

class ChassisEnable(Node):
    def __init__(self):
        super().__init__('chassis_enable')

        self.latest_cmd_vel = Twist()

        self.publisher = self.create_publisher(Twist, '/cmd_vel_const', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.01, self.publish_cmd_vel)

        success = self.call_chassis_enable_service_with_retries(max_retries=5, delay_sec=0.5)
        if not success:
            self.get_logger().error('Chassis enable failed after retries. Exiting.')
            rclpy.shutdown()

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def publish_cmd_vel(self):
        self.publisher.publish(self.latest_cmd_vel)

    def call_chassis_enable_service_with_retries(self, max_retries=5, delay_sec=0.5):
        client = self.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')

        for attempt in range(1, max_retries + 1):
            self.get_logger().info(f'Attempt {attempt}: Waiting for /set_chassis_enable service...')
            if client.wait_for_service(timeout_sec=5.0):
                request = RosSetChassisEnableCmd.Request()
                request.ros_set_chassis_enable_cmd = True

                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future)

                result = future.result()
                if result and result.ros_set_chassis_enable_cmd:
                    self.get_logger().info('Chassis enabled successfully.')
                    return True
                else:
                    self.get_logger().warn('Service call returned failure.')
            else:
                self.get_logger().warn('Service not available.')

            time.sleep(delay_sec)

        return False

def main(args=None):
    rclpy.init(args=args)
    node = ChassisEnable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
