#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode
import time

class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')

        # Setup client
        self.cli_cmd = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cli_cmd.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/command service...')

    def set_servo(self, servo_id, value):
        """Set a servo to a specific value."""
        req = CommandLong.Request()
        req.command = CommandCode.CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = servo_id
        req.param2 = value
        self.cli_cmd.call_async(req).add_done_callback(self.cmd_cb)

    def cmd_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Set servo ✓')
            else:
                self.get_logger().error('Set servo ✗')
        except Exception as e:
            self.get_logger().error(f'Failed to call service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoTester()

    # Example: test servos 1 to 4 with PWM = 1500
    for i in range(1, 5):
        node.get_logger().info(f'Setting servo {i} to 1500...')
        node.set_servo(i, 1500)
        rclpy.spin_once(node, timeout_sec=1.0)
        time.sleep(1.0)

    node.get_logger().info('Servo test complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
