import sys

from custom_interfaces.srv import MyServiceMessage
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(MyServiceMessage, 'my_srv_msg')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MyServiceMessage.Request()

    def send_request(self, label):
        self.req.label = label
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(str(sys.argv[1]))
    minimal_client.get_logger().info(
        response.message)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()