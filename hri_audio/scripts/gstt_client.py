#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered service
# string message # informational, e.g. for error messages

class GSTTClientAsync(Node):

    def __init__(self):
        super().__init__('gstt_srv_client_node')
        self.cli = self.create_client(SetBool, 'gstt_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, cmd):
        self.req.data = cmd
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    gstt_client = GSTTClientAsync()
    start=True;
    response = gstt_client.send_request(start)
    print(response.message)
    gstt_client.get_logger().info(
        'Request complete')


    gstt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()