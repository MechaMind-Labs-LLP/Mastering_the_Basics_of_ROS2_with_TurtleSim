#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtle_msgs.srv import ComputeRectangleArea


class AreaServiceClient(Node):
    def __init__(self):
        super().__init__('area_service_client')

        # Declare parameters (default values)
        self.declare_parameter('length', 5.0)
        self.declare_parameter('width', 3.0)

        # Get parameter values
        length = self.get_parameter('length').value
        width = self.get_parameter('width').value

        self.get_logger().info(f"Requesting area calculation for length={length}, width={width}")

        # Create service client
        self.client = self.create_client(ComputeRectangleArea, 'calculate_area')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Prepare request
        self.request = ComputeRectangleArea.Request()

        self.request.length = length
        self.request.width = width

        # Call service asynchronously
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Area received: {response.area}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main():
    rclpy.init()
    node = AreaServiceClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
