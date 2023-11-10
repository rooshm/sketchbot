#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix

from sketchbot.srv import Svg2Path


class svg2PathClient(Node):
    """Minimal example client node for svg2path service"""

    def __init__(self):
        """Constructor method"""

        super().__init__('svg2path_client')
        self.cli = self.create_client(Svg2Path, 'svg2path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Svg2Path.Request()

    def send_request(self):
        """Send request to service"""

        # Get absolute path to svg file from python package ROS2 share directory
        install_dir = get_package_prefix('pathgen')
        self.req.file_path = os.path.join(install_dir, '../../src/sketchbot/pathgen/data/out.svg')

        self.req.save_dist = 0.1
        self.req.scale = 0.001
        self.req.square_path = True

        self.future = self.cli.call_async(self.req)


def main(args=None):

    # Initialize rclpy
    rclpy.init(args=args)

    # Create client node
    client = svg2PathClient()
    client.send_request()

    # Spin until future is complete
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.success:
                    client.get_logger().info("Path received")
                else:
                    client.get_logger().info("Path empty")
            break

    # Shutdown
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()