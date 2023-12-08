#!/usr/bin/env python3

import os
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix

from sketchbot_interfaces.srv import Svg2Path


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

        # Path is required in service request
        self.req.file_path = os.path.join(install_dir, '../../src/sketchbot/data/out.svg')

        # Optional parameters for service request as default values are set in service definition
        self.req.save_dist = -0.1 # -ve z-value as /draw_board z-axis is pointing down into the table.
        self.req.scale = 0.00035
        self.req.square_path = True
        self.req.add_sign = True

        self.future = self.cli.call_async(self.req)

    def plot_path(self, path, is_scatter:bool=True):
        """Plot the PoseArray path"""

        # Extract coordinates from path
        x = []
        y = []
        z = []
        count = 0
        for pose in path.poses:
            x.append(pose.position.x)
            y.append(pose.position.y)
            z.append(pose.position.z)
            count += 1

        # Plot path
        fig = plt.figure("Robot Path")
        ax = fig.add_subplot(111, projection='3d')

        if is_scatter:
            # Plot as scatter plot for each point
            ax.scatter(x, y, z, s=1)
        else:
            # Plot as line plot for path
            ax.plot(x, y, z)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.title.set_text(f"Total Way-points: {count}")

        # Show plot - blocking
        plt.show()


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
                    # client.plot_path(response.path, is_scatter=True)
                else:
                    client.get_logger().info("Path empty")
            break

    # Shutdown
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
