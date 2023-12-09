#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt

# ROS2 related imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, TransformStamped

# Import custom service
from sketchbot_interfaces.srv import Svg2Path

# To handle svg parsing, can be installed using pip
# pip install svgelements
# Dependency can be removed if image processing pipeline handles svg parsing
from svgelements import *


class svg2path():
    """Converts svg file to a path object for robot to follow
    Adds non-draw air path between each draw path segment
    Creates a path as list of lines with each line as list of points

    Args:
        svg_path (str): path to svg file
        save_dist (float): distance for pen to raise between draw segments (in m)
        pix_scale (float): scale factor to convert svg pixels to meters (in m)
        square_path (bool): if True, the air path between draw segments is a square, instead of triangular. Default: True
        add_sign (bool): if True, adds the robot signature to the end of the path. Default: True
    """

    def __init__(self, svg_path:str, save_dist:float=-0.1, pix_scale:float=0.000175, square_path:bool=True, add_sign:bool=True):
        """Constructor method"""

        # Save parameters to class variables
        self.svg_path = os.path.join(os.getcwd(), svg_path)
        self.save_dist = save_dist
        self.pix_scale = pix_scale
        self.square_path = square_path
        self.add_sign = add_sign

        # Storage lists
        self.lines = []
        self.draw_path = []
        self.air_path = []
        self.signature_path = []
        self.complete_path = []

        # Parse svg file to extract points
        self.svg = SVG.parse(self.svg_path)

        for element in self.svg.elements():
            if isinstance(element, Polyline):
                pts = element.points
                points = []
                for i in range(len(pts)):
                    # Convert from svg pixels to meters
                    # z-coordinate is always 0 as paper touching
                    pt = (pts[i].x * self.pix_scale, pts[i].y * self.pix_scale, 0.0)
                    points.append(pt)
                self.lines.append(points)

        # Create a distance matrix between end and start points of each line
        # Arbitrary large value for same line distance to avoid picking same line twice
        self.dist_mat = np.identity(len(self.lines)) * 4e5
        for i in range(len(self.lines)):
            for j in range(len(self.lines)):
                if i != j:
                    self.dist_mat[i, j] = np.linalg.norm(np.array(self.lines[i][-1]) - np.array(self.lines[j][0]))

        # Parse signature svg file to extract points
        self.sign_svg_path = os.path.join(os.getcwd(), "src/sketchbot/data/signature.svg")
        self.sign_svg = SVG.parse(self.sign_svg_path)
        self.sign_scale = 0.00008

        for element in self.sign_svg.elements():
            if isinstance(element, Polyline):
                pts = element.points
                points = []
                for i in range(len(pts)):
                    # Convert from svg pixels to meters
                    # z-coordinate is always 0 as paper touching
                    pt = (pts[i].x * self.sign_scale, pts[i].y * self.sign_scale, 0.0)

                    # Offset signature to the bottom right corner of the paper
                    pt = (pt[0] + self.pix_scale * 550, pt[1] + self.pix_scale * 450, pt[2])

                    points.append(pt)

                # Add the start and end points of signature to the signature path with save_dist z-value
                sign_start = np.array(points[0])
                sign_start[2] = self.save_dist
                sign_end = np.array(points[-1])
                sign_end[2] = self.save_dist
                self.signature_path.append([sign_start])
                self.signature_path.append(points)
                self.signature_path.append([sign_end])

    def get_air_path(self, start_line:int, end_line:int):
        """Gets intermediate air path between two draw paths by interpolation

        Args:
            start_line (int): index of start line
            end_line (int): index of end line

        Returns:
            air_path (list): list of points to follow for air path
        """

        # Check if start and end lines index in range
        if start_line < 0 or start_line >= len(self.lines):
            raise ValueError("start_line index out of range")
        if end_line < 0 or end_line >= len(self.lines):
            raise ValueError("end_line index out of range")

        # Check if start and end lines are the same
        if start_line == end_line:
            return []

        # Get start and end points of start and end lines
        start_pt = np.array(self.lines[start_line][-1])
        end_pt = np.array(self.lines[end_line][0])

        # Get the middle point between start and end points
        mid_pt = (start_pt + end_pt) / 2
        mid_pt[2] = self.save_dist

        # Get the air path between start and end points
        air_path = []
        if self.square_path:
            # Get the square path between start and end points
            start_pt[2] = self.save_dist
            end_pt[2] = self.save_dist

            air_path.append(start_pt)
            air_path.append(mid_pt)
            air_path.append(end_pt)
        else:
            # Get the triangular path between start and end points
            air_path.append(mid_pt)

        return air_path

    def get_path(self):
        """Computes an ordered list of points to follow for drawing in an optimal order"""

        # Get distance of start point of each line from origin
        start_dist = []
        for i in range(len(self.lines)):
            start_dist.append(np.linalg.norm(np.array(self.lines[i][0])))

        # Pick the line with the shortest distance from origin as the first line
        first_line = np.argmin(start_dist)
        self.draw_path.append(self.lines[first_line])
        last_line_idx = first_line
        used_lines = [last_line_idx]

        # Add the first line first point in the air path list with save_dist z-value
        first_point = np.array(self.lines[first_line][0])
        first_point[2] = self.save_dist
        self.air_path.append([first_point])

        # In loop find the next line with the shortest distance from the end of the previous line
        # Add air path between each draw path segment
        while len(self.draw_path) < len(self.lines):
            next_line_order = np.argsort(self.dist_mat[last_line_idx])
            # Pick the next line with the shortest distance from the end of the previous line and not already used
            for i in range(len(next_line_order)):
                if next_line_order[i] not in used_lines:
                    next_line_idx = next_line_order[i]
                    break

            self.draw_path.append(self.lines[next_line_idx])
            self.air_path.append(self.get_air_path(last_line_idx, next_line_idx))
            last_line_idx = next_line_idx
            used_lines.append(last_line_idx)

        # Add the last line last point to air path list with save_dist z-value
        last_point = np.array(self.lines[last_line_idx][-1])
        last_point[2] = self.save_dist
        self.air_path.append([last_point])

        # Merge draw path and air path lists
        for i in range(len(self.air_path)):
            self.complete_path.append(self.air_path[i])
            if i < len(self.draw_path):
                self.complete_path.append(self.draw_path[i])

        # Add signature path to the end of the complete path
        if self.add_sign:
            for i in range(len(self.signature_path)):
                self.complete_path.append(self.signature_path[i])

        return self.complete_path

    def plot_path(self, is_scatter:bool=False, show_numbers:bool=False):
        """Create a 3D plot of the path

        Args:
            is_scatter (bool): if True, plots the path as a scatter plot. Default: False
            show_numbers (bool): if True, shows the number of path selection order at the start of each line. Default: False
        """

        fig = plt.figure("Robot Path")
        ax = fig.add_subplot(111, projection='3d')
        count = 0

        for i in range(len(self.complete_path)):
            x = []
            y = []
            z = []
            for j in range(len(self.complete_path[i])):
                x.append(self.complete_path[i][j][0])
                y.append(self.complete_path[i][j][1])
                z.append(self.complete_path[i][j][2])
                count += 1

            if is_scatter:
                # Plot as scatter plot for each point
                ax.scatter(x, y, z, s=1)
            else:
                # Plot as line plot for path
                ax.plot(x, y, z)

            # Plot number at start of each line
            if show_numbers and not is_scatter:
                ax.text(x[0], y[0], z[0], str(i+1), color='red')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.title.set_text(f"Total Way-points: {count}")

        return ax


class svg2PathService(Node):
    """ROS2 service wrapper for svg2path class"""

    def __init__(self):
        """Constructor method"""

        super().__init__('svg2path_service')

        # Create service
        self.srv = self.create_service(Svg2Path, '/svg2path', self.svg2path_callback)

        # Publisher for path visualization
        self.path_pub = self.create_publisher(PoseArray, '/draw_path', 10)

        # Create a timer to publish path at 0.2 Hz
        self.timer_period = 0.2 # Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize path message
        self.path_msg = PoseArray()
        self.path_msg.header.frame_id = "draw_board"

        # Initialize tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = TransformStamped()

    def svg2path_callback(self, request, response):
        """Service callback function"""

        # Extract parameters from request
        svg_path = request.file_path
        save_dist = request.save_dist
        pix_scale = request.scale
        square_path = request.square_path
        add_sign = request.add_sign

        # Create svg2path object
        pathCreate = svg2path(svg_path, save_dist, pix_scale, square_path, add_sign)

        # Get path
        pathCreate.get_path()

        # Constant orientation for all points
        quat = Quaternion()
        quat.x = 1e-6
        quat.y = 1e-6
        quat.z = 1e-6
        quat.w = 1.0

        # Clear the existing poses
        self.path_msg.poses.clear()

        # Lookup transform from draw_board to world frame
        try:
            self.transform = self.tf_buffer.lookup_transform("world", "draw_board", rclpy.time.Time())
            # Set the PoseArray frame_id to world
            self.path_msg.header.frame_id = "world"
        except:
            self.get_logger().info('Could not lookup transform from draw_board to world frame')
            # Set transform to identity
            self.transform.transform.translation.x = 0.0
            self.transform.transform.translation.y = 0.0
            self.transform.transform.translation.z = 0.0
            self.transform.transform.rotation.x = 0.0
            self.transform.transform.rotation.y = 0.0
            self.transform.transform.rotation.z = 0.0
            self.transform.transform.rotation.w = 1.0

            # Set the PoseArray frame_id to draw_board
            self.path_msg.header.frame_id = "draw_board"

        # Append poses to PoseArray message
        for i in range(len(pathCreate.complete_path)):
            for j in range(len(pathCreate.complete_path[i])):
                pose = Pose()
                pose.position.x = pathCreate.complete_path[i][j][0]
                pose.position.y = pathCreate.complete_path[i][j][1]
                pose.position.z = pathCreate.complete_path[i][j][2]
                pose.orientation = quat

                # Transform pose to world frame using tf2
                pose = do_transform_pose(pose, self.transform)
                self.path_msg.poses.append(pose)

        # Save path to response
        response.path = self.path_msg

        # Set response success flag if path is not empty
        if len(response.path.poses) > 0:
            response.success = True
            self.get_logger().info('Path generated successfully with %d way-points' % len(response.path.poses))
        else:
            response.success = False
            self.get_logger().info('Path generation failed')

        return response

    def timer_callback(self):
        """Timer callback function"""

        # Publish path
        self.path_pub.publish(self.path_msg)


def main(args=None):
    """Main function"""

    # Initialize rclpy
    rclpy.init(args=args)

    # Create svg2path service
    svg2path_service = svg2PathService()

    # Spin the service
    rclpy.spin(svg2path_service)

    # Shutdown rclpy
    rclpy.shutdown()


if __name__ == "__main__":
    main()