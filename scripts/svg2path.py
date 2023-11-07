import os
import numpy as np
import matplotlib.pyplot as plt

# ROS2 related imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

# To handle svg parsing, can be installed using pip
# pip install svgelements
# Dependency can be removed if image processing pipeline handles svg parsing
from svgelements import *


class svg2path(Node):
    """Converts svg file to a path object for robot to follow
    Adds non-draw air path between each draw path segment
    Creates a PoseStamped for the optimal Path and publishes to a ROS2 topic

    Args:
        svg_path (str): path to svg file
        save_dist (float): distance for pen to raise between draw segments (in m)
        pix_scale (float): scale factor to convert svg pixels to meters (in m)
        square_path (bool): if True, the air path between draw segments is a square, instead of triangular. Default: True
        topic_name (str): name of topic to publish path to. Default: "/draw_path"
    """

    def __init__(self, svg_path:str, save_dist:float=0.1, pix_scale:float=0.001, square_path:bool=True, topic_name:str="/draw_path"):
        """Constructor method"""

        # Save parameters to class variables
        self.svg_path = os.path.join(os.getcwd(), svg_path)
        self.save_dist = save_dist
        self.pix_scale = pix_scale
        self.square_path = square_path
        self.topic_name = topic_name

        # Storage lists
        self.lines = []
        self.draw_path = []
        self.air_path = []
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

        # Create ROS2 node to publish path
        super().__init__("svg2path")
        self.pub = self.create_publisher(PoseArray, self.topic_name, 1)
        self.ros_path = PoseArray()

        # Need to add a subscriber to get the incoming image and frame id
        # Currently using a constant frame id and a saved svg file path
        # self.sub = self.create_subscription(FloatArray, "/image", self.callback, 1)

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

        # Merge draw path and air path
        for i in range(len(self.draw_path)):
            self.complete_path.append(self.draw_path[i])
            if i < len(self.air_path):
                self.complete_path.append(self.air_path[i])

        return self.complete_path

    def plot_path(self, show_numbers:bool=False):
        """Create a 3D plot of the path

        Args:
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
            ax.plot(x, y, z)

            # Plot number at start of each line
            if show_numbers:
                ax.text(x[0], y[0], z[0], str(i+1), color='red')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.title.set_text(f"Total Path Length: {len(self.complete_path)} | Total Points: {count}")

        return ax

    def publish_path(self):
        """Publishes path to ROS2 topic"""

        # Set frame id
        self.ros_path.header.frame_id = "/map" # Need to update as per incoming msg

        # Set constant orientation
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0

        # Get each point from path as a pose and append to path
        for i in range(len(self.complete_path)):
            for j in range(len(self.complete_path[i])):
                pose = Pose()
                pose.position.x = self.complete_path[i][j][0]
                pose.position.y = self.complete_path[i][j][1]
                pose.position.z = self.complete_path[i][j][2]
                pose.orientation = quat

                self.ros_path.poses.append(pose)

        # Publish path
        self.pub.publish(self.ros_path)

def main(svg_path:str, save_dist:float=0.1, pix_scale:float=0.001, square_path:bool=True, topic_name:str="/draw_path", args=None):
    """Main function to run svg2path class"""

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create svg2path object
    pathCreate = svg2path(svg_path, save_dist, pix_scale, square_path, topic_name)

    # Get path
    pathCreate.get_path()
    # Publish path
    pathCreate.publish_path()
    # Plot path
    pathCreate.plot_path()

    ## TODO: This ROS structure will change as per system integration
    # Destroy the node
    pathCreate.destroy_node()
    # Shutdown ROS client
    rclpy.shutdown()


if __name__ == "__main__":

    # Test parameters
    svg_path = "../data/out.svg"
    save_dist = 0.1
    pix_scale = 0.01
    square_path = True
    topic_name = "/draw_path"

    # Run main function
    main(svg_path, save_dist, pix_scale, square_path, topic_name)

    # Show plot
    plt.show()