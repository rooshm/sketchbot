import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from sketchbot_interfaces.srv import Img2Svg, Svg2Path, Drawbot


class DrawbotService(Node):
  def __init__(self):
    super().__init__('drawbot_service')

    # Setup orchestrator service
    self.srv = self.create_service(Drawbot, '/drawbot', self.drawbot_callback)

    # Subscribe to /camera/camera/color/image_raw topic and save value
    self.image = None
    self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

    # Setup clients
    self.img2svg_cli = self.create_client(Img2Svg, '/img2svg')
    self.svg2path_cli = self.create_client(Svg2Path, '/svg2path')

    while not self.img2svg_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('img2svg service not available, waiting...')

    while not self.svg2path_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('svg2path service not available, waiting...')


  def drawbot_callback(self, request, response):
    # Send image to img2svg service
    req = Img2Svg.Request()
    req.image = self.image
    req.resolution = 1024
    req.length_threshold = 32

    img2svg_future = self.img2svg_cli.call_async(req)
    rclpy.spin_until_future_complete(self, img2svg_future)

    if img2svg_future.result() is None:
      self.get_logger().info('Service call failed %r' % (img2svg_future.exception(),))
      return

    # Get vector lines from img2svg service
    lines_json = img2svg_future.result().lines_json

    # Send vector lines to svg2path service
    req = Svg2Path.Request()
    # req.lines_json = lines_json
    req.file_path = '../../data/out.svg'
    req.save_dist = -0.1
    req.scale = 0.001
    req.square_path = True

    svg2path_future = self.svg2path_cli.call_async(req)
    rclpy.spin_until_future_complete(self, svg2path_future)

    if svg2path_future.result() is None:
      self.get_logger().info('Service call failed %r' % (svg2path_future.exception(),))
      return

    # Get path from svg2path service
    path = svg2path_future.result().path

    # TODO: call controller service with path


  def image_callback(self, msg):
    self.image = msg
