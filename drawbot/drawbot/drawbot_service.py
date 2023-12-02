import asyncio
import threading

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sketchbot_interfaces.srv import Drawbot, Img2Svg, Svg2Path, Move2State, Move2Pose, FollowPath
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetRobotStateFromWarehouse, GetCartesianPath, ListRobotStatesInWarehouse

class DrawbotService(Node):
  def __init__(self):
    super().__init__('drawbot_service')

    # Setup orchestrator service
    self.srv = self.create_service(Drawbot, '/drawbot', self.drawbot_callback)

    # Subscribe to /camera/camera/color/image_raw topic and save value
    self.image = None

    # Load image from file
    # img = PILImage.open('src/sketchbot/data/test.jpg')
    # img = img.resize((640, 480))

    # self.image = Image()
    # self.image.header.frame_id = 'camera'
    # self.image.height = 480
    # self.image.width = 640
    # self.image.encoding = 'rgb8'
    # self.image.data = np.asarray(img).tobytes()

    self.robot_state = RobotState()

    self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
    self.create_subscription(JointState, '/joint_states', self.curr_joint_state, 1)

    # Setup clients
    self.getstate_cli = self.create_client(GetRobotStateFromWarehouse, '/get_robot_state')
    self.move2state_cli = self.create_client(Move2State, '/move2state')
    self.move2pose_cli = self.create_client(Move2Pose, '/move2pose')
    self.img2svg_cli = self.create_client(Img2Svg, '/img2svg')
    self.svg2path_cli = self.create_client(Svg2Path, '/svg2path')
    self.getdrawtraj_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
    self.movecartesian_cli = self.create_client(FollowPath, '/followpath')

    while not self.getstate_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('getstate_cli service not available, waiting...')

    while not self.move2state_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('move2state_cli service not available, waiting...')
    
    while not self.move2pose_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('move2pose_cli service not available, waiting...')

    while not self.img2svg_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('img2svg service not available, waiting...')

    while not self.svg2path_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('svg2path service not available, waiting...')
    
    while not self.getdrawtraj_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('compute_cartesian_path service not available, waiting...')
    
    while not self.movecartesian_cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('movecartesian_cli service not available, waiting...')


  def drawbot_callback(self, request, response):
    threading.Thread(target=self.threaded_drawbot_request, args=(request, response)).start()
    return response


  def threaded_drawbot_request(self, request, response):
    # Create a new event loop for the thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Now you can run the async method in this loop
    loop.run_until_complete(self.handle_drawbot_request(request, response))
    loop.close()


  async def handle_drawbot_request(self, request, response):
    req = GetRobotStateFromWarehouse.Request()
    req.name = "say_cheese"
    req.robot = "ur"
    self.get_logger().info(f'Calling service to get robot state {req.name}')
    getstate_future = self.getstate_cli.call_async(req)
    await getstate_future

    if getstate_future.result() is None:
      self.get_logger().info('Service call failed %r' % (getstate_future.exception(),))
      return

    self.get_logger().warn(f'Got robot state {getstate_future.result().state.joint_state}')
    req = Move2State.Request()
    req.goal_state = getstate_future.result().state
    self.get_logger().info('Move to state')
    move2state_future = self.move2state_cli.call_async(req)
    await move2state_future

    if move2state_future.result() is None or move2state_future.result().state is False:
      self.get_logger().info('Service call failed %r' % (move2state_future.exception(),))
      return

    # if self.image is None:
    #   self.get_logger().info('No image received')
    #   return

    # self.get_logger().info('Took image, sending to img2svg service')

    # # Send image to img2svg service
    # req = Img2Svg.Request()
    # req.image = self.image
    # req.resolution = 1024
    # req.length_threshold = 32

    # self.get_logger().info('Calling img2svg service')
    # img2svg_future = self.img2svg_cli.call_async(req)
    # await img2svg_future

    # if img2svg_future.result() is None:
    #   self.get_logger().info('Service call failed %r' % (img2svg_future.exception(),))
    #   return

    # # Get vector lines from img2svg service
    # lines_json = img2svg_future.result().lines_json

    # self.get_logger().info('Got lines from img2svg service, sending to svg2path service')

    # Send vector lines to svg2path service
    req = Svg2Path.Request()
    # req.lines_json = lines_json
    req.file_path = 'src/sketchbot/data/out.svg'
    req.save_dist = -0.025 # m safe distance from table
    req.scale = 0.000175 # Converts 1024px to 18cm
    req.square_path = True

    self.get_logger().info('Calling svg2path service')
    svg2path_future = self.svg2path_cli.call_async(req)
    await svg2path_future

    if svg2path_future.result() is None:
      self.get_logger().info('Service call failed %r' % (svg2path_future.exception(),))
      return

    # Get path from svg2path service
    path_pose_array = svg2path_future.result().path

    self.get_logger().info(f'Got path from svg2path service {len(path_pose_array.poses)} going to start draw')

    req = GetRobotStateFromWarehouse.Request()
    req.name = "start_draw"
    req.robot = "ur"
    self.get_logger().info(f'Calling service to get robot state {req.name}')
    getstate_future = self.getstate_cli.call_async(req)
    await getstate_future

    if getstate_future.result() is None:
      self.get_logger().info('Service call failed %r' % (getstate_future.exception(),))
      return

    self.get_logger().warn(f'Got robot state {getstate_future.result().state.joint_state}')
    req = Move2State.Request()
    req.goal_state = getstate_future.result().state
    curr_state = req.goal_state
    self.get_logger().info('Move to start draw')
    move2state_future = self.move2state_cli.call_async(req)
    await move2state_future

    if move2state_future.result() is None or move2state_future.result().state is False:
      self.get_logger().info('Service call failed %r' % (move2state_future.exception(),))
      return

    # self.get_logger().info('Go to Pose 1 from svgpath')
    # req = Move2Pose.Request()
    # newpose = PoseStamped()
    # newpose.header = path_pose_array.header
    # newpose.pose = path_pose_array.poses[0]
    # req.goal_state = newpose
    # move2pose_future = self.move2pose_cli.call_async(req)
    # await move2pose_future

    # if move2pose_future.result() is None or move2pose_future.result().state is False:
    #   self.get_logger().info('Service call failed %r' % (move2pose_future.exception(),))
    #   return

    self.get_logger().info('Compute cartesian path')
    req = GetCartesianPath.Request()
    req.header = path_pose_array.header
    req.start_state = self.robot_state
    req.group_name = "ur_manipulator"
    req.link_name = "drawbot_tool_pen_tip"
    req.max_step = 0.01
    req.jump_threshold = 0.
    req.avoid_collisions = True
    req.waypoints = path_pose_array.poses
    getdrawtraj_future = self.getdrawtraj_cli.call_async(req)
    await getdrawtraj_future

    if getdrawtraj_future.result() is None:
      self.get_logger().info('Service call failed %r' % (getdrawtraj_future.exception()))
    elif getdrawtraj_future.result().error_code.val != MoveItErrorCodes.SUCCESS:
      self.get_logger().info(f"Error Code {getdrawtraj_future.result().error_code}")
      return
    self.get_logger().info(f'Fraction of cartesian path successfully planned {getdrawtraj_future.result().fraction}')

    self.get_logger().info('Execute cartesian path')
    req = FollowPath.Request()
    req.robot_trajectory = getdrawtraj_future.result().solution
    movecartesian_future = self.movecartesian_cli.call_async(req)
    await movecartesian_future

    if movecartesian_future.result() is None or movecartesian_future.result().state is False:
      self.get_logger().info('Service call failed %r' % (movecartesian_future.exception(),))
      return



  def image_callback(self, msg):
    self.image = msg

  def curr_joint_state(self, msg):
    self.robot_state.joint_state = msg

def main(args=None):
  # Initialize rclpy
  rclpy.init(args=args)

  # Create service
  drawbot_service = DrawbotService()

  # Spin the service
  rclpy.spin(drawbot_service)

  # Shutdown rclpy
  rclpy.shutdown()
