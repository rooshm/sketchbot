import cv2
import numpy as np
import json
from rembg import remove

from .linedraw.linedraw import sketch, makesvg, draw_lines

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion

from sketchbot_interfaces.srv import Img2Svg

class CVPipelineService(Node):
  def __init__(self):
    super().__init__('cv_pipeline_service')

    # Load the face cascade
    self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Create service
    self.srv = self.create_service(Img2Svg, '/img2svg', self.img2svg_callback)

    # Declare parameters
    self.declare_parameter('padding', 0.35)
    self.declare_parameter('contrast', 1.5)
    self.declare_parameter('brightness', -64)


  def adjust_contrast_brightness(self, img, contrast=1.0, brightness=0):
    """
    Adjusts contrast and brightness of an uint8 image.
    contrast:   (0.0,  inf) with 1.0 leaving the contrast as is
    brightness: [-255, 255] with 0 leaving the brightness as is
    """

    brightness += int(round(255*(1-contrast)/2))
    return cv2.addWeighted(img, contrast, img, 0, brightness)


  def img2svg_callback(self, request, response):
    # Convert sensor_msgs/Image to numpy array
    image = np.frombuffer(request.image.data, dtype=np.uint8).reshape(request.image.height, request.image.width, -1)

    resolution = request.resolution if request.resolution > 0 else 1024
    length_threshold = request.length_threshold if request.length_threshold > 0 else 32

    # From ROS parameters
    padding = self.get_parameter('padding').get_parameter_value().double_value
    contrast = self.get_parameter('contrast').get_parameter_value().double_value
    brightness = self.get_parameter('brightness').get_parameter_value().integer_value

    cropped_image = image.copy()

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = self.face_cascade.detectMultiScale(gray)

    if len(faces) > 0:
      print(f'Found {len(faces)} faces')
    else:
      print('No faces found')
      return

    # Get the largest face
    x, y, w, h = sorted(faces, key=lambda x: x[2] * x[3], reverse=True)[0]

    # Draw a rectangle around the face
    # cv2.rectangle(draw_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    x = max(int(x - (w * padding)), 0)
    y = max(int(y - (h * padding)), 0)
    w = min(int(w + (w * padding * 2)), image.shape[1] - x)
    h = min(int(h + (h * padding * 2)), image.shape[0] - y)

    # Make sure the box is square
    w, h = max(w, h), max(w, h)

    # cv2.rectangle(draw_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Crop image to face
    cropped_image = cropped_image[y:y+h, x:x+w]

    # Remove background
    processed_image = remove(cropped_image)

    # Increase contrast and darken image
    processed_image = self.adjust_contrast_brightness(processed_image, contrast=contrast, brightness=brightness)

    # Vectorize the image
    lines = sketch(processed_image, draw_hatch=False, contour_simplify=2, resolution=resolution)

    # Remove lines that are too short
    lines = [line for line in lines if sum([((line[i][0] - line[i - 1][0]) ** 2 + (line[i][1] - line[i - 1][1]) ** 2) ** 0.5 for i in range(1, len(line))]) > length_threshold]

    # Display window with vectorized image
    draw_lines(lines)

    # Create SVG from lines
    svg = makesvg(lines)

    # Save SVG file
    f = open('src/sketchbot/data/out.svg','w')
    f.write(svg)
    f.close()

    # # Display the image
    # cv2.imshow('Image', draw_image)
    # cv2.imshow('Cropped Image', cropped_image)
    # cv2.imshow('Processed Image', processed_image)

    # # Save to files
    # cv2.imwrite('./draw_image.jpg', draw_image)
    # cv2.imwrite('./cropped_image.jpg', cropped_image)
    cv2.imwrite('src/sketchbot/data/processed_image.jpg', processed_image)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    response.lines_json = json.dumps(lines)

    return response


def main(args=None):
  # Initialize rclpy
  rclpy.init(args=args)

  # Create service
  cv_pipeline_service = CVPipelineService()

  # Spin the service
  rclpy.spin(cv_pipeline_service)

  # Shutdown rclpy
  rclpy.shutdown()
