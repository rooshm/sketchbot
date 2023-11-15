import cv2
import numpy as np
from rembg import remove

from linedraw.linedraw import sketch, makesvg, draw_lines

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion


class CVPipelineService(Node):
  def __init__(self):
    super().__init__('cv_pipeline_service')

    # Load the face cascade
    self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Create service
    self.srv = self.create_service(Img2Svg, '/img2svg', self.img2svg_callback)


  def img2svg_callback(self, request, response):
    # Get CV2 image from ROS 2 message
    image = np.frombuffer(request.image.data, dtype=np.uint8).reshape(request.image.height, request.image.width, -1)

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
    cv2.rectangle(draw_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Increase box sizes by a percentage
    padding = 0.35

    x = max(int(x - (w * padding)), 0)
    y = max(int(y - (h * padding)), 0)
    w = min(int(w + (w * padding * 2)), image.shape[1] - x)
    h = min(int(h + (h * padding * 2)), image.shape[0] - y)

    # Make sure the box is square
    w, h = max(w, h), max(w, h)

    cv2.rectangle(draw_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Crop image to face
    cropped_image = cropped_image[y:y+h, x:x+w]

    # Resize image to 480x480
    cropped_image = cv2.resize(cropped_image, (1200, 1200))

    # Remove background
    processed_image = remove(cropped_image)

    # Vectorize the image
    lines = sketch(processed_image, draw_hatch=False, contour_simplify=1)

    # Remove lines that are too short
    length_threshold = 35
    lines = [line for line in lines if sum([((line[i][0] - line[i - 1][0]) ** 2 + (line[i][1] - line[i - 1][1]) ** 2) ** 0.5 for i in range(1, len(line))]) > length_threshold]

    # Display window with vectorized image
    draw_lines(lines)

    # Create SVG from lines
    svg = makesvg(lines)

    # Save SVG file
    f = open('../data/out.svg','w')
    f.write(svg)
    f.close()

    # # Display the image
    # cv2.imshow('Image', draw_image)
    # cv2.imshow('Cropped Image', cropped_image)
    # cv2.imshow('Processed Image', processed_image)

    # # Save to files
    # cv2.imwrite('./draw_image.jpg', draw_image)
    # cv2.imwrite('./cropped_image.jpg', cropped_image)
    # cv2.imwrite('./processed_image.jpg', processed_image)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    response.lines = lines

    return response
