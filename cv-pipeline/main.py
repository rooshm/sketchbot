import cv2
import numpy as np
from rembg import remove

from linedraw.linedraw import sketch, makesvg, draw_lines

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def convert_to_svg(image, resolution=1024):
  draw_image = image.copy()
  cropped_image = image.copy()

  # Convert the image to grayscale
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  # Detect faces in the image
  faces = face_cascade.detectMultiScale(gray)

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

  w, h = max(w, h), max(w, h)

  cv2.rectangle(draw_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

  # Crop image to face
  cropped_image = cropped_image[y:y+h, x:x+w]

  # Resize image to 480x480
  cropped_image = cv2.resize(cropped_image, (1200, 1200))

  # Remove background
  processed_image = remove(cropped_image)

  lines = sketch(processed_image, draw_hatch=False, contour_simplify=1)

  length_threshold = 35

  # Lines are made up of a list of (x, y) points. Remove lines where sum of distances between all points is less than the threshold.
  lines = [line for line in lines if sum([((line[i][0] - line[i - 1][0]) ** 2 + (line[i][1] - line[i - 1][1]) ** 2) ** 0.5 for i in range(1, len(line))]) > length_threshold]

  draw_lines(lines)

  # print(lines)

  svg = makesvg(lines)

  # save svg
  f = open('./image.svg','w')
  f.write(svg)
  f.close()

  # return

  # Display the image
  cv2.imshow('Image', draw_image)
  cv2.imshow('Cropped Image', cropped_image)
  cv2.imshow('Processed Image', processed_image)

  # Save to files
  cv2.imwrite('./draw_image.jpg', draw_image)
  cv2.imwrite('./cropped_image.jpg', cropped_image)
  cv2.imwrite('./processed_image.jpg', processed_image)

  cv2.waitKey(0)
  cv2.destroyAllWindows()

  return lines


if __name__ == '__main__':
  # Read from webcam
  cap = cv2.VideoCapture(0)
  ret, image = cap.read()
  cap.release()

  # image = cv2.imread('./images/face.jpg')

  convert_to_svg(image, resolution=512)
