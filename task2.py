from picamera2 import Picamera2
import cv2
import numpy as np
import time
# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(2)
# Load ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
print("Starting ArUco detection (CTRL+C to stop)...")
try:
while True:
frame = picam2.capture_array()
# TODO 1: use cv2 to grayscale the frame
# gray = ...
corners, ids, rejected = detector.detectMarkers(gray)
if ids is not None:
for i in range(len(ids)):
marker_id = ids[i][0]
# TODO 2:Compute marker center from corners
# center_x = ...
# center_y = ...
print(f"Detected ID: {marker_id} | Center: ({center_x},
{center_y})")
time.sleep(0.1)
except KeyboardInterrupt:
cv2.imwrite("task2.jpg", frame)
print("Stopping...")
finally:
picam2.stop()
