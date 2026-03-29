from ultralytics import YOLO

import os

dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)

model = YOLO('/home/johnnyastudillo/Desktop/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_depth_navigation/models/yolov8n_custom.pt')
print(model.names)