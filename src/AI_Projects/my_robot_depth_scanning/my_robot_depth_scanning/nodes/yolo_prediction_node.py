"""
This module contains a node that reads messages from a Limo's
depth camera and publish an image with all the objects
position, prediction and depth
"""

# YOLO service imports
from typing import Optional
import rclpy
from my_robot_ai_identification.real_time_prediction.yolo_service import YoloService
from services.images_service import *

# ROS2 core imports
from rclpy.node import Node
from rclpy.logging import get_logger

# ROS2 image processing imports
from sensor_msgs.msg import Image

# Custom message import
from custom_msgs.msg import Yolov8Inference, InferenceResult


YOLO_PREDICTIONS_TOPIC = "/yolo/predictions"


class YoloPredictionNode(Node):

    def __init__(self, model_path: str) -> None:
        """
        This node is responsible for reading the image messages from the depth camera
        publish the an image with the predictions, and a message with their x, y-coordinates.

        Parameters:
        ----------
        model_path : str
            The path to the YOLO model to be used for predictions.
        """

         # Sets node's name
        super().__init__("yolo_prediction_node")

        # Declare node topics as parameters
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.yolo = YOLO(model_path)

        # Subscribe node to raw image topic
        self.create_subscription(
            Image,
            camera_topic,
            self.raw_image_callback,
            10
        )

        # Publish images with predictions to yolo predictions topic
        self.prediction_publisher = self.create_publisher(
            Image,
            f'{YOLO_PREDICTIONS_TOPIC}/prediction_image',
            10
        )

        # Publish custom coordinates messages to predictions/coordinates topic
        self.coordinates_publisher = self.create_publisher(
            InferenceCoordinates,
            f'{YOLO_PREDICTIONS_TOPIC}/coordinates',
            10
        )

        # Computation variables
        self.raw_image = None

        # Main loop
        self.create_timer(0.1, self.run)  # Run at 10 Hz


    def raw_image_callback(self, msg: Image) -> None:
        """
        Callback for the raw image topic. It just saves the image to avoid
        interruption blocks.

        Parameters:
        ----------
        msg : Image
            The raw image message from the topic.
        """
        self.raw_image = msg


    def run(self) -> None:
        """
        Main loop responsible for performing the predictions.
        """

        # Early return if no images are read
        if not self.raw_image:
            return

        # Convert ROS image to OpenCV image
        raw_img = ros_to_cv(self.raw_image)

        # Get predictions
        predictions = self.get_predictions(raw_img)

        # Early return if no results
        if not predictions:
            return

        # If predictions publish the image and their coordinates
        self.publish_predictions(predictions)


    def get_predictions(self, img: Image) -> list:
        """
        Returns the predictions made by the YOLO model.

        Parameters:
        ----------
        img : Image
            The image to be predicted.

        Returns:
        -------
        list
            The predictions made by the YOLO model.
        """
        return self.yolo(img)

    
    def publish_predictions(self, detections: list) -> None:

        print(type(detections))

        #for detection in detections.detections:

        for d in detections:
            for b in d.boxes:
                # Extract prediction data
                prediction_class = int(b.cls)
                prediction_box = b.xyxy[0].to('cpu').detach().numpy().copy()

                # Create inference result message
                prediction = InferenceResult()
                prediction.class_name = self.yolo.class_names[prediction_class]
                #prediction.class_name = prediction_class
                prediction.left = int(prediction_box[0])
                prediction.top = int(prediction_box[1])
                prediction.right = int(prediction_box[2])
                prediction.bottom = int(prediction_box[3])
                prediction.box_width = (prediction.right - prediction.left) 
                prediction.box_height = (prediction.bottom - prediction.top)
                prediction.x = prediction.left + (prediction.box_width / 2.0)
                prediction.y = prediction.top + (prediction.box_height / 2.0)

                # Add depth information if available
                if not depth_image is None:
                    # Calculate the centroid of the bounding box
                    centroid_x = int(prediction.x)
                    centroid_y = int(prediction.y)

                    depth_value = depth_image[centroid_y, centroid_x]

                    prediction.depth = float(depth_value)
                    print(prediction.depth)

                inference_image.yolov8_inference.append(prediction)
        
        prediction_image = detections[0].plot()
        prediction_image = self.image_processor.cv_to_ros(prediction_image, "bgr8")
        self.image_publisher.publish(prediction_image)
        self.prediction_publisher.publish(inference_image)


def main() -> None:
    # Initialize ROS2
    rclpy.init()

    # Create YOLO service and node
    model = '/home/johnnyastudillo/Desktop/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt'
    yolo = YoloService(model)
    node = YoloPredictionNode(yolo)

    # Spin the node
    rclpy.spin(node)

    # Shutdown ROS2 when done
    node.destroy_node()
    rclpy.shutdown()