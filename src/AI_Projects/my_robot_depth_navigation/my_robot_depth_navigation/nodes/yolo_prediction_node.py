"""
This module contains a node that reads messages from a Limo's
depth camera and publish an image with all the predictions and a message
with the objects, their positions and depth.
"""

# YOLO service imports
import rclpy
from my_robot_depth_scanning.services.images_service import *
from ultralytics import YOLO

# ROS2 core imports
from rclpy.node import Node
from rclpy.logging import get_logger

# ROS2 image processing imports
from sensor_msgs.msg import Image

# Custom message import
from custom_msgs.msg import InferenceData


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
            f'{YOLO_PREDICTIONS_TOPIC}/predictions_image',
            10
        )

        # Publish custom coordinates messages to predictions/coordinates topic
        self.coordinates_publisher = self.create_publisher(
            InferenceData,
            f'{YOLO_PREDICTIONS_TOPIC}/predictions_data',
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
        raw_image = ros_to_cv(self.raw_image)

        # Get predictions
        predictions = self.get_predictions(raw_img)

        # Early return if no results
        if not predictions:
            return

        # If predictions publish the image and their coordinates
        self.publish_detections(raw_image.copy(), predictions[0]) # Just one prediction is performed as we send just one image at a time


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

    
    def publish_detections(self, raw_image: np.ndarray, prediction: list) -> None:
            # Overlay the detected object's name and confidence level
            # object_name = results.hypothesis.class_id
            # confidence = results.hypothesis.score * 100  # Confidence in percentage
            # label = f"{object_name}: {confidence:.2f}%"

            # Put the label above the bounding box
            # cv2.putText(overlay_image, label, (x_min, y_min - 10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        inference_data = InferenceData()

        # We assume one detection per image
        signal = prediction.boxes[0]
        inference_data.class_name = signal.cls
        signal_xywh = signal.xywh[0].to('cpu').detach().numpy().copy()
        signal_xyxy = signal.xyxy[0].to('cpu').detach().numpy().copy()

        centroid = (signal_xyxy[0] + (signal_xywh[2] / 2), signal_xyxy[1] + (signal_xywh[3] / 2))

        # Print the bounding box
        cv2.rectangle(raw_image, (int(signal_xyxy[0]), int(signal_xyxy[1])), (int(signal_xyxy[2]), int(signal_xyxy[3])), (0, 255, 0), 2)

        # Add depth information if available
        if depth_image:
            depth_value = depth_image[int(centroid[1]), int(centroid[0])]
            inference_data.depth = depth_value

        # Draw the bounding box
        prediction_image = self.image_processor.cv_to_ros(raw_image, "bgr8")
        self.prediction_publisher.publish(prediction_image)
        self.coordinates_publisher.publish(inference_data)


def main() -> None:
    # Initialize ROS2
    rclpy.init()

    # Create YOLO service and node
    model = '/home/johnnyastudillo/Desktop/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt'
    #yolo = YoloService(model)
    node = YoloPredictionNode(model)

    # Spin the node
    rclpy.spin(node)

    # Shutdown ROS2 when done
    node.destroy_node()
    rclpy.shutdown()