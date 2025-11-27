"""
This module contains a node that reada message from a Limo's
depth camera and publish an image with all the objects
position, prediction and depth
"""

# YOLO service imports
from typing import Optional
import rclpy
from my_robot_ai_identification.real_time_prediction.yolo_service import YoloService
from my_robot_ai_identification.real_time_prediction.image_processor import ImageProcessor

# ROS2 core imports
from rclpy.node import Node
from rclpy.logging import get_logger

# ROS2 image processing imports
from sensor_msgs.msg import Image

# Custom message import
from custom_msgs.msg import Yolov8Inference, InferenceResult

class YoloPredictionNode(Node):

    def __init__(self, yolo_service: YoloService) -> None:
        """
        Initializes the YoloPredictionNode. Both subscriber to the depth camera topic
        and the publishers for the predictions and the predicted image are created here.

        Parameters:
        ----------
        node_name : str
            The name of the node.
        yolo_service : YoloService
            The YOLO service instance to be used for predictions.
        """

        super().__init__("yolo_prediction_node")

        self.yolo = yolo_service
        self.image_processor = ImageProcessor()

        # Node subscriptions
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.raw_image_callback,
            10
        )

        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.depth_image_callback,
            10
        )

        # Node publishers
        self.prediction_publisher = self.create_publisher(
            Yolov8Inference,
            "/yolo/predictions",

            10
        )

        # Computation variables
        self.raw_image = None
        self.depth_image = None

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

    
    def depth_image_callback(self, msg: Image) -> None:
        """
        Callback for the depth image topic. It just saves the image to avoid
        interruption blocks.

        Parameters:
        ----------
        msg : Image
            The depth image message from the topic.
        """
        self.depth_image = msg


    def run(self) -> None:
        print("Running YOLO prediction...")
        raw_img = self.image_processor.ros_to_cv(self.raw_image) if self.raw_image else None
        depth_img = self.image_processor.ros_to_cv(self.depth_image, encoding='32FC1') if self.depth_image else None

        results = self.get_predictions(raw_img)

        # Early return if no results
        if not results:
            return
        
        self.analyze_detections(results, depth_img)
        

    def get_predictions(self, img: Optional[Image]) -> None:

        # Early exit if no images are read
        if img is None:
            return

        return self.yolo.predict(img)

    
    def analyze_detections(self, detections: list, depth_image: Optional[Image]) -> None:
        # Create YOLO inference image message
        inference_image = Yolov8Inference()
        inference_image.header.frame_id = "inference"
        inference_image.header.stamp = self.get_clock().now().to_msg()

        print(type(depth_image))

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