import os
import pytest
import numpy as np
import cv2
import rclpy

from pathlib import Path
from sensor_msgs.msg import Image, CompressedImage
from custom_msgs.msg import InferenceData

from my_robot_depth_navigation.nodes.yolo_prediction_node import YoloPredictionNode
from my_robot_depth_navigation.services.images_service import cv_to_ros

from unittest.mock import Mock

# ---------------------------------------------------------------------
# ROS context
# ---------------------------------------------------------------------

@pytest.fixture(scope="session", autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ---------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------

@pytest.fixture(scope="session")
def test_image_path():
    path = Path(__file__).parent / "stop_signal.png"
    assert path.exists(), f"Test image not found: {path}"
    return path


# ---------------------------------------------------------------------
# Node (YOLO REAL)
# ---------------------------------------------------------------------

@pytest.fixture
def node():
    node = YoloPredictionNode()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------
# Helper: crear CompressedImage desde archivo real
# ---------------------------------------------------------------------

def compressed_image_from_file(path: Path) -> CompressedImage:
    img = cv2.imread(str(path))
    assert img is not None, "Failed to load test image"

    _, encoded = cv2.imencode(".jpg", img)

    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


# ---------------------------------------------------------------------
# Helper: depth image dummy (misma resolución)
# ---------------------------------------------------------------------

def dummy_depth_image(height: int, width: int) -> Image:
    depth = np.ones((height, width), dtype=np.float32)

    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = "32FC1"
    msg.step = width * 4
    msg.data = depth.tobytes()
    return msg


# ---------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------

def test_yolo_prediction_with_real_image(node, test_image_path):
    """
    Integration test:
    - Real image
    - Real YOLO model
    - Check that predictions are produced and published
    """

    # -----------------------------------------------------------------
    # Prepare input messages
    # -----------------------------------------------------------------

    color_msg = compressed_image_from_file(test_image_path)

    img = cv2.imread(str(test_image_path))
    h, w, _ = img.shape
    depth_msg = dummy_depth_image(h, w)

    # Feed callbacks
    node.color_image_callback(color_msg)
    node.depth_image_callback(depth_msg)

    # -----------------------------------------------------------------
    # Spy publishers
    # -----------------------------------------------------------------

    node.prediction_publisher.publish = Mock()
    node.coordinates_publisher.publish = Mock()

    # -----------------------------------------------------------------
    # Run node logic
    # -----------------------------------------------------------------

    node.run()

    # -----------------------------------------------------------------
    # Assertions
    # -----------------------------------------------------------------

    # Image with predictions published
    assert node.prediction_publisher.publish.called, \
        "Prediction image was not published"

    # InferenceData published
    assert node.coordinates_publisher.publish.called, \
        "InferenceData was not published"

    inference_msg = node.coordinates_publisher.publish.call_args[0][0]

    assert isinstance(inference_msg, InferenceData)
    assert inference_msg.class_name != ""
    assert inference_msg.depth > 0.0
