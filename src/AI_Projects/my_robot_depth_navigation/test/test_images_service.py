import rclpy
import pytest

from my_robot_depth_navigation.services.images_service import *
from sensor_msgs.msg import Image, CompressedImage

class TestImagesService:
    def test_ros_to_cv(self):
        """
        Test conversion from ROS Image to OpenCV image.
        """
        # Creates a mock Image data
        dummy_img = np.zeros((2, 2, 3), dtype=np.uint8)
        image = Image()
        image.height = dummy_img.shape[0]
        image.width = dummy_img.shape[1]
        image.encoding = "rgb8"  # obligatorio
        image.step = dummy_img.shape[1] * 3  # ancho * número de canales
        image.data = dummy_img.tobytes()

        cv_image = ros_to_cv(image)

        # Validation
        assert isinstance(cv_image, np.ndarray)
        assert cv_image.shape == (2, 2, 3)
        assert np.array_equal(cv_image, dummy_img)

    def test_cv_to_ros(self):
        """
        Test conversion from OpenCV image to ROS Image.
        """
        # Creates a mock OpenCV image
        cv_image = np.zeros((100, 100, 3), dtype=np.uint8)
        ros_image = cv_to_ros(cv_image)
        
        assert isinstance(ros_image, Image)
    
    def test_compressed_to_cv(self):
        """
        Test conversion from CompressedImage to OpenCV image.
        """
        # Creates a mock CompressedImage data
        compressed_image = CompressedImage()
        dummy_img = np.zeros((2, 2, 3), dtype=np.uint8)
        _, img_encoded = cv2.imencode('.png', dummy_img)
        compressed_image.data = img_encoded.tobytes()

        cv_image = compressed_to_cv(compressed_image)

        # Validation
        assert isinstance(cv_image, np.ndarray)
        assert cv_image.shape == (2, 2, 3)
        assert np.array_equal(cv_image, dummy_img)