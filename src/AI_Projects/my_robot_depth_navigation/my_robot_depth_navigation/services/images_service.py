import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import struct

bridge = CvBridge()

def ros_to_cv(source: Image, encoding: str = "bgr8") -> np.ndarray:
    """
    Converts a ROS Image message to an OpenCV image (numpy array)
    
    Parameters:
    ----------
    source : Image
        The ROS Image message to convert.
    encoding : str, optional
        The desired encoding of the OpenCV image (default is "bgr8").

    Returns:
    -------
    np.ndarray
        The converted OpenCV image.
    """
    return bridge.imgmsg_to_cv2(source, encoding)
    

def cv_to_ros(source: np.ndarray, encoding: str = "bgr8") -> Image:
    """
    Converts an OpenCV image (numpy array) to a ROS Image message.
    
    Parameters:
    ----------
    source : np.ndarray
        The OpenCV image to convert.
    encoding : str, optional
        The desired encoding of the ROS Image message (default is "bgr8").
    
    Returns:
    -------
    Image
        The converted ROS Image message.
    """
    return bridge.cv2_to_imgmsg(source, encoding)

def compressed_to_cv(source: CompressedImage, encoding: int = cv2.IMREAD_COLOR):
    """
    Docstring for compressed_to_cv
    
    :param source: Description
    :type source: CompressedImage
    :param encoding: Description
    :type encoding: int
    """

    np_arr = np.frombuffer(source.data, np.uint8)
    return cv2.imdecode(np_arr, encoding)


def compressed_depth_to_cv(msg: CompressedImage):
    """
    Docstring for compressed_depth_to_cv
    
    :param msg: Description
    :type msg: CompressedImage
    """
    try:
        data = bytes(msg.data)

        # # Leer longitud del string de formato
        # fmt_len = struct.unpack_from('I', data, 0)[0]
        # offset = 4

        # # Leer string (normalmente "png")
        # fmt = data[offset:offset + fmt_len].decode('utf-8')
        # offset += fmt_len

        # El resto es PNG
        png_data = data[12:]

        # Decodificar PNG sin convertir a 8 bits
        depth_img = cv2.imdecode(
            np.frombuffer(png_data, np.uint8),
            cv2.IMREAD_UNCHANGED
        )

        return depth_img

    except Exception as e:
        print(f"[compressedDepth] Error: {e}")
        return None


def resize_image(source: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    Resizes an OpenCV image to the specified width and height.

    Parameters:
    ----------
    source : np.ndarray
        The OpenCV image to resize.
    width : int
        The desired width of the resized image.
    height : int
        The desired height of the resized image.

    Returns:
    -------
    np.ndarray
        The resized OpenCV image.
    """
    return cv2.resize(source, (width, height))
