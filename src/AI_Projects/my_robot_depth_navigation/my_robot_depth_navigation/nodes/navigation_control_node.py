# YOLO service imports
import rclpy

# ROS2 core imports
from rclpy.node import Node
from visualization_msgs.msg import Marker

# Custom message import
from custom_msgs.msg import InferenceData

class NavigationControlNode(Node):

    def __init__(self):

        super().__init__("navigation_control_node")

        # ---------------------------------- Parameters Setup ------------------------------------

        self.declare_parameter('distance_threshold', 1.0)
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value

        # ---------------------------------- Predictions Setup -----------------------------------

        self.create_subscription(
            InferenceData,
            '/yolo/predictions/predictions_data',
            self.inference_data_callback,
            10
        )

        self.marker_pub = self.create_publisher(Marker, '/virtual_walls', 10)
        
        self.last_action_time = self.get_clock().now()
        self.is_stopped = False

        self.detection = None

        # Main loop
        self.create_timer(0.1, self.run)  # Run at 10 Hz


    def run(self):
        """
        Main loop responsible for control the robot based on the detected signs.
        """

        detection = self.detection

        if not detection or detection.depth > self.distance_threshold or detection.depth < 0:
            return

        self.handle_signs(detection.class_name, detection.depth)
    

    def inference_data_callback(self, msg: InferenceData):
        """
        Callback function to handle incoming inference data messages.

        Parameters:
        ----------
        msg : InferenceData
            The inference data message containing detected sign information.
        """

        self.detection = msg


    def handle_signs(self, detected_sign: str, distance: float):
        """
        Handles the signs detected by the YOLO service.

        Parameters:
        ----------
        detected_sign : str
            The sign detected by the YOLO service.

        Returns:
        -------
        None
        """

        now = self.get_clock().now()

        if detected_sign == "Stop" and not self.is_stopped:
            self.execute_stop()

        elif detected_sign in ["Derecha", "Izquierda", "Prohibido"]:
            self.create_virtual_wall(detected_sign, distance)

    def execute_stop(self) -> None:
        req = Empty.Request()
        self.pause_client.call_async(req)
        self.is_stopped = True

        self.create_timer(3.0, self.resume_navigation, timer_period_callback=None)

    def resume_navigation(self, timer) -> None:
        req = Empty.Request()
        self.resume_client.call_async(req)
        self.is_stopped = False
        timer.cancel()

def main() -> None:
    # Initialize ROS2
    rclpy.init()

    node = NavigationControlNode()

    # Spin the node
    rclpy.spin(node)

    # Shutdown ROS2 when done
    node.destroy_node()
    rclpy.shutdown()