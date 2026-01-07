# YOLO service imports
import rclpy

# ROS2 core imports
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Custom message import
from custom_msgs.msg import InferenceData

class NavigationControlNode(Node):

    def __init__(self):

        super().__init__("navigation_control_node")

        # ---------------------------------- Parameters Setup ------------------------------------

        self.declare_parameter('distance_threshold', 20.0)
        distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value

        self.distance_threshold = distance_threshold

        # ---------------------------------- Predictions Setup -----------------------------------

        self.create_subscription(
            InferenceData,
            '/yolo/predictions/predictions_data',
            self.inference_data_callback,
            10
        )

        self.detection = None

        # Main loop
        self.create_timer(0.1, self.run)  # Run at 10 Hz


    def run(self):
        """
        Main loop responsible for control the robot based on the detected signs.
        """

        if not self.detection:
            return

        if not self.should_react(self.detection.depth):
            return

        self.handle_signs(self.detection.class_name)
    

    def inference_data_callback(self, msg: InferenceData):
        """
        Callback function to handle incoming inference data messages.

        Parameters:
        ----------
        msg : InferenceData
            The inference data message containing detected sign information.
        """

        self.detection = msg
        

    def should_react(self, depth: float) -> bool:
        """
        Defines whether the robot should react to a sign based on its distance.

        Parameters:
        ----------
        depth : float
            The Z coordinate of the sign relative to the robot.

        Returns:
        -------
        bool
            True if the robot is close enough to the sign to react.
        """

        return depth <= self.distance_threshold


    def handle_signs(self, detected_sign: str):
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

        now = self.get_clock().now().nanoseconds / 1e9

        def T(vx=0.0, vy=0.0, wz=0.0):
            """
            Inner function to create a Twist message.

            Parameters:
            ----------
            vx : float
                The linear velocity in the x direction.
            vy : float
                The linear velocity in the y direction.
            wz : float
                The angular velocity in the z direction.

            Returns:
            -------
            Twist
                The Twist message.
            """
            t = Twist()
            t.linear.x = float(vx)
            t.linear.y = float(vy)
            t.angular.z = float(wz)
            return t

        if detected_sign == "Prohibido":
            self.current_twist = T(0.0, 0.0, 0.0)

        elif detected_sign == "STOP":
            self.current_twist = T(0.0, 0.0, 0.0)

        elif detected_sign == "Ceda":
            self.current_twist = T(0.05, 0.0, 0.0)
            self.action_end_time = now + 3.0

        elif detected_sign == "Derecha":
            wp = self.create_lateral_waypoint("Derecha", side="right")
            if wp:
                self.waypoint_pub.publish(wp)

        elif detected_sign == "Izquierda":
            wp = self.create_lateral_waypoint("Izquierda", side="left")
            if wp:
                self.waypoint_pub.publish(wp)

        if self.current_twist:
            self.cmd_vel_pub.publish(self.current_twist)


def main() -> None:
    # Initialize ROS2
    rclpy.init()

    node = NavigationControlNode()

    # Spin the node
    rclpy.spin(node)

    # Shutdown ROS2 when done
    node.destroy_node()
    rclpy.shutdown()