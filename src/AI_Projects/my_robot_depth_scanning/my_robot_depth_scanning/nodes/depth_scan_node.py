class DepthScanNode(Node):
    def __init__(self):
        super().__init__('depth_scan_node')
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.depth_image_callback,
            10
        )