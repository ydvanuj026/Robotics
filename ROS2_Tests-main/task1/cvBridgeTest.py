import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthImageConverter(Node):
    def __init__(self):
        super().__init__('depth_image_converter')
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the depth image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',  # Change to the depth topic you're using
            self.depth_callback,
            10
        )

    def depth_callback(self, msg):
        try:
            # Convert ROS 2 Depth Image message to OpenCV image
            # Adjust encoding based on your sensor's depth image encoding
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Normalize the depth image for display purposes (convert to 8-bit image)
            depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_display)

            # Display the depth image
            cv2.imshow("Depth Image Window", depth_display)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the depth image converter node
    depth_image_converter = DepthImageConverter()
    
    # Keep the node running
    rclpy.spin(depth_image_converter)
    
    # Shutdown and release resources
    depth_image_converter.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
