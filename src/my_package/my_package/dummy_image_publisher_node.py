import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class DummyImagePublisher(Node):
    def __init__(self):
        super().__init__('dummy_image_publisher')
        
        # Set up publisher
        self.image_publisher = self.create_publisher(Image, '/zed2i/zed_node/rgb/image_rect_color', 10)
        self.timer = self.create_timer(1.0, self.publish_image)  # Publish image every second
        
        # Path to the test image
        self.image_path = '/home/kunj/Downloads/Pallets_Resized-20241109T220408Z-001/Pallets_Resized/543202-7201_jpg.rf.06dd4762d69c0fdf739b25d3d551a04a.jpg'  # Replace with your actual path
        self.bridge = CvBridge()

    def publish_image(self):
        # Load image
        if not os.path.exists(self.image_path):
            self.get_logger().error(f"Image path '{self.image_path}' does not exist.")
            return
        cv_image = cv2.imread(self.image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image '{self.image_path}'.")
            return

        # Convert OpenCV image to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_publisher.publish(image_msg)
        self.get_logger().info("Published dummy image.")

def main(args=None):
    rclpy.init(args=args)
    dummy_publisher = DummyImagePublisher()
    rclpy.spin(dummy_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
