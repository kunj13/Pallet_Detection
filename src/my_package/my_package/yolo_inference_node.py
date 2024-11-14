import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from pathlib import Path
import cv2
from yolov5 import YOLO  # Assuming you have the YOLO package properly set up

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # ROS publishers and subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/processed/annotated_image', self.image_callback, 10)
        self.label_subscriber = self.create_subscription(
            String, '/processed/yolo_labels', self.label_callback, 10)
        self.detection_publisher = self.create_publisher(String, '/processed/yolo_detections', 10)
        self.annotated_image_publisher = self.create_publisher(Image, '/processed/yolo_annotated_image', 10)
        
        # YOLO model setup
        self.bridge = CvBridge()
        weights_path = 'src/yolov5/best.pt'  # Update this path
        self.model = YOLO(weights_path)  # Instantiate your YOLO model
        self.model.conf = 0.13  # Confidence threshold
        self.model.classes = [0, 1]  # Detect classes 'pallet' and 'ground'

        self.current_image = None
        self.current_labels = None

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def label_callback(self, msg):
        # Capture the YOLO label data as a string
        self.current_labels = msg.data
        self.process_detections()

    def process_detections(self):
        if self.current_image is not None and self.current_labels is not None:
            # Run YOLO model inference
            results = self.model(self.current_image)

            # Annotate image with bounding boxes and labels
            annotated_image = self.current_image.copy()
            detections = []
            for det in results.pred[0]:
                x1, y1, x2, y2, conf, cls = det
                label = self.model.names[int(cls)]
                
                # Draw bounding box
                cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Add label and confidence score
                label_text = f"{label} {conf:.2f}"
                cv2.putText(annotated_image, label_text, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Collect detection info for publishing
                detections.append(f"{label} {conf:.2f} {x1:.0f} {y1:.0f} {x2:.0f} {y2:.0f}")

            # Convert annotated image to ROS2 format and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.annotated_image_publisher.publish(annotated_msg)

            # Publish detection results as a string message
            detection_message = "\n".join(detections)
            self.detection_publisher.publish(String(data=detection_message))

            # Reset the current image and labels
            self.current_image = None
            self.current_labels = None

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
