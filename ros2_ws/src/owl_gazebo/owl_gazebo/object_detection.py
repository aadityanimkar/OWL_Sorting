import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'owr_6_5/camera/image_processed',
            self.image_callback,
            30
        )
        self.publisher = self.create_publisher(Image, 'owr_6_5/camera/image_processed', 10)
        self.cv_bridge = CvBridge()
        # Load the Haar cascade classifier for object detection
        self.cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.detect_and_draw_objects(cv_image)

    def detect_and_draw_objects(self, cv_image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect objects using the Haar cascade classifier
        bounding_boxes = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw bounding boxes on the image
        for (x, y, w, h) in bounding_boxes:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the processed image back to ROS Image message
        processed_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')

        # Publish the processed image
        self.publisher.publish(processed_image_msg)

        # Display the image with bounding boxes (optional for local visualization)
        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()