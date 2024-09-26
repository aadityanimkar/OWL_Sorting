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
            '/camera/image_raw',
            self.image_callback,
            30
        )
        self.publisher = self.create_publisher(Image, '/image_processed', 30)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.detect_and_draw_objects(cv_image)

    def detect_and_draw_objects(self, cv_image):
        # Flip the image vertically
        cv_image = cv2.flip(cv_image, 0)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected objects
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Draw the bounding box
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        processed_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher.publish(processed_image_msg)
        cv2.imshow("Video Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()