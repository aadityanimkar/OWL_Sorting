import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.subscription = self.create_subscription(
            Image,
            '/owr_6.5/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        processed_image = self.process_image(cv_image)
        # Here you can publish the processed image or bounding box information

    def process_image(self, image):
        # Convert image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define color ranges and apply masks
        lower_color = (36, 25, 25)
        upper_color = (70, 255, 255)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        # Find contours and draw bounding boxes
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()