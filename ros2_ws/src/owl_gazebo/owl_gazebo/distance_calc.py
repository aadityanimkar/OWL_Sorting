import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class DistanceCalculationNode(Node):
    def __init__(self):
        super().__init__('distance_calculation_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_processed',
            self.image_callback,
            30
        )
        self.publisher = self.create_publisher(Float32, '/object_distance', 30)
        self.cv_bridge = CvBridge()
        self.focal_length = 500  
        self.known_width = 0.1 
        
    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.calculate_distance(cv_image)

    def calculate_distance(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            distance = (self.known_width * self.focal_length) / w
            print(f"Distance to object: {distance} meters")

            distance_msg = Float32()
            distance_msg.data = distance
            self.publisher.publish(distance_msg)
            print("Distance published")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceCalculationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()