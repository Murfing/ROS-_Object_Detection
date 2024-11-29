import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class TennisBallDetector(Node):

    def __init__(self):
        super().__init__('tennis_ball_detector')
        self.subscription = self.create_subscription(Image,'image_raw',self.image_callback,10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'CameraCoordinates', 10)
        self.br = CvBridge()
        self.get_logger().info('Tennis ball detector node has been started.')

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg, 'bgr8')
        # Detect the tennis ball
        center = self.detect_ball(cv_image)
        if center:
            # Create and publish the coordinates message
            coordinates_msg = Float32MultiArray()
            coordinates_msg.data = [float(center[0]), float(center[1])]
            self.publisher_.publish(coordinates_msg)
            self.get_logger().info(f'Published ball center coordinates: {coordinates_msg.data}')

    def detect_ball(self, image):
        # Convert image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range of tennis ball color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:  # Only consider the contour if it is large enough
                return (int(x), int(y))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = TennisBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
