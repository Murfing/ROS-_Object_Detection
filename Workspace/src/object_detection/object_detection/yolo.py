import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from yolov5 import YOLOv5

class YOLOv5Detector(Node):
    def __init__(self):
        super().__init__('yolov5_detector')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ball_center', 10)
        self.br = CvBridge()
        self.get_logger().info('YOLOv5 detector node has been started.')

        # Load YOLOv5 model
        self.model = YOLOv5("yolov5s.pt")  # yolov5s.pt is a small version of YOLOv5

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        cv_image = self.br.imgmsg_to_cv2(msg, 'bgr8')
        center = self.detect_ball(cv_image)
        if center:
            coordinates_msg = Float32MultiArray()
            coordinates_msg.data = [float(center[0]), float(center[1])]
            self.publisher_.publish(coordinates_msg)
            self.get_logger().info(f'Published ball center coordinates: {coordinates_msg.data}')

    def detect_ball(self, image):
        results = self.model(image)
        detections = results.xyxy[0].cpu().numpy()

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            if cls == 0:
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                return int(x_center), int(y_center)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv5Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
