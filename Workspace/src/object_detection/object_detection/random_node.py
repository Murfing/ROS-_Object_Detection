import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class IntegerArrayPublisher(Node):
    def __init__(self):
        super().__init__('integer_array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'integer_array', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.get_logger().info('Integer array publisher node has been started.')

    def timer_callback(self):
        array_msg = Float32MultiArray()
        array_msg.data = [random.randint(1, 100), random.randint(1, 100)]
        self.publisher_.publish(array_msg)
        self.get_logger().info(f'Published: {array_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = IntegerArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
