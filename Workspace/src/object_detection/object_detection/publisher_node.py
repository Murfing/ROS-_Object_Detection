import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FinalArrayPublisher(Node):
    def __init__(self):
        super().__init__('final_array_publisher')
        self.camera_coordinates_subscriber = self.create_subscription(
            Float32MultiArray,
            'CameraCoordinates',
            self.camera_callback,
            10)
        self.random_subscriber = self.create_subscription(
            Float32MultiArray,
            'Random',
            self.random_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'FinalArray', 10)
        self.camera_coordinates = None
        self.random_values = None
        self.get_logger().info('Final array publisher node has been started.')

    def camera_callback(self, msg):
        self.camera_coordinates = msg.data
        self.publish_final_array()

    def random_callback(self, msg):
        self.random_values = msg.data
        self.publish_final_array()

    def publish_final_array(self):
        if self.camera_coordinates is not None and self.random_values is not None:
            final_array_msg = Float32MultiArray()
            final_array_msg.data = [
                self.camera_coordinates[0],
                self.camera_coordinates[1],
                self.random_values[0],
                self.random_values[1]
            ]
            self.publisher_.publish(final_array_msg)
            self.get_logger().info(f'Published FinalArray: {final_array_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = FinalArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
