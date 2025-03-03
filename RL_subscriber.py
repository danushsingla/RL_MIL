# Right now, it only subscribes to /front_cam/image_raw topic
# However, more sensor data and subscribers to be incorporated in the future
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import time

class RL_subscriber(Node):
    def __init__(self):
        super().__init__('RL_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            '/front_cam/image_raw', #topic for sub9 cam
            self.image_callback,
            10
        )
        self.lock = threading.Lock()
        self.image_subscription
        self.front_cam_image = None        

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        with self.lock:
            self.front_cam_image = msg

    def get_image(self):
        with self.lock:
            return self.front_cam_image
        
def main(args=None):
    rclpy.init(args=args)

    # Declare node and spin it
    node = RL_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
