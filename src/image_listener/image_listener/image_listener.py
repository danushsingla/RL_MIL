# The rest of the ros package still needs to be committed
# but I am unsure which files are from the colcon build or not
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.subscription = self.create_subscription(
            Image,
            '/front_cam/image', #topic for sub9 cam
            self.image_callback,
            10
        )
        self.subscription
        
    def image_callback(self, msg):
        try:
            print(msg)
        except Exception as e:
            print(e)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
