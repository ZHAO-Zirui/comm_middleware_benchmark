import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert ROS Image message to NumPy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        
        # Display image
        # cv2.imshow('Received Image', image)
        # cv2.waitKey(1)  # Refresh window
        
        # Get marker
        marker = time.time()
        print(marker-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()