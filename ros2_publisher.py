import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2
import core
import time

frequency = 30  # 30Hz


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.frame_id = 0

    def publish_image(self):
        # Generate image
        image = core.generate_random_image(1920, 1080)
        timestamp = core.generate_timestamp()
        
        t1 = time.time()
        image_hex = image.tobytes()
        t2 = time.time()
        print(f"ATime: {t2-t1}")
        
        # Create Image message
        msg = Image()
        msg.header = Header()
        msg.header.stamp.sec = timestamp[0]
        msg.header.stamp.nanosec = timestamp[1]
        msg.header.frame_id = str(self.frame_id)
        
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = image.strides[0]
        t1 = time.time()
        msg.data = image_hex
        t2 = time.time()
        print(f"BTime: {t2-t1}")


        self.publisher_.publish(msg)
        
        self.frame_id += 1
        self.get_logger().info(f'Published frame {self.frame_id}')


rclpy.init()
image_publisher = ImagePublisher()
time_last_frame = time.time()
try:
    while rclpy.ok():
        if time.time() - time_last_frame < (1 / frequency):
            continue
        image_publisher.publish_image()

        
        time_last_frame = time.time()
except KeyboardInterrupt:
    pass
finally:
    image_publisher.destroy_node()
    rclpy.shutdown()