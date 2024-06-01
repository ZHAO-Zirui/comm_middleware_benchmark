import rclpy
import core
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.frame_id = 0
        self.bridge = CvBridge()

    def publish_image(self):
        # Generate image
        image = core.generate_random_image(1920, 1080)
        timestamp = core.generate_timestamp()
        
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
        msg.data.frombytes(image.tobytes())
        self.publisher_.publish(msg)        
        self.frame_id += 1


def main():
    frequency = 30  # 30Hz
    log_send_count = 0
    log_last_time = time.time()
    next_time = time.perf_counter()
    
    rclpy.init()
    image_publisher = ImagePublisher()
    
    try:
        while rclpy.ok():
            current_time = time.perf_counter()
            if current_time < next_time:
                time.sleep(next_time - current_time)
        
            image_publisher.publish_image()
            log_send_count += 1
            next_time += + 1/frequency
            
            # print log
            if time.time() - log_last_time >= 1:
                image_publisher.get_logger().info(f'Published {log_count} frames in the last second')
                log_last_time = time.time()
                log_count = 0
    except KeyboardInterrupt:
        print("Goodbye!")
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()