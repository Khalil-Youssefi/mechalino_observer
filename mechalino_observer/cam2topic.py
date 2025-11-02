import rclpy
from rclpy.node import Node
import rclpy.parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
class CameraPublisher(Node):
    def __init__(self,args):
        super().__init__('camera_publisher')
        
        # get parameters
        self.declare_parameter('camera_index', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('image_width', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('image_height', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('camera_topic', rclpy.Parameter.Type.STRING)

        params = self._parameters
        self.camera_index = params['camera_index'].value
        self.image_width = params['image_width'].value
        self.image_height = params['image_height'].value
        self.camera_topic = params['camera_topic'].value

        self.publisher = self.create_publisher(Image, self.camera_topic, 10)
        self.timer = self.create_timer(0.1, self.capture_and_publish)  # 10 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            self.timer.cancel()
            return

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
            # self.get_logger().info("Publishing image")
        else:
            self.get_logger().warn("Failed to capture image")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()