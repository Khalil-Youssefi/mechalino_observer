import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class undistorted_img_pub(Node):
    def __init__(self):
        super().__init__('undistorted_img_pub')
        
        # Initialize CvBridge to convert ROS messages to OpenCV
        self.bridge = CvBridge()

        # get parameters
        self.declare_parameter('camera_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('undistorted_camera_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('camera_matrix', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dist_coeff', rclpy.Parameter.Type.DOUBLE_ARRAY)

        params = self._parameters
        self.camera_topic = params['camera_topic'].value
        self.undistorted_camera_topic = params['undistorted_camera_topic'].value
        self.K = np.array(params['camera_matrix'].value).reshape(3, 3)
        self.dist_coeffs = np.array(params['dist_coeff'].value).reshape(5, 1)

        # Create a subscriber to the input image topic
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,  # Input image topic
            self.listener_callback,
            10
        )
        
        # Create a publisher to the output image topic
        self.publisher = self.create_publisher(Image, self.undistorted_camera_topic, 10)

    def listener_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Apply undistortion using OpenCV
            undistorted_image = cv2.undistort(cv_image, self.K, self.dist_coeffs)
            
            # Convert the undistorted image back to a ROS Image message
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')
            
            # Publish the undistorted image
            self.publisher.publish(undistorted_msg)
            # self.get_logger().info('Undistorted image published.')

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = undistorted_img_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
