import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # get parameters
        self.declare_parameter('camera_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ArUcoDictionary', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ArenaArUcoMarkerSize', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('MechalinoArUcoMarkerSize', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('arena_ArUcoID', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('robots_pose_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('arena_pose_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('camera_matrix', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dist_coeff', rclpy.Parameter.Type.DOUBLE_ARRAY)

        params = self._parameters
        self.camera_topic = params['camera_topic'].value
        self.ArUcoDictionary = params['ArUcoDictionary'].value
        self.ArenaArUcoMarkerSize = params['ArenaArUcoMarkerSize'].value
        self.MechalinoArUcoMarkerSize = params['MechalinoArUcoMarkerSize'].value
        self.arena_ArUcoID = params['arena_ArUcoID'].value
        self.robots_pose_topic = params['robots_pose_topic'].value
        self.arena_pose_topic = params['arena_pose_topic'].value
        self.K = np.array(params['camera_matrix'].value).reshape(3, 3)
        self.dist_coeffs = np.array(params['dist_coeff'].value).reshape(5, 1)
        self.arena_object_points = np.array([[-self.ArenaArUcoMarkerSize/2, self.ArenaArUcoMarkerSize/2, 0],
                               [self.ArenaArUcoMarkerSize/2, self.ArenaArUcoMarkerSize/2, 0],
                               [self.ArenaArUcoMarkerSize/2, -self.ArenaArUcoMarkerSize/2, 0],
                               [-self.ArenaArUcoMarkerSize/2, -self.ArenaArUcoMarkerSize/2, 0]], dtype=np.float32)
        self.mechalino_object_points = np.array([[-self.MechalinoArUcoMarkerSize/2, self.MechalinoArUcoMarkerSize/2, 0],
                               [self.MechalinoArUcoMarkerSize/2, self.MechalinoArUcoMarkerSize/2, 0],
                               [self.MechalinoArUcoMarkerSize/2, -self.MechalinoArUcoMarkerSize/2, 0],
                               [-self.MechalinoArUcoMarkerSize/2, -self.MechalinoArUcoMarkerSize/2, 0]], dtype=np.float32)
        
        self.get_logger().info(f'opencv version: {cv2.__version__}') 
        # OpenCV bridge
        self.cv_bridge = CvBridge()
        
        # # ArUco dictionary and detector parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ArUcoDictionary)

        detectorParams = cv2.aruco.DetectorParameters()

        # Very important! These are important for pose estimation
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # CORNER_REFINE_NONE | CORNER_REFINE_SUBPIX | CORNER_REFINE_CONTOUR
        # for subpix
        detectorParams.cornerRefinementWinSize = 5 # 5
        detectorParams.cornerRefinementMaxIterations = 30 # 30
        detectorParams.cornerRefinementMinAccuracy = 0.1 # 0.1
        self.aruco_marker_detector = cv2.aruco.ArucoDetector(aruco_dict, detectorParams, cv2.aruco.RefineParameters())

        # tvec and rvec history for averaging for arena tf
        self.rotation_filtered = None  # Store as Rotation object
        self.tvec_filtered = None
        self.alpha = 0.15  # filter coefficient

        self.subscription = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.robot_pose_publishers = dict()

        self.get_logger().info("Pose Estimator Node has started.")

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.get_logger().debug("Image received.")
            marker_corners, marker_ids, _ = self.aruco_marker_detector.detectMarkers(gray)
            
            if marker_ids is None:
                self.get_logger().warn("No markers detected!")
                return
            
            for i, marker_id in enumerate(marker_ids.flatten()):
                if marker_id == self.arena_ArUcoID:
                    object_points = self.arena_object_points
                else:
                    object_points = self.mechalino_object_points

                retval, rvec, tvec = cv2.solvePnP(
                    object_points,
                    marker_corners[i],
                    self.K,
                    self.dist_coeffs)
                
                rvec_f = None # placeholder
                tvec_f = None # placeholder

                if marker_id == self.arena_ArUcoID:
                    # Convert rvec to Rotation object
                    current_rotation = R.from_rotvec(rvec.flatten())
                    
                    if self.rotation_filtered is None:
                        rotation_f = current_rotation
                        tvec_f = tvec
                    else:
                        # SLERP (Spherical Linear Interpolation) for rotations
                        # This is the mathematically correct way to "average" rotations
                        rotation_f = self.rotation_filtered * (
                            self.rotation_filtered.inv() * current_rotation
                        ) ** self.alpha  # Interpolate on manifold
                        
                        # Or use scipy's built-in slerp:
                        # from scipy.spatial.transform import Slerp
                        # slerp = Slerp([0, 1], R.concatenate([self.rotation_filtered, current_rotation]))
                        # rotation_f = slerp(self.alpha)
                        
                        tvec_f = self.alpha * tvec + (1 - self.alpha) * self.tvec_filtered
                    
                    self.rotation_filtered = rotation_f
                    self.tvec_filtered = tvec_f
                    
                    # Convert back to rvec for solvePnP output format
                    rvec_f = rotation_f.as_rotvec().reshape(-1, 1)
                else:
                    rvec_f = rvec
                    tvec_f = tvec

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec_f))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera'

                if marker_id == self.arena_ArUcoID:
                    t.child_frame_id = f"arena"
                else:
                    t.child_frame_id = f"mechalino_{marker_id}"
                    
                t.transform.translation.x = tvec_f[0][0]
                t.transform.translation.y = tvec_f[1][0]
                t.transform.translation.z = tvec_f[2][0]
        
                # # Quaternion format     
                t.transform.rotation.x = quat[0] 
                t.transform.rotation.y = quat[1] 
                t.transform.rotation.z = quat[2] 
                t.transform.rotation.w = quat[3] 
                
                self.tf_broadcaster.sendTransform(t)

                if marker_id != self.arena_ArUcoID:
                    # check if marker_id is in the robot pose publishers dict
                    if marker_id not in self.robot_pose_publishers:
                        # using standard msg pose
                        self.robot_pose_publishers[marker_id] = self.create_publisher(PoseStamped, f"{self.robots_pose_topic}/mechalino_{marker_id}", 10)
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = f"mechalino_{marker_id}"
                    pose_msg.pose.position.x = tvec_f[0][0]
                    pose_msg.pose.position.y = tvec_f[1][0]
                    pose_msg.pose.position.z = tvec_f[2][0]
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]
                    self.robot_pose_publishers[marker_id].publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()