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
import time

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # get parameters
        self.declare_parameter('camera_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ArUcoDictionary', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ArenaArUcoMarkerSize', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('MechalinoArUcoMarkerSize', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('arena_ArUcoID', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('arena_marker_xy', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('robots_pose_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('arena_pose_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('camera_matrix', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('dist_coeff', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('cutoff_freq', rclpy.Parameter.Type.DOUBLE)

        params = self._parameters
        self.camera_topic = params['camera_topic'].value
        self.ArUcoDictionary = params['ArUcoDictionary'].value
        self.ArenaArUcoMarkerSize = params['ArenaArUcoMarkerSize'].value
        self.MechalinoArUcoMarkerSize = params['MechalinoArUcoMarkerSize'].value
        self.arena_ArUcoID = params['arena_ArUcoID'].value
        self.arena_marker_xy = params['arena_marker_xy'].value
        self.robots_pose_topic = params['robots_pose_topic'].value
        self.arena_pose_topic = params['arena_pose_topic'].value
        self.K = np.array(params['camera_matrix'].value).reshape(3, 3)
        self.dist_coeffs = np.array(params['dist_coeff'].value).reshape(5, 1)
        self.cutoff_freq = params['cutoff_freq'].value
        
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

        # Low-pass filter state for each marker
        self.filtered_states = {}  # {marker_id: {'rotation': R, 'tvec': array, 'timestamp': float}}
        
        # Velocity constraint for outlier rejection
        self.max_velocity = 0.20  # 20 cm/s in meters
        self.max_angular_velocity = np.pi  # rad/s (reasonable default)

        self.subscription = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.robot_pose_publishers = dict()

        self.get_logger().info(f"Pose Estimator Node started with {self.cutoff_freq} Hz cutoff frequency")

        self.declare_parameter('grid_k', 0.15)
        self.declare_parameter('grid_offset_x', 0.15)
        self.declare_parameter('grid_offset_y', 0.15)
        self.declare_parameter('grid_m', 11)
        self.declare_parameter('grid_n', 4)
        self.declare_parameter('grid_image_topic', '/arena/grid_image')

        self.grid_k = self.get_parameter('grid_k').value
        self.grid_offset_x = self.get_parameter('grid_offset_x').value
        self.grid_offset_y = self.get_parameter('grid_offset_y').value
        self.grid_m = self.get_parameter('grid_m').value
        self.grid_n = self.get_parameter('grid_n').value

        self.grid_image_publisher = self.create_publisher(
            Image,
            self.get_parameter('grid_image_topic').value,
            10
        )

    def draw_grid(self, image, rvec, tvec):
        overlay = image.copy()

        k = self.grid_k

        # X0/Y0 are centers of cell [0][0].
        # Therefore the outer grid boundary is half a cell before them.
        xmin = self.grid_offset_x - k / 2.0
        ymin = self.grid_offset_y - k / 2.0

        xmax = xmin + self.grid_m * k
        ymax = ymin + self.grid_n * k

        marker_x = self.arena_marker_xy[0]
        marker_y = self.arena_marker_xy[1]

        def project(table_points):
            marker_points = []

            for x, y in table_points:
                marker_points.append([
                    x - marker_x,
                    y - marker_y,
                    0.0
                ])

            marker_points = np.asarray(
                marker_points,
                dtype=np.float32
            )

            img_points, _ = cv2.projectPoints(
                marker_points,
                rvec,
                tvec,
                self.K,
                self.dist_coeffs
            )

            return img_points.reshape(-1, 2).astype(int)

        # Number of points used to approximate lens distortion
        N = 50

        # Vertical grid lines
        for c in range(self.grid_m + 1):
            x = xmin + c * k

            ys = np.linspace(ymin, ymax, N)

            table_points = [
                (x, y)
                for y in ys
            ]

            p = project(table_points)

            cv2.polylines(
                overlay,
                [p.reshape(-1, 1, 2)],
                False,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )


        # Horizontal grid lines
        for r in range(self.grid_n + 1):
            y = ymin + r * k

            xs = np.linspace(xmin, xmax, N)

            table_points = [
                (x, y)
                for x in xs
            ]

            p = project(table_points)

            cv2.polylines(
                overlay,
                [p.reshape(-1, 1, 2)],
                False,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        return overlay

    def compute_alpha(self, dt, cutoff_freq):
        """
        Compute filter coefficient from cutoff frequency and sample time.
        alpha = dt / (dt + RC) where RC = 1 / (2 * pi * fc)
        """
        if dt <= 0:
            return 0.0
        RC = 1.0 / (2.0 * np.pi * cutoff_freq)
        alpha = dt / (dt + RC)
        return np.clip(alpha, 0.0, 1.0)

    def is_outlier(self, marker_id, tvec, rvec, dt):
        """
        Check if detection is an outlier based on velocity constraints.
        """
        if marker_id not in self.filtered_states or dt <= 0 or dt > 1.0:
            return False
        
        prev_state = self.filtered_states[marker_id]
        
        # Check translation velocity
        position_diff = np.linalg.norm(tvec - prev_state['tvec'])
        velocity = position_diff / dt
        
        if velocity > self.max_velocity * 1.5:  # 1.5x safety factor
            self.get_logger().warn(
                f"Outlier detected for marker {marker_id}: velocity {velocity:.3f} m/s "
                f"(max: {self.max_velocity:.3f} m/s)"
            )
            return True
        
        # Check angular velocity
        current_rotation = R.from_rotvec(rvec.flatten())
        rotation_diff = prev_state['rotation'].inv() * current_rotation
        angle_diff = np.linalg.norm(rotation_diff.as_rotvec())
        angular_velocity = angle_diff / dt
        
        if angular_velocity > self.max_angular_velocity * 1.5:
            self.get_logger().warn(
                f"Outlier detected for marker {marker_id}: angular velocity {angular_velocity:.3f} rad/s "
                f"(max: {self.max_angular_velocity:.3f} rad/s)"
            )
            return True
        
        return False

    def apply_lowpass_filter(self, marker_id, tvec, rvec, timestamp):
        """
        Apply low-pass filter to translation and rotation.
        Returns filtered rvec and tvec.
        """
        current_rotation = R.from_rotvec(rvec.flatten())
        
        # Initialize filter state for new markers
        if marker_id not in self.filtered_states:
            self.filtered_states[marker_id] = {
                'rotation': current_rotation,
                'tvec': tvec.copy(),
                'timestamp': timestamp
            }
            return rvec, tvec
        
        prev_state = self.filtered_states[marker_id]
        dt = timestamp - prev_state['timestamp']
        
        # Outlier rejection
        if self.is_outlier(marker_id, tvec, rvec, dt):
            # Return previous filtered value instead of raw measurement
            return prev_state['rotation'].as_rotvec().reshape(-1, 1), prev_state['tvec']
        
        # Compute filter coefficient based on actual sample time
        alpha = self.compute_alpha(dt, self.cutoff_freq)
        
        # Filter translation (simple exponential filter)
        tvec_filtered = alpha * tvec + (1.0 - alpha) * prev_state['tvec']
        
        # Filter rotation using SLERP (Spherical Linear Interpolation)
        rotation_filtered = prev_state['rotation'] * (
            prev_state['rotation'].inv() * current_rotation
        ) ** alpha
        
        # Update filter state
        self.filtered_states[marker_id] = {
            'rotation': rotation_filtered,
            'tvec': tvec_filtered,
            'timestamp': timestamp
        }
        
        rvec_filtered = rotation_filtered.as_rotvec().reshape(-1, 1)
        
        return rvec_filtered, tvec_filtered

    def image_callback(self, msg):
        try:
            timestamp = time.time()  # Use monotonic time for dt calculation
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.get_logger().debug("Image received.")
            marker_corners, marker_ids, _ = self.aruco_marker_detector.detectMarkers(gray)
            
            if marker_ids is None:
                self.get_logger().warn("No markers detected!")
                return

            arena_pose = None
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
                
                # Apply low-pass filter to all markers
                rvec_f, tvec_f = self.apply_lowpass_filter(marker_id, tvec, rvec, timestamp)

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec_f))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'camera'

                if marker_id == self.arena_ArUcoID:
                    t.child_frame_id = f"arena_ArUcoID"
                    arena_pose = (rvec_f.copy(), tvec_f.copy())
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

                # # make a copy of t for arena itself with adjusted position
                # if marker_id == self.arena_ArUcoID:
                #     arena_t = TransformStamped()
                #     arena_t.header = t.header
                #     arena_t.child_frame_id = "arena"
                #     arena_t.transform.translation.x = tvec_f[0][0] - self.arena_marker_xy[0]
                #     arena_t.transform.translation.y = tvec_f[1][0] + self.arena_marker_xy[1]
                #     arena_t.transform.translation.z = tvec_f[2][0]
                #     arena_t.transform.rotation = t.transform.rotation
                    
                #     self.tf_broadcaster.sendTransform(arena_t)

                if marker_id != self.arena_ArUcoID:
                    # check if marker_id is in the robot pose publishers dict
                    if marker_id not in self.robot_pose_publishers:
                        # using standard msg pose
                        self.robot_pose_publishers[marker_id] = self.create_publisher(PoseStamped, f"{self.robots_pose_topic}/mechalino_{marker_id}", 10)
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = f"camera"
                    pose_msg.pose.position.x = tvec_f[0][0]
                    pose_msg.pose.position.y = tvec_f[1][0]
                    pose_msg.pose.position.z = tvec_f[2][0]
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]
                    self.robot_pose_publishers[marker_id].publish(pose_msg)
            if arena_pose is not None:
                arena_rvec, arena_tvec = arena_pose

                grid_image = self.draw_grid(
                    cv_image,
                    arena_rvec,
                    arena_tvec
                )

                grid_msg = self.cv_bridge.cv2_to_imgmsg(
                    grid_image,
                    encoding='bgr8'
                )
                grid_msg.header = msg.header

                self.grid_image_publisher.publish(grid_msg)
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
