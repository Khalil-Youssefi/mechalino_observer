#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import requests
import math
from threading import Thread, Lock


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Declare and get parameters
        self.declare_parameter('odom_publisher_frequency', 10.0)
        self.declare_parameter('robots_intern_pose_topic', 'robot_pose')
        self.declare_parameter('robots_pose_topic', 'robot_pose')
        self.declare_parameter('ip_list_file', '/opt/mechalinos/ip_list.txt')
        self.declare_parameter('http_timeout', 1.0)
        
        params = self._parameters
        self.frequency = params['odom_publisher_frequency'].value
        self.robots_intern_pose_topic = params['robots_intern_pose_topic'].value
        self.robots_pose_topic = params['robots_pose_topic'].value
        self.ip_list_file = params['ip_list_file'].value
        self.http_timeout = params['http_timeout'].value
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store latest poses for each robot
        self.robot_poses = {}
        self.pose_lock = Lock()
        
        # Read IP list
        self.robot_list = []
        self.read_ip_list()
        
        if not self.robot_list:
            self.get_logger().error(f'No robots found in {self.ip_list_file}')
            return
        
        self.get_logger().info(f'Loaded {len(self.robot_list)} robots')
        for ip, robot_id in self.robot_list:
            self.get_logger().info(f'  Robot {robot_id}: {ip}')
        
        # Create subscribers for each robot
        self.subscribers = {}
        for ip, robot_id in self.robot_list:
            topic = f'{self.robots_pose_topic}/mechalino_{robot_id}'
            self.get_logger().info(f'Subscribing to {topic}')
            self.subscribers[robot_id] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, rid=robot_id, rip=ip: self.pose_callback(msg, rid, rip),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')
        
        # Create timer for periodic updates
        timer_period = 2 #1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Odom publisher started at {self.frequency} Hz')
    
    def read_ip_list(self):
        """Read IP list from file"""
        try:
            with open(self.ip_list_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) == 2:
                            ip = parts[0].strip()
                            robot_id = parts[1].strip()
                            self.robot_list.append((ip, robot_id))
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {self.ip_list_file}')
        except Exception as e:
            self.get_logger().error(f'Error reading IP list: {str(e)}')
    
    def pose_callback(self, msg, robot_id, ip):
        """Callback for robot pose messages"""
        # Extract pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Convert quaternion to yaw (in radians)
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Calculate yaw from quaternion
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Store pose
        with self.pose_lock:
            self.robot_poses[robot_id] = (x, y, z, yaw, ip)
    
    def send_pose_to_robot(self, ip, x, y, z, yaw):
        """Send pose to robot via HTTP"""
        try:
            url = f'http://{ip}/updatePose?x={x}&y={y}&theta={yaw}' # z is ignored
            # url = f'http://{ip}/getPose' # z is ignored
            response = requests.get(url, timeout=self.http_timeout)
            if response.status_code == 200:
                x, y, yaw = map(float, response.text.strip().split('#'))
                return x, y, z, yaw # z comes from the pose_estimator node detections, x,y,yaw are received from robot
        except requests.exceptions.Timeout:
            self.get_logger().warn(f'Timeout sending pose to {ip}')
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Request error for {ip}: {str(e)}')
        return None
    
    def get_robot_pose(self, ip):
        """Fetch pose from robot via HTTP"""
        try:
            url = f'http://{ip}/getPose'
            self.get_logger().info(f'Fetching pose from {ip}')
            response = requests.get(url, timeout=self.http_timeout)
            if response.status_code == 200:
                # Parse response: "x#y#yaw"
                data = response.text.strip().split('#')
                if len(data) == 3:
                    x = float(data[0])
                    y = float(data[1])
                    yaw = float(data[2])
                    return x, y, 125.0, yaw # z is set to 0 since not provided, x,y,yaw are received from robot
        except requests.exceptions.Timeout:
            self.get_logger().warn(f'Timeout fetching pose from {ip}')
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Request error for {ip}: {str(e)}')
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Parse error for {ip}: {str(e)}')
        except Exception as e:
            self.get_logger().warn(f'Unexpected error for {ip}: {str(e)}')
        return None
    
    def publish_transform(self, robot_id, x, y, z, yaw):
        """Publish transform for robot"""
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera'
        t.child_frame_id = f'{self.robots_intern_pose_topic}/{robot_id}'


        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 1.0
        
        # Convert yaw to quaternion
        qx, qy, qz, qw = quaternion_from_euler(-math.pi, 0, yaw)

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
    
    def timer_callback(self):
        """Periodic callback to send poses to robots"""
        # First, send poses from subscribed topics
        with self.pose_lock:
            robots_with_pose = set(self.robot_poses.keys())
            for robot_id, (x, y, z, yaw, ip) in self.robot_poses.items():
                pose = self.send_pose_to_robot(ip, x, y, z, yaw)
                if pose is not None:
                    x, y, z, yaw = pose
                    self.publish_transform(robot_id, x, y, z, yaw)

        
        # For robots without pose from topic, use getPose (which also updates the robot)
        for ip, robot_id in self.robot_list:
            if robot_id not in robots_with_pose:
                pose = self.get_robot_pose(ip)
                if pose is not None:
                    x, y, z, yaw = pose
                    self.publish_transform(robot_id, x, y, z, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()