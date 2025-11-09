#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TableMarkerPub(Node):
    def __init__(self):
        super().__init__('plane_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.marker = Marker()
        self.marker.header.frame_id = "arena"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "plane"
        self.marker.id = 0
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0

        # Define plane corners (z=0)
        p1 = Point(x=0.0, y=0.0, z=0.0)
        p2 = Point(x=1.8, y=0.0, z=0.0)
        p3 = Point(x=1.8, y=0.8, z=0.0)
        p4 = Point(x=0.0, y=0.8, z=0.0)

        # Split into two triangles (p1,p2,p3) and (p1,p3,p4)
        self.marker.points = [p1, p2, p3, p1, p3, p4]

        # Plane color (light gray)
        self.marker.color.r = 0.7
        self.marker.color.g = 0.7
        self.marker.color.b = 0.7
        self.marker.color.a = 0.5  # transparency

        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

    def timer_callback(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = TableMarkerPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
