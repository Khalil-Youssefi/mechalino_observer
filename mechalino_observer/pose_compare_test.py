#!/usr/bin/env python3
import re
import math
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


POSE_RE = re.compile(
    r'([+-]?\d+(?:\.\d+)?)#([+-]?\d+(?:\.\d+)?)#([+-]?\d+(?:\.\d+)?)'
)


def quat_from_yaw(yaw):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


class PoseCompareTest(Node):
    def __init__(self):
        super().__init__("pose_compare_test")

        # ---- STATIC CONFIG ----
        self.url = "http://192.168.50.15/cmd?cmd=P"
        self.frame_id = "arena"
        self.robot_pose_topic = "/robots/pose/mechalino_15"
        self.marker_topic = "/pose_compare_markers"
        self.poll_period = 5.0
        self.http_timeout = 5.0
        # -----------------------

        self.last_cam_pose = None        # PoseStamped
        self.last_robot_pose = None      # (x, y, yaw)

        self.create_subscription(
            PoseStamped,
            self.robot_pose_topic,
            self.cam_pose_cb,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.create_timer(self.poll_period, self.tick)

        self.get_logger().info("Pose comparison test node started (robot 15 only)")

    # -------------------------------------------------

    def cam_pose_cb(self, msg: PoseStamped):
        self.last_cam_pose = msg

    def tick(self):
        self.poll_robot_http()
        self.publish_markers()

    # -------------------------------------------------

    def poll_robot_http(self):
        try:
            with urllib.request.urlopen(self.url, timeout=self.http_timeout) as r:
                txt = r.read().decode("utf-8", errors="ignore")
        except Exception as e:
            self.get_logger().warn(f"HTTP error: {e}")
            return

        m = POSE_RE.search(txt)
        if not m:
            return

        try:
            x = float(m.group(1))
            y = float(m.group(2))
            yaw = float(m.group(3))   # assumed radians
        except ValueError:
            return

        self.last_robot_pose = (x, y, yaw)

    # -------------------------------------------------

    def publish_markers(self):
        if self.last_cam_pose is None and self.last_robot_pose is None:
            return

        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # --- CAMERA POSE (GREEN) ---
        if self.last_cam_pose is not None:
            ma.markers.append(
                self.make_marker(
                    mid=1,
                    ns="camera_pose",
                    stamp=now,
                    x=self.last_cam_pose.pose.position.x,
                    y=self.last_cam_pose.pose.position.y,
                    yaw=self.yaw_from_pose(self.last_cam_pose),
                    color=(0.0, 1.0, 0.0, 1.0)
                )
            )

        # --- ROBOT SELF POSE (RED) ---
        if self.last_robot_pose is not None:
            x, y, yaw = self.last_robot_pose
            ma.markers.append(
                self.make_marker(
                    mid=2,
                    ns="robot_self_pose",
                    stamp=now,
                    x=x,
                    y=y,
                    yaw=yaw,
                    color=(1.0, 0.0, 0.0, 1.0)
                )
            )

        self.marker_pub.publish(ma)

    # -------------------------------------------------

    def make_marker(self, mid, ns, stamp, x, y, yaw, color):
        qx, qy, qz, qw = quat_from_yaw(yaw)

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        m.scale.x = 0.12
        m.scale.y = 0.03
        m.scale.z = 0.03

        m.color.r, m.color.g, m.color.b, m.color.a = color

        m.lifetime.sec = 6   # survive across polls
        m.lifetime.nanosec = 0

        return m

    @staticmethod
    def yaw_from_pose(p: PoseStamped):
        q = p.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)


def main():
    rclpy.init()
    node = PoseCompareTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()