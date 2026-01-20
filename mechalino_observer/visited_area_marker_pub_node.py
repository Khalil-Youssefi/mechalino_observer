#!/usr/bin/env python3
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class visited_area_marker_pub_node(Node):
    """
    Publishes, for each robot frame, a persistent LINE_LIST marker containing
    circles (rings) of radius Rs at the robot's historical positions.

    Frame convention:
      - world_frame: "arena" (default)
      - robot_frames: ["mechalino_15", "mechalino_16", ...] (parameter)
    """

    def __init__(self):
        super().__init__("visited_area_marker_pub_node")

        # ---- Hardcoded / tuneables (keep hardcoded if you prefer) ----
        self.rs_m = 0.06               # 6 cm radius
        self.circle_segments = 24      # circle resolution (more = smoother, heavier)
        self.sample_hz = 1.0           # how often we sample TF
        self.min_dist_m = 0.01         # ignore samples if robot moved less than 1 cm
        self.line_width = 0.004        # marker line width in meters

        # ---- Parameters (optional; sensible defaults) ----
        self.declare_parameter("world_frame", "arena")
        self.declare_parameter("robot_frames", ["mechalino_15", "mechalino_16"])
        self.declare_parameter("marker_topic", "coverage_markers")

        self.world_frame = self.get_parameter("world_frame").value
        self.robot_frames = list(self.get_parameter("robot_frames").value)
        self.marker_topic = self.get_parameter("marker_topic").value

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # State: last position per robot and accumulated segments
        self.last_pos: Dict[str, Tuple[float, float, float]] = {}
        self.ring_segments: Dict[str, List[Point]] = {rf: [] for rf in self.robot_frames}

        # Assign colors per robot (example mapping + fallback palette)
        self.robot_colors = self._build_color_map(self.robot_frames)

        # Timer
        period = 1.0 / max(self.sample_hz, 0.1)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Coverage rings from TF started. world_frame='{self.world_frame}', "
            f"robot_frames={self.robot_frames}, Rs={self.rs_m:.3f} m, topic='{self.marker_topic}'"
        )

    def _build_color_map(self, robot_frames: List[str]) -> Dict[str, Tuple[float, float, float, float]]:
        """
        Returns RGBA per robot frame.
        Example requirement: mechalino_15 red, mechalino_16 blue.
        Others cycle through a palette.
        """
        palette = [
            (1.0, 0.0, 0.0, 0.9),  # red
            (0.0, 0.0, 1.0, 0.9),  # blue
            (0.0, 0.7, 0.0, 0.9),  # green
            (1.0, 0.6, 0.0, 0.9),  # orange
            (0.6, 0.0, 0.6, 0.9),  # purple
            (0.0, 0.7, 0.7, 0.9),  # cyan
            (0.5, 0.5, 0.5, 0.9),  # gray
        ]

        explicit = {
            "mechalino_15": (1.0, 0.0, 0.0, 0.9),
            "mechalino_16": (0.0, 0.0, 1.0, 0.9),
        }

        colors = {}
        palette_i = 0
        for rf in robot_frames:
            if rf in explicit:
                colors[rf] = explicit[rf]
            else:
                colors[rf] = palette[palette_i % len(palette)]
                palette_i += 1
        return colors

    def _distance(self, a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _append_circle_as_line_segments(self, robot_frame: str, cx: float, cy: float, cz: float):
        """
        Append a circle centered at (cx,cy,cz) in the XY plane to the robot's LINE_LIST geometry.
        LINE_LIST expects points in pairs: (p0,p1), (p2,p3), ... each pair is a segment.
        """
        n = max(8, int(self.circle_segments))
        r = float(self.rs_m)

        for k in range(n):
            a0 = 2.0 * math.pi * (k / n)
            a1 = 2.0 * math.pi * ((k + 1) / n)

            p0 = Point(x=cx + r * math.cos(a0), y=cy + r * math.sin(a0), z=cz)
            p1 = Point(x=cx + r * math.cos(a1), y=cy + r * math.sin(a1), z=cz)

            self.ring_segments[robot_frame].append(p0)
            self.ring_segments[robot_frame].append(p1)

    def on_timer(self):
        marker_array = MarkerArray()

        for idx, robot_frame in enumerate(self.robot_frames):
            # Lookup TF: world_frame -> robot_frame
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    robot_frame,
                    rclpy.time.Time()
                )
            except TransformException as e:
                # Don't spam at high rate; debug is usually enough
                self.get_logger().debug(f"TF lookup failed for {self.world_frame}->{robot_frame}: {e}")
                continue

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            pos = (x, y, z)

            # Sample gating: only add a new ring if moved enough
            if robot_frame in self.last_pos:
                if self._distance(pos, self.last_pos[robot_frame]) < self.min_dist_m:
                    # Still publish existing markers, but do not append new geometry
                    pass
                else:
                    self._append_circle_as_line_segments(robot_frame, x, y, z)
                    self.last_pos[robot_frame] = pos
            else:
                # First sample: add ring immediately
                self._append_circle_as_line_segments(robot_frame, x, y, z)
                self.last_pos[robot_frame] = pos

            # Build per-robot marker (persistent, growing geometry)
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = f"coverage_rings/{robot_frame}"
            m.id = 0  # one marker per robot namespace
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD

            # LINE_LIST uses scale.x as line width
            m.scale.x = float(self.line_width)

            r, g, b, a = self.robot_colors.get(robot_frame, (1.0, 1.0, 1.0, 0.9))
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)

            # Persist forever unless deleted
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0

            # Add accumulated segments
            m.points = self.ring_segments[robot_frame]

            marker_array.markers.append(m)

        # Publish all robot markers in one message
        if marker_array.markers:
            self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = visited_area_marker_pub_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()