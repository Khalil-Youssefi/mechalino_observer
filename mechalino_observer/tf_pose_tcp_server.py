#!/usr/bin/env python3
import math, socket, threading
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

def yaw_from_quat(q):
    # yaw around Z
    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

class TfPoseTcpServer(Node):
    def __init__(self):
        super().__init__("tf_pose_tcp_server")

        self.world = "camera"
        self.child_prefix = "mechalino_"
        self.port = 9000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        threading.Thread(target=self.tcp_loop, daemon=True).start()
        self.get_logger().info(f"Listening on TCP :{self.port}, world='{self.world}', child_prefix='{self.child_prefix}'")

    def tcp_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", int(self.port)))
        srv.listen(64)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
        while rclpy.ok():
            conn, addr = srv.accept()              # addr = (ip, port)
            conn.settimeout(2.0)
            threading.Thread(target=self.handle_client, args=(conn, addr[0]), daemon=True).start()

    def handle_client(self, conn, ip_str: str):
        # info about connected client: ip_str
        self.get_logger().info(f"Client connected: {ip_str}")
        try:
            rid = int(ip_str.split(".")[-1])       # last octet
        except Exception:
            rid = None

        try:
            buf = b""
            while rclpy.ok():
                try:
                    data = conn.recv(64)
                except TimeoutError:
                    continue
                if not data:
                    break
                buf += data
                if b"\n" not in buf:
                    continue
                line, _, buf = buf.partition(b"\n")
                if line.strip() != b"POSE":
                    conn.sendall(b"ERR\n")
                    continue

                if rid is None:
                    conn.sendall(b"ERR\n")
                    continue

                child = f"{self.child_prefix}{rid}"
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "arena",          # target frame
                        child,            # source frame
                        Time()            # latest available
                    )
                except Exception as e:
                    conn.sendall(b"ERR\n")
                    self.get_logger().warn(f"TF lookup failed arena->{child}: {e}")
                    continue

                tr = tf.transform.translation
                q  = tf.transform.rotation
                x, y, yaw = tr.x, tr.y, yaw_from_quat(q)
                conn.sendall(f"{x:.6f} {y:.6f} {yaw:.6f}\n".encode())
                self.get_logger().info(f"Sent pose to {ip_str}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                # self.get_logger().info(f"I should tell Mechalino_{ip_str} that it is in: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        finally:
            conn.close()

def main():
    rclpy.init()
    node = TfPoseTcpServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()