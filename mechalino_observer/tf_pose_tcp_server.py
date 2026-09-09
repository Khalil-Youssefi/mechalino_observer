#!/usr/bin/env python3
"""Serve arena-relative robot poses over a bounded TCP interface."""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(quaternion):
    """Return the Z-axis yaw from a quaternion message."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class TfPoseTcpServer(Node):
    """Answer newline-delimited POS requests without unbounded client threads."""

    def __init__(self):
        super().__init__('tf_pose_tcp_server')

        self.declare_parameter('world_frame', 'arena')
        self.declare_parameter('robot_frame_prefix', 'mechalino_')
        self.declare_parameter('tcp_port', 9000)
        self.declare_parameter('max_tcp_clients', 16)
        self.declare_parameter('client_idle_timeout_s', 30.0)
        self.declare_parameter('max_request_bytes', 256)

        self.world = str(self.get_parameter('world_frame').value)
        self.child_prefix = str(
            self.get_parameter('robot_frame_prefix').value
        )
        self.port = int(self.get_parameter('tcp_port').value)
        self.max_clients = int(self.get_parameter('max_tcp_clients').value)
        self.client_idle_timeout_s = float(
            self.get_parameter('client_idle_timeout_s').value
        )
        self.max_request_bytes = int(
            self.get_parameter('max_request_bytes').value
        )
        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._stop_event = threading.Event()
        self._client_slots = threading.BoundedSemaphore(self.max_clients)
        self._clients_lock = threading.Lock()
        self._client_connections = set()
        self._client_threads = set()
        self._server_socket = None
        self._server_thread = threading.Thread(
            target=self.tcp_loop,
            name='tf_pose_tcp_listener',
            daemon=True,
        )
        self._server_thread.start()

    def _validate_parameters(self):
        if not 1 <= self.port <= 65535:
            raise ValueError('tcp_port must be between 1 and 65535')
        if self.max_clients < 1:
            raise ValueError('max_tcp_clients must be positive')
        if self.client_idle_timeout_s <= 0.0:
            raise ValueError('client_idle_timeout_s must be positive')
        if self.max_request_bytes < 4:
            raise ValueError('max_request_bytes must be at least 4')

    def tcp_loop(self):
        """Accept clients until shutdown while enforcing the client limit."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self._server_socket = server
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind(('0.0.0.0', self.port))
                server.listen(self.max_clients)
                server.settimeout(0.5)
                self.get_logger().info(
                    f'Listening on TCP :{self.port}, world={self.world!r}, '
                    f'child_prefix={self.child_prefix!r}, '
                    f'max_clients={self.max_clients}'
                )

                while rclpy.ok() and not self._stop_event.is_set():
                    try:
                        connection, address = server.accept()
                    except socket.timeout:
                        continue
                    except OSError as error:
                        if not self._stop_event.is_set() and rclpy.ok():
                            self.get_logger().error(
                                f'TCP listener stopped unexpectedly: {error}'
                            )
                        break
                    self._start_client(connection, address[0])
        except OSError as error:
            if not self._stop_event.is_set() and rclpy.ok():
                self.get_logger().error(f'Could not start TCP server: {error}')
        finally:
            self._server_socket = None

    def _start_client(self, connection, ip_address):
        if not self._client_slots.acquire(blocking=False):
            self.get_logger().debug(
                f'Rejecting TCP client {ip_address}: client limit reached'
            )
            connection.close()
            return

        connection.settimeout(min(1.0, self.client_idle_timeout_s))
        thread = threading.Thread(
            target=self._run_client,
            args=(connection, ip_address),
            name=f'tf_pose_client_{ip_address}',
            daemon=True,
        )
        with self._clients_lock:
            self._client_connections.add(connection)
            self._client_threads.add(thread)
        thread.start()

    def _run_client(self, connection, ip_address):
        try:
            self.handle_client(connection, ip_address)
        finally:
            with self._clients_lock:
                self._client_connections.discard(connection)
                self._client_threads.discard(threading.current_thread())
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
            self._client_slots.release()

    def handle_client(self, connection, ip_address):
        """Read and answer all complete requests from one bounded client."""
        self.get_logger().debug(f'TCP client connected: {ip_address}')
        try:
            robot_id = int(ip_address.rsplit('.', 1)[-1])
        except ValueError:
            robot_id = None

        buffer = b''
        last_activity = time.monotonic()
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                data = connection.recv(self.max_request_bytes)
            except socket.timeout:
                if time.monotonic() - last_activity >= self.client_idle_timeout_s:
                    self.get_logger().debug(
                        f'Closing idle TCP client {ip_address}'
                    )
                    break
                continue
            except (ConnectionResetError, OSError):
                break

            if not data:
                break
            last_activity = time.monotonic()
            buffer += data

            while b'\n' in buffer:
                line, _, buffer = buffer.partition(b'\n')
                if len(line) > self.max_request_bytes:
                    connection.sendall(b'ERR\n')
                    continue
                self._answer_request(connection, robot_id, ip_address, line)

            if len(buffer) > self.max_request_bytes:
                connection.sendall(b'ERR\n')
                break

    def _answer_request(self, connection, robot_id, ip_address, request):
        if request.strip() != b'POS' or robot_id is None:
            connection.sendall(b'ERR\n')
            return

        child_frame = f'{self.child_prefix}{robot_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world,
                child_frame,
                Time(),
            )
        except Exception as error:
            connection.sendall(b'ERR\n')
            self.get_logger().debug(
                f'TF lookup failed {self.world}->{child_frame}: {error}'
            )
            return

        translation = transform.transform.translation
        yaw = yaw_from_quat(transform.transform.rotation)
        connection.sendall(
            f'{translation.x:.6f} {translation.y:.6f} {yaw:.6f}\n'.encode()
        )
        self.get_logger().debug(
            f'Sent pose to {ip_address}: x={translation.x:.2f}, '
            f'y={translation.y:.2f}, yaw={yaw:.2f}'
        )

    def destroy_node(self):
        """Close all sockets before destroying ROS resources."""
        self._stop_event.set()
        server = self._server_socket
        if server is not None:
            try:
                server.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            server.close()

        with self._clients_lock:
            connections = list(self._client_connections)
            threads = list(self._client_threads)
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

        deadline = time.monotonic() + 2.0
        self._server_thread.join(timeout=2.0)
        for thread in threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TfPoseTcpServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
