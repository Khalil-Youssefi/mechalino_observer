import socket
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from mechalino_observer.tf_pose_tcp_server import TfPoseTcpServer


def _client_server():
    return SimpleNamespace(
        child_prefix='mechalino_',
        client_idle_timeout_s=30.0,
        max_request_bytes=256,
        world='arena',
        _stop_event=MagicMock(is_set=MagicMock(return_value=False)),
        _answer_request=MagicMock(),
        get_logger=MagicMock(return_value=MagicMock()),
    )


def test_client_processes_every_buffered_request_line():
    server = _client_server()
    connection = MagicMock()
    connection.recv.side_effect = [b'POS\nPOS\n', b'']

    with patch(
        'mechalino_observer.tf_pose_tcp_server.rclpy.ok',
        return_value=True,
    ):
        TfPoseTcpServer.handle_client(server, connection, '192.168.50.15')

    requests = server._answer_request.call_args_list
    assert len(requests) == 2
    assert requests[0].args[1:] == (15, '192.168.50.15', b'POS')
    assert requests[1].args[1:] == (15, '192.168.50.15', b'POS')


def test_idle_client_is_expired():
    server = _client_server()
    connection = MagicMock()
    connection.recv.side_effect = socket.timeout

    with patch(
        'mechalino_observer.tf_pose_tcp_server.rclpy.ok',
        return_value=True,
    ), patch(
        'mechalino_observer.tf_pose_tcp_server.time.monotonic',
        side_effect=[0.0, 31.0],
    ):
        TfPoseTcpServer.handle_client(server, connection, '192.168.50.15')

    connection.recv.assert_called_once_with(256)


def test_client_is_rejected_when_handler_limit_is_reached():
    connection = MagicMock()
    server = SimpleNamespace(
        _client_slots=MagicMock(),
        get_logger=MagicMock(return_value=MagicMock()),
    )
    server._client_slots.acquire.return_value = False

    TfPoseTcpServer._start_client(server, connection, '192.168.50.15')

    connection.close.assert_called_once_with()
