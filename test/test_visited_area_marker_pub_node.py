from types import SimpleNamespace
from unittest.mock import MagicMock

from mechalino_observer.visited_area_marker_pub_node import (
    visited_area_marker_pub_node,
)
from visualization_msgs.msg import Marker


def test_reset_markers_clears_history_and_deletes_rviz_markers():
    node = SimpleNamespace(
        last_pos={'mechalino_15': (1.0, 2.0, 0.0)},
        ring_segments={'mechalino_15': [object(), object()]},
        pub=MagicMock(),
        get_logger=MagicMock(return_value=MagicMock()),
    )

    visited_area_marker_pub_node.reset_markers(node, object())

    assert node.last_pos == {}
    assert node.ring_segments == {'mechalino_15': []}
    marker_array = node.pub.publish.call_args.args[0]
    assert len(marker_array.markers) == 1
    assert marker_array.markers[0].action == Marker.DELETEALL
