#!/usr/bin/env python3
"""Publish robot debug maps and live IR detections for RViz."""

from dataclasses import dataclass
import math
import re
import time
from typing import Optional, Sequence, Tuple
import urllib.request

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


SENSOR_NAMES = ('front', 'front-right', 'front-left')
SENSOR_OFFSETS_RAD = (0.0, -math.pi / 4.0, math.pi / 4.0)
GOTO_STATE_NAMES = {
    0: 'idle',
    1: 'rotate',
    2: 'drive',
    3: 'backoff',
    4: 'done',
    5: 'inactive',
}


@dataclass(frozen=True)
class IrStatus:
    """One decoded #IR record and its optional #IC cell projections."""

    frame: int
    age_ms: int
    fresh: bool
    raw_mask: int
    confirmed_mask: int
    robot_mask: int
    static_mask: int
    goto_state: int
    cells: Tuple[Optional[Tuple[int, int]], ...]


def _extract_field(text: str, names: Sequence[str]) -> str:
    """Extract a field from either HTTP lines or the raw DEBUG# stream."""
    for name in names:
        pattern = rf'(?:^|[#\r\n]){re.escape(name)}\s*[:=]\s*([^#\r\n]*)'
        match = re.search(pattern, text, re.MULTILINE)
        if match:
            return match.group(1).strip()
    raise ValueError(f'{names[0]} not found')


def _decimal(value: str, field_name: str) -> int:
    try:
        return int(value.strip(), 10)
    except ValueError as exc:
        raise ValueError(f'invalid {field_name}: {value!r}') from exc


def _hex_mask(value: str, field_name: str) -> int:
    try:
        # The firmware emits these fields using printf's %X, without a 0x
        # prefix. Parsing as hexadecimal also handles the usual 0..7 masks.
        return int(value.strip(), 16)
    except ValueError as exc:
        raise ValueError(f'invalid {field_name} mask: {value!r}') from exc


def parse_debug_matrix(
    text: str,
    names: Sequence[str],
    rows: int,
    columns: int,
) -> list:
    """Parse a semicolon-separated integer matrix."""
    payload = _extract_field(text, names)
    row_fields = payload.split(';')
    if len(row_fields) != rows:
        raise ValueError(
            f'{names[0]} has {len(row_fields)} rows; expected {rows}'
        )

    matrix = []
    for row_index, row_field in enumerate(row_fields):
        try:
            values = [int(value.strip(), 10) for value in row_field.split(',')]
        except ValueError as exc:
            raise ValueError(
                f'{names[0]} row {row_index} contains a non-integer'
            ) from exc
        if len(values) != columns:
            raise ValueError(
                f'{names[0]} row {row_index} has {len(values)} columns; '
                f'expected {columns}'
            )
        matrix.append(values)
    return matrix


def parse_debug_obstacles(
    text: str,
    names: Sequence[str],
    rows: int,
    columns: int,
) -> list:
    """Decode one hexadecimal obstacle bit mask per row."""
    payload = _extract_field(text, names)
    masks = payload.split(',')
    if len(masks) != rows:
        raise ValueError(
            f'{names[0]} has {len(masks)} row masks; expected {rows}'
        )

    matrix = []
    for row_index, value in enumerate(masks):
        mask = _hex_mask(value, f'{names[0]} row {row_index}')
        matrix.append(
            [1 if mask & (1 << column) else 0 for column in range(columns)]
        )
    return matrix


def parse_ir_status(text: str) -> IrStatus:
    """Decode the new #IR and #IC debug fields."""
    fields = [value.strip() for value in _extract_field(text, ('IR',)).split(',')]
    if len(fields) != 8:
        raise ValueError(f'IR has {len(fields)} fields; expected 8')

    cells = [None, None, None]
    try:
        cell_fields = _extract_field(text, ('IC',)).split(';')
    except ValueError:
        cell_fields = []

    if cell_fields and len(cell_fields) != len(SENSOR_NAMES):
        raise ValueError(
            f'IC has {len(cell_fields)} cells; expected {len(SENSOR_NAMES)}'
        )

    for index, cell_field in enumerate(cell_fields):
        coordinates = [value.strip() for value in cell_field.split(',')]
        if len(coordinates) != 2:
            raise ValueError(f'IC entry {index} is not a row,column pair')
        try:
            row = int(coordinates[0], 10)
            column = int(coordinates[1], 10)
        except ValueError:
            # A textual unavailable value is treated like firmware's -1,-1.
            continue
        if row >= 0 and column >= 0:
            cells[index] = (row, column)

    return IrStatus(
        frame=_decimal(fields[0], 'IR frame'),
        age_ms=_decimal(fields[1], 'IR age_ms'),
        fresh=bool(_decimal(fields[2], 'IR fresh')),
        raw_mask=_hex_mask(fields[3], 'IR raw'),
        confirmed_mask=_hex_mask(fields[4], 'IR confirmed'),
        robot_mask=_hex_mask(fields[5], 'IR robot'),
        static_mask=_hex_mask(fields[6], 'IR static'),
        goto_state=_decimal(fields[7], 'IR goto_state'),
        cells=tuple(cells),
    )


class DebugMapViz(Node):
    """Poll a Mechalino debug endpoint and publish spatial diagnostics."""

    def __init__(self):
        super().__init__('debug_map_viz')

        self._declare_parameters()
        self._read_parameters()

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Preserve the original image topics for quick inspection with an
        # Image display, and add real OccupancyGrid topics for spatial RViz use.
        self.pub_visits_image = self.create_publisher(
            Image, '/debug/map_visits', map_qos
        )
        self.pub_penalties_image = self.create_publisher(
            Image, '/debug/map_penalties_x10', map_qos
        )
        self.pub_obstacles_image = self.create_publisher(
            Image, '/debug/map_obstacles', map_qos
        )

        self.pub_visits_grid = self.create_publisher(
            OccupancyGrid, '/debug/map_visits_grid', map_qos
        )
        self.pub_penalties_grid = self.create_publisher(
            OccupancyGrid, '/debug/map_penalties_x10_grid', map_qos
        )
        self.pub_obstacles_grid = self.create_publisher(
            OccupancyGrid, '/debug/map_obstacles_grid', map_qos
        )
        self.pub_ir_markers = self.create_publisher(
            MarkerArray, '/debug/ir_obstacles', map_qos
        )

        self._last_warning = {}
        self._last_ir_status = None
        self.timer = self.create_timer(self.update_period_s, self.timer_cb)

        self.get_logger().info(
            'Debug map viz started: '
            f'url={self.debug_url}, robot_frame={self.robot_frame}, '
            f'grid={self.columns}x{self.rows}, period={self.update_period_s:.2f}s'
        )

    def _declare_parameters(self):
        self.declare_parameter('debug_url', 'http://192.168.50.15/debug')
        self.declare_parameter('update_period_s', 1.0)
        self.declare_parameter('http_timeout_s', 2.0)
        self.declare_parameter('world_frame', 'arena')
        self.declare_parameter('robot_frame', 'mechalino_15')
        self.declare_parameter('grid_m', 11)
        self.declare_parameter('grid_n', 4)
        self.declare_parameter('grid_k', 0.15)
        self.declare_parameter('grid_offset_x', 0.15)
        self.declare_parameter('grid_offset_y', 0.15)
        self.declare_parameter('map_z', 0.01)
        self.declare_parameter('obstacle_distance_m', 0.15)
        self.declare_parameter('marker_lifetime_s', 2.5)
        self.declare_parameter('cell_pixels', 60)
        self.declare_parameter('visits_max_value', 1000.0)
        # A non-positive scale automatically follows the largest current
        # penalty, keeping small but meaningful penalties visible.
        self.declare_parameter('penalties_x10_max_value', 0.0)

    def _read_parameters(self):
        self.debug_url = str(self.get_parameter('debug_url').value)
        self.update_period_s = float(
            self.get_parameter('update_period_s').value
        )
        self.http_timeout_s = float(
            self.get_parameter('http_timeout_s').value
        )
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.columns = int(self.get_parameter('grid_m').value)
        self.rows = int(self.get_parameter('grid_n').value)
        self.cell_size = float(self.get_parameter('grid_k').value)
        self.grid_offset_x = float(
            self.get_parameter('grid_offset_x').value
        )
        self.grid_offset_y = float(
            self.get_parameter('grid_offset_y').value
        )
        self.map_z = float(self.get_parameter('map_z').value)
        self.obstacle_distance_m = float(
            self.get_parameter('obstacle_distance_m').value
        )
        self.marker_lifetime_s = float(
            self.get_parameter('marker_lifetime_s').value
        )
        self.cell_pixels = int(self.get_parameter('cell_pixels').value)
        self.visits_max_value = float(
            self.get_parameter('visits_max_value').value
        )
        self.penalties_max_value = float(
            self.get_parameter('penalties_x10_max_value').value
        )

        if self.rows < 1 or self.columns < 1:
            raise ValueError('grid_m and grid_n must be positive')
        if self.cell_size <= 0.0 or self.cell_pixels < 2:
            raise ValueError('grid_k and cell_pixels must be positive')
        if self.update_period_s <= 0.0 or self.http_timeout_s <= 0.0:
            raise ValueError('update period and HTTP timeout must be positive')
        if self.marker_lifetime_s <= 0.0:
            raise ValueError('marker_lifetime_s must be positive')

        self.image_width = self.columns * self.cell_pixels
        self.image_height = self.rows * self.cell_pixels

    def timer_cb(self):
        try:
            text = self.fetch_debug_text()
        except Exception as exc:
            self._warn_throttled('fetch', f'Could not fetch debug data: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        map_specs = (
            (
                'visits',
                ('VISITS', 'V'),
                self.pub_visits_image,
                self.pub_visits_grid,
                self.visits_max_value,
            ),
            (
                'penalties',
                ('PENALTIES_X10', 'P10'),
                self.pub_penalties_image,
                self.pub_penalties_grid,
                self.penalties_max_value,
            ),
        )

        for mode, names, image_publisher, grid_publisher, scale_max in map_specs:
            try:
                matrix = parse_debug_matrix(
                    text, names, self.rows, self.columns
                )
                effective_scale = self._effective_scale(matrix, scale_max)
                image_publisher.publish(
                    self.matrix_to_image(matrix, mode, effective_scale, stamp)
                )
                grid_publisher.publish(
                    self.matrix_to_grid(matrix, mode, effective_scale, stamp)
                )
            except ValueError as exc:
                self._warn_throttled(mode, str(exc))

        try:
            obstacles = parse_debug_obstacles(
                text, ('OBSTACLES', 'O'), self.rows, self.columns
            )
            self.pub_obstacles_image.publish(
                self.matrix_to_image(obstacles, 'obstacles', 1.0, stamp)
            )
            self.pub_obstacles_grid.publish(
                self.matrix_to_grid(obstacles, 'obstacles', 1.0, stamp)
            )
        except ValueError as exc:
            self._warn_throttled('obstacles', str(exc))

        try:
            ir_status = parse_ir_status(text)
            # Do not refresh a frozen HTTP snapshot forever. If the firmware
            # stops producing IR frames, the last markers expire naturally.
            if ir_status != self._last_ir_status:
                self.pub_ir_markers.publish(
                    self.ir_status_to_markers(ir_status, stamp)
                )
                self._last_ir_status = ir_status
        except ValueError as exc:
            if self._last_ir_status is not None:
                markers = MarkerArray()
                delete_all = Marker()
                delete_all.action = Marker.DELETEALL
                markers.markers.append(delete_all)
                self.pub_ir_markers.publish(markers)
                self._last_ir_status = None
            self._warn_throttled('ir', str(exc))

    def _warn_throttled(self, key: str, message: str):
        now = time.monotonic()
        if now - self._last_warning.get(key, -math.inf) >= 10.0:
            self._last_warning[key] = now
            self.get_logger().warn(message)

    def fetch_debug_text(self) -> str:
        with urllib.request.urlopen(
            self.debug_url, timeout=self.http_timeout_s
        ) as response:
            return response.read().decode('utf-8', errors='replace')

    # Keep these wrappers convenient for callers which used the old class API.
    def parse_matrix(self, text: str, key: str) -> list:
        aliases = {'VISITS': 'V', 'PENALTIES_X10': 'P10'}
        names = (key, aliases[key]) if key in aliases else (key,)
        return parse_debug_matrix(text, names, self.rows, self.columns)

    def parse_obstacles(self, text: str) -> list:
        return parse_debug_obstacles(
            text, ('OBSTACLES', 'O'), self.rows, self.columns
        )

    def matrix_to_grid(
        self,
        matrix: list,
        mode: str,
        scale_max: float,
        stamp,
    ) -> OccupancyGrid:
        """Create an arena-aligned OccupancyGrid with row 0 at minimum Y."""
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.info.map_load_time = stamp
        msg.info.resolution = self.cell_size
        msg.info.width = self.columns
        msg.info.height = self.rows
        msg.info.origin.position.x = self.grid_offset_x - self.cell_size / 2.0
        msg.info.origin.position.y = self.grid_offset_y - self.cell_size / 2.0
        msg.info.origin.position.z = self.map_z
        msg.info.origin.orientation.w = 1.0

        msg.data = [
            self._occupancy_value(value, mode, scale_max)
            for row in matrix
            for value in row
        ]
        return msg

    @staticmethod
    def _effective_scale(matrix: list, requested_scale: float) -> float:
        if requested_scale > 0.0:
            return requested_scale
        return max(1.0, max(max(row) for row in matrix))

    @staticmethod
    def _occupancy_value(value: int, mode: str, scale_max: float) -> int:
        if mode == 'obstacles':
            return 100 if value > 0 else 0
        if value <= 0:
            return 0
        if scale_max <= 0.0:
            return 100
        return max(1, min(100, round(100.0 * value / scale_max)))

    def matrix_to_image(
        self,
        matrix: list,
        mode: str,
        scale_max: float,
        stamp,
    ) -> Image:
        """Create a heatmap image, with physical row 0 displayed at bottom."""
        data = bytearray(self.image_width * self.image_height * 3)

        for row in range(self.rows):
            image_row = self.rows - 1 - row
            for column in range(self.columns):
                value = matrix[row][column]
                if mode == 'visits':
                    color = self.color_scale(
                        value, scale_max, base=(40, 175, 80)
                    )
                elif mode == 'penalties':
                    color = self.color_scale(
                        value, scale_max, base=(255, 120, 20)
                    )
                elif mode == 'obstacles':
                    color = (220, 20, 20) if value > 0 else (245, 245, 245)
                else:
                    color = (255, 255, 255)
                self.fill_cell(data, image_row, column, color)

        self.draw_grid(data)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.height = self.image_height
        msg.width = self.image_width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.image_width * 3
        msg.data = bytes(data)
        return msg

    @staticmethod
    def color_scale(value: int, scale_max: float, base: tuple) -> tuple:
        if value <= 0:
            return (245, 245, 245)
        denominator = max(scale_max, 1.0)
        fraction = min(float(value) / denominator, 1.0)
        return tuple(
            int(245 * (1.0 - fraction) + component * fraction)
            for component in base
        )

    def fill_cell(self, data: bytearray, row: int, column: int, color: tuple):
        y_start = row * self.cell_pixels
        y_end = y_start + self.cell_pixels
        x_start = column * self.cell_pixels
        x_end = x_start + self.cell_pixels

        for y in range(y_start, y_end):
            row_offset = y * self.image_width * 3
            for x in range(x_start, x_end):
                index = row_offset + x * 3
                data[index:index + 3] = bytes(color)

    def draw_grid(self, data: bytearray):
        line_width = 2
        for row in range(self.rows + 1):
            y = min(row * self.cell_pixels, self.image_height - 1)
            for delta in range(line_width):
                yy = y + delta
                if yy >= self.image_height:
                    continue
                start = yy * self.image_width * 3
                data[start:start + self.image_width * 3] = bytes(
                    self.image_width * 3
                )

        for column in range(self.columns + 1):
            x = min(column * self.cell_pixels, self.image_width - 1)
            for delta in range(line_width):
                xx = x + delta
                if xx >= self.image_width:
                    continue
                for y in range(self.image_height):
                    index = (y * self.image_width + xx) * 3
                    data[index:index + 3] = b'\x00\x00\x00'

    def ir_status_to_markers(self, status: IrStatus, stamp) -> MarkerArray:
        """Build live hit, ray, classification, and projected-cell markers."""
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        state_name = GOTO_STATE_NAMES.get(
            status.goto_state, str(status.goto_state)
        )
        freshness = 'fresh' if status.fresh else 'STALE'
        status_marker = self._marker(
            'ir/status', 0, Marker.TEXT_VIEW_FACING, self.robot_frame, stamp
        )
        status_marker.pose.position.z = 0.20
        status_marker.scale.z = 0.035
        status_marker.text = (
            f'IR {freshness}  frame={status.frame}  age={status.age_ms} ms\n'
            f'raw=0x{status.raw_mask:X} confirmed=0x{status.confirmed_mask:X} '
            f'robot=0x{status.robot_mask:X} static=0x{status.static_mask:X} '
            f'goto={state_name}'
        )
        if status.fresh:
            self._set_color(status_marker, (1.0, 1.0, 1.0, 0.95))
        else:
            self._set_color(status_marker, (1.0, 0.15, 0.05, 0.95))
        markers.markers.append(status_marker)

        if not status.fresh:
            return markers

        active_mask = (
            status.raw_mask
            | status.confirmed_mask
            | status.robot_mask
            | status.static_mask
        )
        for index, (name, offset) in enumerate(
            zip(SENSOR_NAMES, SENSOR_OFFSETS_RAD)
        ):
            bit = 1 << index
            if not active_mask & bit:
                continue

            classification, color = self._ir_classification(status, bit)
            hit_x = self.obstacle_distance_m * math.cos(math.pi / 2.0 + offset)
            hit_y = self.obstacle_distance_m * math.sin(math.pi / 2.0 + offset)

            ray = self._marker(
                'ir/rays', index, Marker.ARROW, self.robot_frame, stamp
            )
            ray.points = [
                Point(x=0.0, y=0.0, z=0.03),
                Point(x=hit_x, y=hit_y, z=0.03),
            ]
            ray.scale.x = 0.009
            ray.scale.y = 0.022
            ray.scale.z = 0.025
            self._set_color(ray, color)
            markers.markers.append(ray)

            hit = self._marker(
                'ir/hits', index, Marker.SPHERE, self.robot_frame, stamp
            )
            hit.pose.position.x = hit_x
            hit.pose.position.y = hit_y
            hit.pose.position.z = 0.03
            hit.scale.x = 0.04
            hit.scale.y = 0.04
            hit.scale.z = 0.04
            self._set_color(hit, color)
            markers.markers.append(hit)

            cell = status.cells[index]
            cell_text = 'outside grid'
            if cell is not None:
                row, column = cell
                if 0 <= row < self.rows and 0 <= column < self.columns:
                    cell_x = self.grid_offset_x + column * self.cell_size
                    cell_y = self.grid_offset_y + row * self.cell_size
                    cell_text = (
                        f'cell (r={row}, c={column})\n'
                        f'center=({cell_x:.2f}, {cell_y:.2f}) m'
                    )
                    cell_marker = self._marker(
                        'ir/cells',
                        index,
                        Marker.CUBE,
                        self.world_frame,
                        stamp,
                    )
                    cell_marker.pose.position.x = cell_x
                    cell_marker.pose.position.y = cell_y
                    cell_marker.pose.position.z = self.map_z + 0.012
                    cell_marker.scale.x = self.cell_size * 0.82
                    cell_marker.scale.y = self.cell_size * 0.82
                    cell_marker.scale.z = 0.018
                    self._set_color(
                        cell_marker,
                        (color[0], color[1], color[2], 0.35),
                    )
                    markers.markers.append(cell_marker)

            label = self._marker(
                'ir/labels',
                index,
                Marker.TEXT_VIEW_FACING,
                self.robot_frame,
                stamp,
            )
            label.pose.position.x = hit_x
            label.pose.position.y = hit_y
            label.pose.position.z = 0.10 + index * 0.025
            label.scale.z = 0.028
            label.text = f'{name}: {classification}\n{cell_text}'
            self._set_color(label, color)
            markers.markers.append(label)

        return markers

    def _marker(self, namespace, marker_id, marker_type, frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=self.marker_lifetime_s).to_msg()
        return marker

    @staticmethod
    def _set_color(marker: Marker, color: tuple):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    @staticmethod
    def _ir_classification(status: IrStatus, bit: int) -> tuple:
        if status.robot_mask & bit:
            return 'robot', (0.05, 0.75, 1.0, 0.95)
        if status.static_mask & bit:
            return 'static', (1.0, 0.12, 0.04, 0.95)
        if status.confirmed_mask & bit:
            return 'confirmed', (1.0, 0.45, 0.0, 0.95)
        return 'raw / unconfirmed', (1.0, 0.9, 0.05, 0.65)


def main(args=None):
    rclpy.init(args=args)
    node = DebugMapViz()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
