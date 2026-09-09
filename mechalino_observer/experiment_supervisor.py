#!/usr/bin/env python3

from concurrent.futures import as_completed, ThreadPoolExecutor
import csv
from datetime import datetime
import math
from pathlib import Path as FilesystemPath
import shutil
import time
import urllib.parse
import urllib.request

from geometry_msgs.msg import PoseStamped
from mechalino_observer.debug_map_viz import (
    GOTO_STATE_NAMES,
    parse_debug_obstacles,
    parse_ir_status,
)
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def _default_output_directory():
    for parent in FilesystemPath(__file__).resolve().parents:
        if parent.name in ('src', 'install'):
            return parent.parent / 'experiment_results'
    return FilesystemPath.cwd() / 'experiment_results'


class RobotRun:
    def __init__(self, robot_id, path_publisher):
        self.robot_id = robot_id
        self.path_publisher = path_publisher
        self.path = Path()
        self.path.header.frame_id = 'arena'
        self.trajectory = []
        self.last_stamp_ns = None
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        self.moving_time = 0.0


class ExperimentSupervisor(Node):
    WAITING = 'waiting_for_poses'
    STARTING = 'starting'
    RUNNING = 'running'
    FINISHED = 'finished'
    FAILED = 'failed'

    def __init__(self):
        super().__init__('experiment_supervisor')

        self.declare_parameter('N', 1)
        self.declare_parameter('first_robot_id', 15)
        self.declare_parameter('world_frame', 'arena')
        self.declare_parameter('robot_frame_prefix', 'mechalino_')
        self.declare_parameter('robot_ip_prefix', '192.168.50.')
        self.declare_parameter('http_timeout', 2.0)
        self.declare_parameter('reset_delay_s', 0.2)
        self.declare_parameter('completion_poll_period_s', 1.0)
        self.declare_parameter('sample_hz', 10.0)
        self.declare_parameter('movement_epsilon', 0.005)
        self.declare_parameter(
            'output_directory', str(_default_output_directory())
        )
        self.declare_parameter('all_experiments_filename', 'all_experiments.csv')
        self.declare_parameter('shutdown_on_finish', True)

        # A cell's configured position is its center. These names intentionally
        # match config/params.yaml.
        self.declare_parameter('grid_m', 11)
        self.declare_parameter('grid_n', 4)
        self.declare_parameter('grid_k', 0.15)
        self.declare_parameter('grid_offset_x', 0.15)
        self.declare_parameter('grid_offset_y', 0.15)
        self.declare_parameter('excluded_cells', [0, 0])

        self.n = int(self.get_parameter('N').value)
        self.first_robot_id = int(self.get_parameter('first_robot_id').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame_prefix = str(self.get_parameter('robot_frame_prefix').value)
        self.robot_ip_prefix = str(self.get_parameter('robot_ip_prefix').value)
        self.http_timeout = float(self.get_parameter('http_timeout').value)
        self.reset_delay_s = float(self.get_parameter('reset_delay_s').value)
        self.completion_poll_period_s = float(
            self.get_parameter('completion_poll_period_s').value
        )
        self.sample_hz = float(self.get_parameter('sample_hz').value)
        self.movement_epsilon = float(self.get_parameter('movement_epsilon').value)
        self.output_directory = FilesystemPath(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self.all_experiments_filename = str(
            self.get_parameter('all_experiments_filename').value
        )
        self.shutdown_on_finish = bool(
            self.get_parameter('shutdown_on_finish').value
        )

        self.grid_m = int(self.get_parameter('grid_m').value)
        self.grid_n = int(self.get_parameter('grid_n').value)
        self.grid_k = float(self.get_parameter('grid_k').value)
        self.grid_offset_x = float(self.get_parameter('grid_offset_x').value)
        self.grid_offset_y = float(self.get_parameter('grid_offset_y').value)
        excluded_values = [
            int(value) for value in self.get_parameter('excluded_cells').value
        ]
        if len(excluded_values) % 2 != 0:
            raise ValueError(
                'excluded_cells must contain flattened row/column pairs'
            )
        self.excluded_cells = set(zip(excluded_values[::2], excluded_values[1::2]))

        self._validate_parameters()
        self.coverable_cell_count = (
            self.grid_m * self.grid_n - len(self.excluded_cells)
        )

        self.robot_ids = list(
            range(self.first_robot_id, self.first_robot_id + self.n)
        )
        self.visited = [
            [False for _ in range(self.grid_m)] for _ in range(self.grid_n)
        ]
        self.visited_count = 0
        self.state = self.WAITING
        self.experiment_start_monotonic = None
        self.experiment_datetime = None
        self.result_paths = None
        self._shutdown_timer = None
        self._completion_executor = ThreadPoolExecutor(max_workers=self.n)
        self._completion_futures = {}
        self._completion_armed = set()
        self._completion_last_states = {}
        self._completion_baseline_frames = {}
        self._next_completion_poll = 0.0
        self._last_debug_warning = {}
        self._robot_obstacles = {robot_id: set() for robot_id in self.robot_ids}
        self.completion_reporter_id = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coverage_publisher = self.create_publisher(
            Float32, '/coverage/percentage', latched_qos
        )
        self.finished_publisher = self.create_publisher(
            Bool, '/coverage/finished', latched_qos
        )
        self.marker_reset_publisher = self.create_publisher(
            Empty, '/coverage_markers/reset', latched_qos
        )
        self.stop_service = self.create_service(
            Trigger, '/experiment/stop', self._stop_experiment_callback
        )

        self.robots = {}
        for robot_id in self.robot_ids:
            path_publisher = self.create_publisher(
                Path,
                f'/robots/mechalino_{robot_id}/trajectory',
                latched_qos,
            )
            robot = RobotRun(robot_id, path_publisher)
            robot.path.header.frame_id = self.world_frame
            self.robots[robot_id] = robot

        self._publish_coverage(False)
        self.timer = self.create_timer(1.0 / self.sample_hz, self._timer_callback)
        self.get_logger().info(
            f'Waiting for robot frames {self._robot_frames()} before starting '
            f'a {self.grid_m}x{self.grid_n} cell experiment with '
            f'{self.coverable_cell_count} coverable cells; excluded cells: '
            f'{sorted(self.excluded_cells)}'
        )

    def _validate_parameters(self):
        if self.n < 1:
            raise ValueError('N must be at least 1')
        if self.first_robot_id < 1 or self.first_robot_id + self.n - 1 > 254:
            raise ValueError('Selected robot IDs must be between 1 and 254')
        if self.grid_m < 1 or self.grid_n < 1 or self.grid_k <= 0.0:
            raise ValueError('grid_m, grid_n, and grid_k must be positive')
        for row, column in self.excluded_cells:
            if not (0 <= row < self.grid_n and 0 <= column < self.grid_m):
                raise ValueError(
                    f'Excluded cell ({row}, {column}) is outside the grid'
                )
        if len(self.excluded_cells) >= self.grid_m * self.grid_n:
            raise ValueError('At least one grid cell must be coverable')
        if self.sample_hz <= 0.0:
            raise ValueError('sample_hz must be positive')
        if self.http_timeout <= 0.0:
            raise ValueError('http_timeout must be positive')
        if self.reset_delay_s < 0.0:
            raise ValueError('reset_delay_s cannot be negative')
        if self.completion_poll_period_s <= 0.0:
            raise ValueError('completion_poll_period_s must be positive')
        if self.movement_epsilon < 0.0:
            raise ValueError('movement_epsilon cannot be negative')

    def _robot_frames(self):
        return [f'{self.robot_frame_prefix}{robot_id}' for robot_id in self.robot_ids]

    def _timer_callback(self):
        if self.state == self.WAITING:
            initial_transforms = self._get_all_transforms()
            if initial_transforms is not None:
                self._start_experiment(initial_transforms)
        elif self.state == self.RUNNING:
            self._sample_running_robots()

    def _get_transform(self, robot_id):
        child_frame = f'{self.robot_frame_prefix}{robot_id}'
        try:
            return self.tf_buffer.lookup_transform(
                self.world_frame,
                child_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

    def _get_all_transforms(self):
        transforms = {}
        for robot_id in self.robot_ids:
            transform = self._get_transform(robot_id)
            if transform is None:
                return None
            transforms[robot_id] = transform
        return transforms

    def _start_experiment(self, initial_transforms):
        self._reset_coverage_markers()
        self.get_logger().info(
            f'All robot poses are available; sending H to reset robot memory '
            f'on robots {self.robot_ids}'
        )
        self.state = self.STARTING
        failures = self._send_command_to_all('H')
        if failures:
            self._abort_start('H', failures)
            return

        time.sleep(self.reset_delay_s)
        self.get_logger().info(
            f'Robot memory reset complete; sending Q to robots {self.robot_ids}'
        )
        failures = self._send_command_to_all('Q')
        if failures:
            self._abort_start('Q', failures)
            return

        self.experiment_datetime = datetime.now().astimezone()
        self.experiment_start_monotonic = time.monotonic()
        self._completion_armed.clear()
        self._completion_last_states.clear()
        self._completion_baseline_frames.clear()
        self.completion_reporter_id = None
        self.state = self.RUNNING
        for robot_id, transform in initial_transforms.items():
            self._record_transform(self.robots[robot_id], transform)

        self.get_logger().info(
            f'Experiment started with N={self.n}; observer coverage is progress '
            'only, and the run ends when every selected robot reports inactive'
        )

    def _reset_coverage_markers(self):
        self.marker_reset_publisher.publish(Empty())
        self.get_logger().debug('Requested cleanup of previous coverage markers')

    def _abort_start(self, command, failures):
        details = '; '.join(
            f'{robot_id}: {message}' for robot_id, message in failures.items()
        )
        self.get_logger().error(
            f'Command {command} failed for one or more robots: {details}'
        )
        self.get_logger().warning('Sending S to all selected robots for safety')
        self._send_command_to_all('S')
        self.state = self.FAILED
        self._stop_completion_polling()
        self._schedule_shutdown()

    def _sample_running_robots(self):
        for robot_id in self.robot_ids:
            transform = self._get_transform(robot_id)
            if transform is not None:
                self._record_transform(self.robots[robot_id], transform)

        self._poll_robot_completion()

    def _poll_robot_completion(self):
        """Poll debug endpoints without blocking trajectory sampling."""
        for robot_id, future in list(self._completion_futures.items()):
            if not future.done():
                continue

            del self._completion_futures[robot_id]
            try:
                status, obstacles, obstacle_error = future.result()
            except Exception as error:
                self._warn_debug_poll(robot_id, error)
                continue

            if obstacle_error is not None:
                self._warn_debug_poll(robot_id, obstacle_error)
            else:
                self._store_robot_obstacles(robot_id, obstacles)
            self._handle_robot_completion_status(robot_id, status)
            if self.state != self.RUNNING:
                return

        now = time.monotonic()
        if now < self._next_completion_poll:
            return

        self._next_completion_poll = now + self.completion_poll_period_s
        for robot_id in self.robot_ids:
            if robot_id not in self._completion_futures:
                self._completion_futures[robot_id] = (
                    self._completion_executor.submit(
                        self._fetch_robot_debug_snapshot, robot_id
                    )
                )

    def _fetch_robot_debug_snapshot(self, robot_id):
        url = f'http://{self.robot_ip_prefix}{robot_id}/debug'
        with urllib.request.urlopen(url, timeout=self.http_timeout) as response:
            status_code = response.getcode()
            text = response.read().decode('utf-8', errors='replace')
        if not 200 <= status_code < 300:
            raise RuntimeError(f'HTTP status {status_code}')
        status = parse_ir_status(text)
        try:
            obstacles = parse_debug_obstacles(
                text,
                ('OBSTACLES', 'O'),
                self.grid_n,
                self.grid_m,
            )
            obstacle_error = None
        except ValueError as error:
            obstacles = None
            obstacle_error = f'obstacle map unavailable: {error}'
        return status, obstacles, obstacle_error

    def _store_robot_obstacles(self, robot_id, obstacles):
        self._robot_obstacles[robot_id] = {
            (row, column)
            for row, values in enumerate(obstacles)
            for column, occupied in enumerate(values)
            if occupied
        }

    def _aggregated_obstacles(self):
        cells = set()
        for robot_cells in self._robot_obstacles.values():
            cells.update(robot_cells)
        return sorted(cells)

    def _handle_robot_completion_status(self, robot_id, status):
        # Discard the first cached sample of each run and require a later IR
        # frame. Otherwise an INACTIVE response cached before H could end the
        # next experiment immediately.
        if hasattr(status, 'frame'):
            baseline_frame = self._completion_baseline_frames.get(robot_id)
            if baseline_frame is None:
                self._completion_baseline_frames[robot_id] = status.frame
                return
            if status.frame == baseline_frame:
                return

        previous_state = self._completion_last_states.get(robot_id)
        self._completion_last_states[robot_id] = status.goto_state

        # The cached HTTP response can still contain the pre-Q idle state.
        # Seeing a non-idle Q state first arms the selected robot. DONE (4)
        # means it is replanning; INACTIVE (5) means its flood-fill found no
        # reachable unvisited cell.
        if status.goto_state != 0:
            if robot_id not in self._completion_armed:
                self._completion_armed.add(robot_id)
                state_name = GOTO_STATE_NAMES.get(
                    status.goto_state, str(status.goto_state)
                )
                self.get_logger().info(
                    f'Robot {robot_id} completion reporting armed at '
                    f'goto_state={status.goto_state} ({state_name})'
                )
        if status.goto_state == 5 and previous_state != 5:
            self.get_logger().info(
                f'Robot {robot_id} is inactive: no reachable unvisited cell'
            )

        # One inactive robot must not stop peers that still have reachable work.
        # Terminate only after every selected robot has supplied a post-Q,
        # non-idle state and their latest reports all say INACTIVE.
        if not all(
            selected_id in self._completion_armed
            and self._completion_last_states.get(selected_id) == 5
            for selected_id in self.robot_ids
        ):
            return

        self.completion_reporter_id = 'all selected robots'
        self.get_logger().info(
            'All selected robots are inactive; ending the experiment'
        )
        self._finish_experiment()

    def _warn_debug_poll(self, robot_id, error):
        now = time.monotonic()
        last_warning = self._last_debug_warning.get(robot_id, -math.inf)
        if now - last_warning < 10.0:
            return
        self._last_debug_warning[robot_id] = now
        self.get_logger().warning(
            f'Could not read all robot {robot_id} debug data: {error}'
        )

    def _stop_completion_polling(self):
        if self._completion_executor is None:
            return
        for future in self._completion_futures.values():
            future.cancel()
        self._completion_futures.clear()
        self._completion_executor.shutdown(wait=False, cancel_futures=True)
        self._completion_executor = None

    def _record_transform(self, robot, transform):
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if robot.last_stamp_ns == stamp_ns:
            return

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)

        if robot.last_x is not None and stamp_ns > robot.last_stamp_ns:
            distance = math.hypot(x - robot.last_x, y - robot.last_y)
            if distance >= self.movement_epsilon:
                robot.total_distance += distance
                robot.moving_time += (stamp_ns - robot.last_stamp_ns) / 1e9

        robot.trajectory.append((x, y))
        robot.last_x = x
        robot.last_y = y
        robot.last_stamp_ns = stamp_ns

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = float(transform.transform.translation.z)
        pose.pose.orientation = transform.transform.rotation
        robot.path.header.stamp = transform.header.stamp
        robot.path.poses.append(pose)
        robot.path_publisher.publish(robot.path)

        self._mark_cell(x, y)

    def _mark_cell(self, x, y):
        xmin = self.grid_offset_x - self.grid_k / 2.0
        ymin = self.grid_offset_y - self.grid_k / 2.0
        column = math.floor((x - xmin) / self.grid_k)
        row = math.floor((y - ymin) / self.grid_k)

        if not (0 <= column < self.grid_m and 0 <= row < self.grid_n):
            return
        if (row, column) in self.excluded_cells:
            return
        if self.visited[row][column]:
            return

        self.visited[row][column] = True
        self.visited_count += 1
        percent = 100.0 * self.visited_count / self.coverable_cell_count
        self._publish_coverage(False)
        self.get_logger().info(
            f'Covered cell (row={row}, column={column}): '
            f'{self.visited_count}/{self.coverable_cell_count} ({percent:.1f}%)'
        )
        remaining_cells = self._remaining_cells()
        if len(remaining_cells) <= 5 and remaining_cells:
            self.get_logger().info(f'Remaining cells: {remaining_cells}')

    def _remaining_cells(self):
        return [
            (row, column)
            for row in range(self.grid_n)
            for column in range(self.grid_m)
            if (row, column) not in self.excluded_cells
            and not self.visited[row][column]
        ]

    def _publish_coverage(self, finished):
        percentage = 100.0 * self.visited_count / self.coverable_cell_count
        self.coverage_publisher.publish(Float32(data=percentage))
        self.finished_publisher.publish(Bool(data=finished))

    def _finish_experiment(self):
        self._end_experiment('completed')

    def _log_if_ros_ok(self, level, message):
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)

    def _stop_experiment_callback(self, request, response):
        del request
        if self.state != self.RUNNING:
            response.success = False
            response.message = f'No running experiment; current state is {self.state}'
            return response

        self.get_logger().warning('Manual stop requested; recording run as failed')
        self._end_experiment('failed')
        response.success = True
        if self.result_paths is None:
            response.message = 'Experiment stopped, but CSV output failed'
        else:
            response.message = f'Failed run recorded in {self.result_paths[0]}'
        return response

    def _end_experiment(self, status):
        if self.state != self.RUNNING:
            return

        finish_monotonic = time.monotonic()
        total_time = finish_monotonic - self.experiment_start_monotonic
        self.state = self.FINISHED if status == 'completed' else self.FAILED
        self.timer.cancel()
        self._stop_completion_polling()
        if rclpy.ok():
            self._publish_coverage(status == 'completed')

        if status == 'completed':
            self._log_if_ros_ok(
                'info',
                f'Coverage run ended in {total_time:.3f} s after '
                f'{self.completion_reporter_id} reported inactive; observer tracked '
                f'{self.visited_count}/{self.coverable_cell_count} configured '
                f'cells; sending S to robots {self.robot_ids}',
            )
        else:
            self._log_if_ros_ok(
                'warning',
                f'Experiment stopped after {total_time:.3f} s at '
                f'{self.visited_count}/{self.coverable_cell_count} cells; '
                f'sending S to robots {self.robot_ids}',
            )
        failures = self._send_command_to_all('S')
        for robot_id, message in failures.items():
            self._log_if_ros_ok(
                'error',
                f'Stop command failed for robot {robot_id}: {message}',
            )

        try:
            self.result_paths = self._write_csv_results(total_time, status)
            self._log_if_ros_ok(
                'info',
                f'Per-robot CSV: {self.result_paths[0]}',
            )
            self._log_if_ros_ok(
                'info',
                f'All-experiments CSV: {self.result_paths[1]}',
            )
        except Exception as error:
            self._log_if_ros_ok('error', f'Could not write CSV results: {error}')

        if self.shutdown_on_finish:
            self._schedule_shutdown()

    def _command_url(self, robot_id, command):
        query = urllib.parse.urlencode({'cmd': command})
        return f'http://{self.robot_ip_prefix}{robot_id}/cmd?{query}'

    def _send_command(self, robot_id, command):
        url = self._command_url(robot_id, command)
        with urllib.request.urlopen(url, timeout=self.http_timeout) as response:
            status = response.getcode()
            response.read()
        if not 200 <= status < 300:
            raise RuntimeError(f'HTTP status {status}')

    def _send_command_to_all(self, command):
        failures = {}
        with ThreadPoolExecutor(max_workers=len(self.robot_ids)) as executor:
            futures = {
                executor.submit(self._send_command, robot_id, command): robot_id
                for robot_id in self.robot_ids
            }
            for future in as_completed(futures):
                robot_id = futures[future]
                try:
                    future.result()
                except Exception as error:
                    failures[robot_id] = str(error)
        return failures

    def _write_csv_results(self, total_time, status):
        self.output_directory.mkdir(parents=True, exist_ok=True)
        filename_stamp = self.experiment_datetime.strftime('%Y%m%d_%H%M%S_%f')
        run_path = self.output_directory / (
            f'experiment_{filename_stamp}_N{self.n}_{status}.csv'
        )

        speeds = []
        speeds_no_stop = []
        trajectories = []
        robot_obstacles = []
        for robot_id in self.robot_ids:
            robot = self.robots[robot_id]
            speeds.append(robot.total_distance / total_time if total_time > 0.0 else 0.0)
            speeds_no_stop.append(
                robot.total_distance / robot.moving_time
                if robot.moving_time > 0.0
                else 0.0
            )
            trajectories.append(
                [(round(x, 6), round(y, 6)) for x, y in robot.trajectory]
            )
            robot_obstacles.append(sorted(self._robot_obstacles[robot_id]))

        aggregated_obstacles = self._aggregated_obstacles()

        with run_path.open('w', encoding='utf-8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                ['trajectory', 'speed', 'speed_no_stop', 'status', 'obstacles']
            )
            for trajectory, speed, speed_no_stop, obstacles in zip(
                trajectories, speeds, speeds_no_stop, robot_obstacles
            ):
                writer.writerow(
                    [
                        repr(trajectory),
                        f'{speed:.9f}',
                        f'{speed_no_stop:.9f}',
                        status,
                        repr(obstacles),
                    ]
                )

        all_path = self.output_directory / self.all_experiments_filename
        all_header = [
            'N',
            'date_time',
            'total_time',
            'trajectories',
            'speeds',
            'speeds_no_stop',
            'avg_robots_speeds',
            'avg_robots_no_stop_speeds',
            'status',
            'obstacles',
        ]
        needs_header = self._prepare_all_experiments_csv(all_path, all_header)
        with all_path.open('a', encoding='utf-8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            if needs_header:
                writer.writerow(all_header)
            writer.writerow(
                [
                    self.n,
                    self.experiment_datetime.isoformat(timespec='seconds'),
                    f'{total_time:.9f}',
                    repr(trajectories),
                    repr([round(speed, 9) for speed in speeds]),
                    repr([round(speed, 9) for speed in speeds_no_stop]),
                    f'{sum(speeds) / len(speeds):.9f}',
                    f'{sum(speeds_no_stop) / len(speeds_no_stop):.9f}',
                    status,
                    repr(aggregated_obstacles),
                ]
            )

        return run_path.resolve(), all_path.resolve()

    def _prepare_all_experiments_csv(self, all_path, expected_header):
        if not all_path.exists() or all_path.stat().st_size == 0:
            return True

        with all_path.open('r', encoding='utf-8', newline='') as csv_file:
            rows = list(csv.reader(csv_file))

        if rows[0] == expected_header:
            return False

        header_with_status = expected_header[:-1]
        header_without_status = expected_header[:-2]
        if rows[0] == header_with_status:
            added_values = ['[]']
        elif rows[0] == header_without_status:
            added_values = ['completed', '[]']
        else:
            raise ValueError(
                f'Unexpected CSV header in {all_path}: {rows[0]}'
            )

        backup_path = all_path.with_name(
            f'{all_path.name}.pre_obstacles_backup'
        )
        if not backup_path.exists():
            shutil.copy2(all_path, backup_path)

        temporary_path = all_path.with_name(f'{all_path.name}.tmp')
        with temporary_path.open('w', encoding='utf-8', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(expected_header)
            for row in rows[1:]:
                writer.writerow(row + added_values)
        temporary_path.replace(all_path)
        return False

    def _schedule_shutdown(self):
        if (
            self.shutdown_on_finish
            and self._shutdown_timer is None
            and rclpy.ok()
        ):
            self._shutdown_timer = self.create_timer(0.25, self._shutdown)

    def _shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def stop_robots_if_running(self):
        if self.state == self.RUNNING:
            self._log_if_ros_ok(
                'warning',
                'Experiment interrupted; recording partial run as failed'
            )
            self._end_experiment('failed')
        elif self.state == self.STARTING:
            self._log_if_ros_ok(
                'warning',
                'Experiment interrupted while starting; sending S to all robots'
            )
            self._send_command_to_all('S')
            self.state = self.FAILED
        self._stop_completion_polling()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExperimentSupervisor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_robots_if_running()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
