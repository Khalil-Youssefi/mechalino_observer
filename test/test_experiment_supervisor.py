import csv
from types import SimpleNamespace
from unittest.mock import call, MagicMock, patch

from mechalino_observer.experiment_supervisor import ExperimentSupervisor
from std_msgs.msg import Bool, Float32


def _startup_supervisor(command_results):
    supervisor = SimpleNamespace(
        STARTING='starting',
        RUNNING='running',
        n=2,
        reset_delay_s=0.2,
        robot_ids=[15, 16],
        robots={15: object(), 16: object()},
        _completion_armed=set(),
        _completion_last_states={},
        _completion_baseline_frames={},
        completion_reporter_id=None,
    )
    supervisor.get_logger = MagicMock(return_value=MagicMock())
    supervisor._send_command_to_all = MagicMock(side_effect=command_results)
    supervisor._abort_start = MagicMock()
    supervisor._record_transform = MagicMock()
    supervisor._reset_coverage_markers = MagicMock()
    return supervisor


def test_start_resets_robot_memory_before_start_command():
    supervisor = _startup_supervisor([{}, {}])
    transforms = {15: object(), 16: object()}

    with patch(
        'mechalino_observer.experiment_supervisor.time.sleep'
    ) as sleep:
        ExperimentSupervisor._start_experiment(supervisor, transforms)

    assert supervisor._send_command_to_all.call_args_list == [
        call('H'),
        call('Q'),
    ]
    sleep.assert_called_once_with(0.2)
    supervisor._abort_start.assert_not_called()
    supervisor._reset_coverage_markers.assert_called_once_with()
    assert supervisor.state == supervisor.RUNNING


def test_failed_reset_does_not_send_start_command():
    failures = {15: 'timed out'}
    supervisor = _startup_supervisor([failures])

    with patch(
        'mechalino_observer.experiment_supervisor.time.sleep'
    ) as sleep:
        ExperimentSupervisor._start_experiment(supervisor, {})

    supervisor._send_command_to_all.assert_called_once_with('H')
    sleep.assert_not_called()
    supervisor._abort_start.assert_called_once_with('H', failures)


def _completion_supervisor():
    supervisor = SimpleNamespace(
        robot_ids=[15, 16],
        _completion_armed=set(),
        _completion_last_states={},
        _completion_baseline_frames={},
        completion_reporter_id=None,
    )
    supervisor.get_logger = MagicMock(return_value=MagicMock())
    supervisor._finish_experiment = MagicMock()
    return supervisor


def test_one_inactive_robot_does_not_end_experiment():
    supervisor = _completion_supervisor()

    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(goto_state=5)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 16, SimpleNamespace(goto_state=2)
    )

    supervisor._finish_experiment.assert_not_called()


def test_all_selected_robots_inactive_end_experiment():
    supervisor = _completion_supervisor()

    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(goto_state=5)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 16, SimpleNamespace(goto_state=5)
    )

    supervisor._finish_experiment.assert_called_once_with()
    assert supervisor.completion_reporter_id == 'all selected robots'


def test_robot_can_reactivate_before_everyone_becomes_inactive():
    supervisor = _completion_supervisor()

    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(goto_state=5)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(goto_state=1)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 16, SimpleNamespace(goto_state=5)
    )

    supervisor._finish_experiment.assert_not_called()


def test_cached_inactive_samples_do_not_end_new_run():
    supervisor = _completion_supervisor()

    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(frame=10, goto_state=5)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 16, SimpleNamespace(frame=20, goto_state=5)
    )
    supervisor._finish_experiment.assert_not_called()

    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 15, SimpleNamespace(frame=11, goto_state=5)
    )
    ExperimentSupervisor._handle_robot_completion_status(
        supervisor, 16, SimpleNamespace(frame=21, goto_state=5)
    )
    supervisor._finish_experiment.assert_called_once_with()


def test_finished_experiment_publishes_actual_coverage():
    supervisor = SimpleNamespace(
        visited_count=30,
        coverable_cell_count=40,
        coverage_publisher=MagicMock(),
        finished_publisher=MagicMock(),
    )

    ExperimentSupervisor._publish_coverage(supervisor, True)

    supervisor.coverage_publisher.publish.assert_called_once_with(
        Float32(data=75.0)
    )
    supervisor.finished_publisher.publish.assert_called_once_with(
        Bool(data=True)
    )


def test_aggregated_obstacles_are_the_union_of_latest_robot_maps():
    supervisor = SimpleNamespace(
        _robot_obstacles={15: set(), 16: set()},
    )
    ExperimentSupervisor._store_robot_obstacles(
        supervisor,
        15,
        [[1, 0, 1], [0, 0, 0]],
    )
    ExperimentSupervisor._store_robot_obstacles(
        supervisor,
        16,
        [[0, 1, 1], [0, 0, 1]],
    )

    assert ExperimentSupervisor._aggregated_obstacles(supervisor) == [
        (0, 0),
        (0, 1),
        (0, 2),
        (1, 2),
    ]


def test_debug_snapshot_contains_completion_and_obstacle_data():
    response = MagicMock()
    response.getcode.return_value = 200
    response.read.return_value = (
        b'#IR=7,0,1,0,0,0,0,2#O=1,2,0,400'
    )
    response.__enter__.return_value = response
    supervisor = SimpleNamespace(
        robot_ip_prefix='192.168.50.',
        http_timeout=2.0,
        grid_n=4,
        grid_m=11,
    )

    with patch(
        'mechalino_observer.experiment_supervisor.urllib.request.urlopen',
        return_value=response,
    ):
        status, obstacles, error = (
            ExperimentSupervisor._fetch_robot_debug_snapshot(supervisor, 15)
        )

    assert status.goto_state == 2
    assert error is None
    assert obstacles[0][0] == 1
    assert obstacles[1][1] == 1
    assert obstacles[3][10] == 1


def test_existing_summary_csv_is_migrated_with_empty_obstacles(tmp_path):
    csv_path = tmp_path / 'all_experiments.csv'
    old_header = [
        'N',
        'date_time',
        'total_time',
        'trajectories',
        'speeds',
        'speeds_no_stop',
        'avg_robots_speeds',
        'avg_robots_no_stop_speeds',
        'status',
    ]
    old_row = ['1', 'date', '1.0', '[]', '[]', '[]', '0', '0', 'completed']
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(old_header)
        writer.writerow(old_row)

    needs_header = ExperimentSupervisor._prepare_all_experiments_csv(
        SimpleNamespace(),
        csv_path,
        old_header + ['obstacles'],
    )

    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))
    assert needs_header is False
    assert rows == [old_header + ['obstacles'], old_row + ['[]']]
    assert csv_path.with_name(
        'all_experiments.csv.pre_obstacles_backup'
    ).exists()
