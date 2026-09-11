import csv

from mechalino_observer.experiment_browser import (
    load_experiments,
    load_grid_config,
)


BASE_HEADER = [
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
BASE_ROW = [
    '1',
    '2026-09-06T12:00:00+02:00',
    '1.5',
    '[[(0.1, 0.2)]]',
    '[0.1]',
    '[0.2]',
    '0.1',
    '0.2',
    'completed',
]


def _write_csv(path, header, row):
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerow(row)


def test_loads_aggregated_obstacles(tmp_path):
    csv_path = tmp_path / 'experiments.csv'
    _write_csv(
        csv_path,
        BASE_HEADER + ['obstacles'],
        BASE_ROW + ['[(0, 2), (1, 3), (1, 3)]'],
    )

    experiments = load_experiments(csv_path)

    assert experiments[0]['obstacles'] == [(0, 2), (1, 3)]


def test_old_csv_without_obstacles_remains_loadable(tmp_path):
    csv_path = tmp_path / 'experiments.csv'
    _write_csv(csv_path, BASE_HEADER, BASE_ROW)

    experiments = load_experiments(csv_path)

    assert experiments[0]['obstacles'] == []


def test_loads_grid_geometry_from_ros_parameters(tmp_path):
    params_path = tmp_path / 'params.yaml'
    params_path.write_text(
        """/**:
  ros__parameters:
    grid_m: 7
    grid_n: 3
    grid_k: 0.2
    grid_offset_x: 0.18
    grid_offset_y: 0.28
""",
        encoding='utf-8',
    )

    assert load_grid_config(params_path) == {
        'columns': 7,
        'rows': 3,
        'cell_size': 0.2,
        'offset_x': 0.18,
        'offset_y': 0.28,
    }


def test_obstacle_validation_uses_configured_grid_size(tmp_path):
    csv_path = tmp_path / 'experiments.csv'
    _write_csv(
        csv_path,
        BASE_HEADER + ['obstacles'],
        BASE_ROW + ['[(0, 0), (1, 2), (1, 3), (2, 0)]'],
    )
    grid = {
        'columns': 3,
        'rows': 2,
        'cell_size': 0.15,
        'offset_x': 0.18,
        'offset_y': 0.28,
    }

    experiments = load_experiments(csv_path, grid)

    assert experiments[0]['obstacles'] == [(0, 0), (1, 2)]
