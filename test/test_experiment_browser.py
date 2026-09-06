import csv

from mechalino_observer.experiment_browser import load_experiments


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
