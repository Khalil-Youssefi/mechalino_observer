import csv
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from mechalino_observer.experiment_browser import (
    ExperimentBrowser,
    export_valid_completed_experiments,
    load_experiments,
    load_grid_config,
    set_experiment_validity,
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
    assert experiments[0]['valid'] is True


def test_validity_can_be_added_to_an_old_csv(tmp_path):
    """The first review of an old CSV adds its validity column."""
    csv_path = tmp_path / 'experiments.csv'
    _write_csv(csv_path, BASE_HEADER, BASE_ROW)

    set_experiment_validity(csv_path, 0, False)

    experiments = load_experiments(csv_path)
    assert experiments[0]['valid'] is False
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == BASE_HEADER + ['valid']
    assert rows[1] == BASE_ROW + ['false']


def test_validity_update_changes_only_the_selected_row(tmp_path):
    """Changing validity must address the selected data row only."""
    csv_path = tmp_path / 'experiments.csv'
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(BASE_HEADER + ['valid'])
        writer.writerow(BASE_ROW + ['true'])
        writer.writerow(BASE_ROW + ['false'])

    set_experiment_validity(csv_path, 1, True)

    experiments = load_experiments(csv_path)
    assert [experiment['valid'] for experiment in experiments] == [True, True]


def test_csv_picker_loads_selected_file(tmp_path):
    """Selecting a CSV updates the browser path and reloads its content."""
    csv_path = tmp_path / 'experiments.csv'
    browser = SimpleNamespace(
        root=object(),
        csv_path=None,
        csv_path_text=MagicMock(),
        reload=MagicMock(),
    )

    with patch(
        'mechalino_observer.experiment_browser.filedialog.askopenfilename',
        return_value=str(csv_path),
    ):
        ExperimentBrowser.choose_csv(browser)

    assert browser.csv_path == Path(csv_path).resolve()
    browser.csv_path_text.set.assert_called_once_with(str(csv_path.resolve()))
    browser.reload.assert_called_once_with()


def test_export_contains_only_completed_valid_experiments(tmp_path):
    """CSV export excludes failed and manually invalidated experiments."""
    source_path = tmp_path / 'experiments.csv'
    output_path = tmp_path / 'export.csv'
    with source_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(BASE_HEADER + ['valid'])
        writer.writerow(BASE_ROW + ['true'])
        writer.writerow(BASE_ROW[:-1] + ['failed', 'true'])
        writer.writerow(BASE_ROW + ['false'])

    count = export_valid_completed_experiments(source_path, output_path)

    with output_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))
    assert count == 1
    assert rows == [BASE_HEADER + ['valid'], BASE_ROW + ['true']]


def test_export_adds_validity_to_legacy_csv(tmp_path):
    """Rows from an older aggregate CSV are considered valid by default."""
    source_path = tmp_path / 'experiments.csv'
    output_path = tmp_path / 'export.csv'
    _write_csv(source_path, BASE_HEADER, BASE_ROW)

    count = export_valid_completed_experiments(source_path, output_path)

    with output_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))
    assert count == 1
    assert rows == [BASE_HEADER + ['valid'], BASE_ROW + ['true']]


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
