"""Tests for movement-based Group A experiment analysis."""

import group_a_analysis as analysis

import pytest


def _grid(columns=2):
    return analysis.GridConfig(
        rows=1,
        columns=columns,
        cell_size=1.0,
        offset_x=0.5,
        offset_y=0.5,
        excluded_cells=frozenset(),
    )


def _experiment(n, points, total_time=10.0, status='completed', valid=True):
    return analysis.Experiment(
        n=n,
        date_time=f'run-n{n}',
        total_time_s=total_time,
        trajectories=tuple(tuple(path) for path in points),
        status=status,
        valid=valid,
    )


def test_stationary_samples_are_one_cell_entry():
    """Repeated stationary samples must not count as overcoverage."""
    metrics = analysis.analyze_trajectory(
        [(0.50, 0.50), (0.502, 0.499), (0.498, 0.501)],
        total_time_s=10.0,
        grid=_grid(),
        movement_epsilon_m=0.005,
        cell_hysteresis_m=0.1,
    )

    assert [cell for _, cell in metrics.entries] == [(0, 0)]
    assert metrics.moving_segments == 0
    assert metrics.distance_m == 0.0


def test_reentering_a_cell_counts_overcoverage_once():
    """Leaving and later re-entering a cell is one genuine revisit."""
    run = analysis.analyze_experiment(
        _experiment(
            1,
            [[(0.5, 0.5), (1.5, 0.5), (0.5, 0.5)]],
        ),
        _grid(),
        movement_epsilon_m=0.005,
        cell_hysteresis_m=0.1,
    )

    assert run.overvisited_cells == 1
    assert run.excess_cell_entries == 1
    assert run.cell_entries == 3
    assert run.movement_cm == pytest.approx(200.0)


def test_group_movement_is_reported_in_centimetres():
    """Movement summaries convert trajectory distance to centimetres."""
    run = analysis.analyze_experiment(
        _experiment(1, [[(0.5, 0.5), (1.5, 0.5)]]),
        _grid(),
    )
    group = analysis.aggregate_groups([run])[0]

    assert run.movement_cm == pytest.approx(100.0)
    assert group.avg_movement_cm == pytest.approx(100.0)
    assert group.total_movement_cm == pytest.approx(100.0)


def test_speedup_uses_smallest_n_average_recorded_time():
    """Speedup uses completed-run time from the smallest N."""
    runs = [
        analysis.analyze_experiment(
            _experiment(1, [[(0.5, 0.5), (1.5, 0.5)]], 20.0),
            _grid(),
        ),
        analysis.analyze_experiment(
            _experiment(
                2,
                [[(0.5, 0.5)], [(0.5, 0.5), (1.5, 0.5)]],
                10.0,
            ),
            _grid(),
        ),
    ]

    groups = analysis.aggregate_groups(runs)

    assert groups[0].avg_coverage_time_s == pytest.approx(
        2.0 * groups[1].avg_coverage_time_s
    )
    assert groups[0].speedup == pytest.approx(1.0)
    assert groups[1].speedup == pytest.approx(2.0)
    assert groups[0].parallel_efficiency is None
    assert groups[1].parallel_efficiency == pytest.approx(1.0)


def test_noncompleted_runs_are_not_included_in_group_averages():
    """Only normally completed experiments enter performance summaries."""
    completed = analysis.analyze_experiment(
        _experiment(1, [[(0.5, 0.5)]], total_time=20.0),
        _grid(),
    )
    stopped = analysis.analyze_experiment(
        _experiment(
            1,
            [[(0.5, 0.5)]],
            total_time=1.0,
            status='failed',
        ),
        _grid(),
    )

    group = analysis.aggregate_groups([completed, stopped])[0]

    assert group.runs == 1
    assert group.avg_coverage_time_s == 20.0


def test_invalid_completed_runs_are_not_included_in_group_averages():
    """A manually rejected completed run must not affect results."""
    valid = analysis.analyze_experiment(
        _experiment(1, [[(0.5, 0.5)]], total_time=20.0),
        _grid(),
    )
    invalid = analysis.analyze_experiment(
        _experiment(
            1,
            [[(0.5, 0.5)]],
            total_time=1.0,
            valid=False,
        ),
        _grid(),
    )

    group = analysis.aggregate_groups([valid, invalid])[0]

    assert group.runs == 1
    assert group.avg_coverage_time_s == 20.0


def test_summary_chart_is_written(tmp_path):
    """The plotter writes a non-empty PNG dashboard."""
    run = analysis.analyze_experiment(
        _experiment(1, [[(0.5, 0.5), (1.5, 0.5)]]),
        _grid(),
    )
    output_path = tmp_path / 'charts.png'

    analysis.save_charts(analysis.aggregate_groups([run]), output_path)

    assert output_path.read_bytes().startswith(b'\x89PNG')
