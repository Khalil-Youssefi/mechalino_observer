#!/usr/bin/env python3
"""
Analyze Group A (obstacle-free arena 1) experiment results.

The analyzer independently reconstructs grid coverage from robot trajectories.
Repeated samples while a robot remains in one cell are collapsed into one cell
entry. A small cell-boundary hysteresis suppresses camera jitter.
False-positive obstacles, failed or invalid runs, and binary completeness
classifications are ignored. Valid completed experiments are compared using
elapsed time, physical movement distance, overcoverage, speedup, and parallel
efficiency.
"""

from __future__ import annotations

import argparse
import ast
import csv
import dataclasses
import math
import pathlib
import statistics
import sys
import typing

import yaml


GROUP_A_EXCLUDED_CELLS = frozenset({(0, 0)})
DEFAULT_MOVEMENT_EPSILON_M = 0.005
DEFAULT_CELL_HYSTERESIS_M = 0.010


@dataclasses.dataclass(frozen=True)
class GridConfig:
    """Coverage-grid geometry and cells excluded from Group A coverage."""

    rows: int
    columns: int
    cell_size: float
    offset_x: float
    offset_y: float
    excluded_cells: frozenset[tuple[int, int]]

    @property
    def xmin(self) -> float:
        """Return the x coordinate of the grid's left boundary."""
        return self.offset_x - self.cell_size / 2.0

    @property
    def ymin(self) -> float:
        """Return the y coordinate of the grid's lower boundary."""
        return self.offset_y - self.cell_size / 2.0

    @property
    def coverable_cells(self) -> frozenset[tuple[int, int]]:
        """Return every grid cell that should be covered in Group A."""
        return frozenset(
            (row, column)
            for row in range(self.rows)
            for column in range(self.columns)
            if (row, column) not in self.excluded_cells
        )

    def cell_for_point(self, x: float, y: float) -> tuple[int, int] | None:
        """Map an arena point to a row/column, or return None when outside."""
        column = math.floor((x - self.xmin) / self.cell_size)
        row = math.floor((y - self.ymin) / self.cell_size)
        if 0 <= row < self.rows and 0 <= column < self.columns:
            return row, column
        return None

    def point_is_inside_cell(
        self,
        x: float,
        y: float,
        cell: tuple[int, int],
        margin: float,
    ) -> bool:
        """Return whether a point is inside a cell by at least margin."""
        row, column = cell
        cell_xmin = self.xmin + column * self.cell_size
        cell_ymin = self.ymin + row * self.cell_size
        return (
            cell_xmin + margin <= x < cell_xmin + self.cell_size - margin
            and cell_ymin + margin <= y < cell_ymin + self.cell_size - margin
        )


@dataclasses.dataclass(frozen=True)
class Experiment:
    """One experiment row loaded from all_experiments.csv."""

    n: int
    date_time: str
    total_time_s: float
    trajectories: tuple[tuple[tuple[float, float], ...], ...]
    status: str
    valid: bool = True


@dataclasses.dataclass(frozen=True)
class TrajectoryMetrics:
    """Movement and cell-entry events reconstructed for one robot."""

    entries: tuple[tuple[float, tuple[int, int]], ...]
    distance_m: float
    moving_segments: int


@dataclasses.dataclass(frozen=True)
class RunMetrics:
    """Trajectory-derived coverage and movement measurements for one run."""

    n: int
    date_time: str
    status: str
    reported_time_s: float
    trajectory_count: int
    overvisited_cells: int
    excess_cell_entries: int
    cell_entries: int
    moving_segments: int
    movement_cm: float
    valid: bool = True


@dataclasses.dataclass(frozen=True)
class GroupMetrics:
    """Aggregate Group A measurements for one robot count."""

    n: int
    runs: int
    avg_coverage_time_s: float | None
    avg_overvisited_cells: float | None
    avg_excess_cell_entries: float | None
    avg_movement_cm: float | None
    total_movement_cm: float
    speedup: float | None
    parallel_efficiency: float | None


def default_results_path() -> pathlib.Path:
    """Locate the workspace's aggregate experiment CSV."""
    for parent in pathlib.Path(__file__).resolve().parents:
        if parent.name in ('src', 'install'):
            return parent.parent / 'experiment_results' / 'all_experiments.csv'
    return pathlib.Path.cwd() / 'experiment_results' / 'all_experiments.csv'


def default_params_path() -> pathlib.Path:
    """Locate the grid parameter file in source or installed layouts."""
    source_path = (
        pathlib.Path(__file__).resolve().parent / 'config' / 'params.yaml'
    )
    if source_path.is_file():
        return source_path

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise FileNotFoundError(
            'Could not locate config/params.yaml; pass --params explicitly'
        ) from error

    return (
        pathlib.Path(get_package_share_directory('mechalino_observer'))
        / 'config'
        / 'params.yaml'
    )


def _pairs_from_flat(
    values: typing.Sequence[object], field_name: str
) -> frozenset:
    """Convert a flattened row,column sequence into a cell set."""
    if len(values) % 2:
        raise ValueError(
            f'{field_name} must contain flattened row,column pairs'
        )
    result = set()
    for index in range(0, len(values), 2):
        result.add((int(values[index]), int(values[index + 1])))
    return frozenset(result)


def load_grid_config(params_path: pathlib.Path) -> GridConfig:
    """Load shared grid geometry and excluded cells from ROS parameters."""
    with params_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    parameters = document['/**']['ros__parameters']
    excluded = _pairs_from_flat(
        parameters.get('excluded_cells', [0, 0]), 'excluded_cells'
    )
    grid = GridConfig(
        rows=int(parameters['grid_n']),
        columns=int(parameters['grid_m']),
        cell_size=float(parameters['grid_k']),
        offset_x=float(parameters['grid_offset_x']),
        offset_y=float(parameters['grid_offset_y']),
        excluded_cells=excluded | GROUP_A_EXCLUDED_CELLS,
    )
    if grid.rows <= 0 or grid.columns <= 0 or grid.cell_size <= 0.0:
        raise ValueError(f'Invalid grid geometry in {params_path}')
    return grid


def _literal(value: str, field_name: str, row_number: int):
    """Parse a Python-literal CSV field with a useful row error."""
    try:
        return ast.literal_eval(value)
    except (SyntaxError, ValueError) as error:
        raise ValueError(
            f'CSV row {row_number}: invalid {field_name}: {error}'
        ) from error


def _parse_trajectories(
    value: str, row_number: int
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """Parse all robot x,y trajectories from one aggregate CSV row."""
    raw_trajectories = _literal(value, 'trajectories', row_number)
    trajectories = []
    for robot_index, raw_trajectory in enumerate(raw_trajectories):
        points = []
        for point_index, raw_point in enumerate(raw_trajectory):
            if not isinstance(raw_point, (tuple, list)) or len(raw_point) < 2:
                raise ValueError(
                    f'CSV row {row_number}: robot {robot_index} point '
                    f'{point_index} is not an x,y pair'
                )
            points.append((float(raw_point[0]), float(raw_point[1])))
        trajectories.append(tuple(points))
    return tuple(trajectories)


def load_experiments(csv_path: pathlib.Path) -> list[Experiment]:
    """Load experiments from an aggregate results CSV."""
    with csv_path.open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'N', 'date_time', 'total_time', 'trajectories', 'status'}
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f'{csv_path} is missing columns: {", ".join(sorted(missing))}'
            )

        experiments = []
        for row_number, row in enumerate(reader, start=2):
            validity = (row.get('valid') or 'true').strip().lower()
            if validity not in ('true', 'false'):
                raise ValueError(
                    f'CSV row {row_number}: invalid valid value: '
                    f'{row.get("valid")!r}'
                )
            experiments.append(
                Experiment(
                    n=int(row['N']),
                    date_time=row['date_time'],
                    total_time_s=float(row['total_time']),
                    trajectories=_parse_trajectories(
                        row['trajectories'], row_number
                    ),
                    status=(row['status'] or '').strip().lower(),
                    valid=validity == 'true',
                )
            )
    return experiments


def _sample_time(index: int, point_count: int, total_time_s: float) -> float:
    """Estimate a sample time because stored trajectories lack timestamps."""
    if point_count <= 1:
        return 0.0
    return total_time_s * index / (point_count - 1)


def analyze_trajectory(
    points: typing.Sequence[tuple[float, float]],
    total_time_s: float,
    grid: GridConfig,
    movement_epsilon_m: float,
    cell_hysteresis_m: float,
) -> TrajectoryMetrics:
    """Reconstruct movement and stable cell-entry events for one robot."""
    if not points:
        return TrajectoryMetrics((), 0.0, 0)

    entries: list[tuple[float, tuple[int, int]]] = []
    current_cell = grid.cell_for_point(*points[0])
    if current_cell is not None and current_cell not in grid.excluded_cells:
        entries.append((0.0, current_cell))

    distance_m = 0.0
    moving_segments = 0
    motion_anchor = points[0]
    motion_anchor_time = 0.0
    previous_point = points[0]
    interpolation_step = grid.cell_size / 20.0

    for index, point in enumerate(points[1:], start=1):
        current_time = _sample_time(index, len(points), total_time_s)
        sample_distance = math.dist(previous_point, point)
        if sample_distance >= movement_epsilon_m:
            distance_m += sample_distance
            moving_segments += 1
        previous_point = point

        anchor_distance = math.dist(motion_anchor, point)
        if anchor_distance < movement_epsilon_m:
            continue

        steps = max(1, math.ceil(anchor_distance / interpolation_step))
        for step in range(1, steps + 1):
            fraction = step / steps
            x = motion_anchor[0] + (point[0] - motion_anchor[0]) * fraction
            y = motion_anchor[1] + (point[1] - motion_anchor[1]) * fraction
            candidate = grid.cell_for_point(x, y)
            if candidate is None or candidate == current_cell:
                continue
            if not grid.point_is_inside_cell(
                x, y, candidate, cell_hysteresis_m
            ):
                continue

            current_cell = candidate
            if candidate not in grid.excluded_cells:
                event_time = motion_anchor_time + (
                    current_time - motion_anchor_time
                ) * fraction
                entries.append((event_time, candidate))

        motion_anchor = point
        motion_anchor_time = current_time

    return TrajectoryMetrics(tuple(entries), distance_m, moving_segments)


def analyze_experiment(
    experiment: Experiment,
    grid: GridConfig,
    movement_epsilon_m: float = DEFAULT_MOVEMENT_EPSILON_M,
    cell_hysteresis_m: float = DEFAULT_CELL_HYSTERESIS_M,
) -> RunMetrics:
    """Measure trajectory coverage and movement for one experiment."""
    trajectory_metrics = [
        analyze_trajectory(
            trajectory,
            experiment.total_time_s,
            grid,
            movement_epsilon_m,
            cell_hysteresis_m,
        )
        for trajectory in experiment.trajectories
    ]
    events = sorted(
        event
        for metrics in trajectory_metrics
        for event in metrics.entries
    )

    entry_counts: dict[tuple[int, int], int] = {}
    for _, cell in events:
        entry_counts[cell] = entry_counts.get(cell, 0) + 1
    overvisited_cells = sum(count > 1 for count in entry_counts.values())
    excess_cell_entries = sum(
        max(0, count - 1) for count in entry_counts.values()
    )
    cell_entries = sum(entry_counts.values())

    return RunMetrics(
        n=experiment.n,
        date_time=experiment.date_time,
        status=experiment.status,
        reported_time_s=experiment.total_time_s,
        trajectory_count=len(experiment.trajectories),
        overvisited_cells=overvisited_cells,
        excess_cell_entries=excess_cell_entries,
        cell_entries=cell_entries,
        moving_segments=sum(
            item.moving_segments for item in trajectory_metrics
        ),
        movement_cm=(
            100.0 * sum(item.distance_m for item in trajectory_metrics)
        ),
        valid=experiment.valid,
    )


def _average(values: typing.Iterable[float]) -> float | None:
    """Return an arithmetic mean, or None for an empty population."""
    values = list(values)
    return statistics.fmean(values) if values else None


def aggregate_groups(
    runs: typing.Sequence[RunMetrics],
) -> list[GroupMetrics]:
    """Aggregate completed runs by N and calculate strong-scaling speedup."""
    grouped: dict[int, list[RunMetrics]] = {}
    for run in runs:
        if run.status == 'completed' and run.valid:
            grouped.setdefault(run.n, []).append(run)

    partial = []
    for n in sorted(grouped):
        group_runs = grouped[n]
        partial.append(
            GroupMetrics(
                n=n,
                runs=len(group_runs),
                avg_coverage_time_s=_average(
                    run.reported_time_s for run in group_runs
                ),
                avg_overvisited_cells=_average(
                    float(run.overvisited_cells) for run in group_runs
                ),
                avg_excess_cell_entries=_average(
                    float(run.excess_cell_entries) for run in group_runs
                ),
                avg_movement_cm=_average(
                    run.movement_cm for run in group_runs
                ),
                total_movement_cm=sum(
                    run.movement_cm for run in group_runs
                ),
                speedup=None,
                parallel_efficiency=None,
            )
        )

    if not partial:
        return []
    baseline_time = partial[0].avg_coverage_time_s
    result = []
    baseline_n = partial[0].n
    for group in partial:
        values = dataclasses.asdict(group)
        if baseline_time is not None and group.avg_coverage_time_s:
            values['speedup'] = baseline_time / group.avg_coverage_time_s
            if group.n != baseline_n:
                relative_robot_count = group.n / baseline_n
                values['parallel_efficiency'] = (
                    values['speedup'] / relative_robot_count
                )
        result.append(GroupMetrics(**values))
    return result


def _format(value, digits: int = 2) -> str:
    """Format values for aligned terminal tables."""
    if value is None:
        return 'N/A'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return f'{value:.{digits}f}'
    return str(value)


def _print_table(
    headers: typing.Sequence[str],
    rows: typing.Sequence[typing.Sequence[object]],
) -> None:
    """Print a compact aligned text table."""
    rendered = [[_format(value) for value in row] for row in rows]
    widths = [len(header) for header in headers]
    for row in rendered:
        for index, value in enumerate(row):
            widths[index] = max(widths[index], len(value))

    print(
        '  '.join(
            header.ljust(widths[i]) for i, header in enumerate(headers)
        )
    )
    print('  '.join('-' * width for width in widths))
    for row in rendered:
        print('  '.join(value.rjust(widths[i]) for i, value in enumerate(row)))


def print_report(
    csv_path: pathlib.Path,
    grid: GridConfig,
    groups: typing.Sequence[GroupMetrics],
) -> None:
    """Print time, movement, speedup, and efficiency tables."""
    print('Group A Analysis')
    print(f'Input: {csv_path}')
    print(
        f'Grid: {grid.rows}x{grid.columns}; '
        f'{len(grid.coverable_cells)} coverable cells; '
        f'excluded={sorted(grid.excluded_cells)}'
    )
    print('Population: valid runs reported as completed.')
    print(
        'Failed or invalid runs, obstacle detections, and completeness checks '
        'are ignored.'
    )

    print('\nTime, movement, and overcoverage')
    _print_table(
        (
            'N', 'runs', 'avg time (s)',
            'avg overvisited', 'avg excess entries',
            'avg movement (cm)', 'total movement (cm)',
        ),
        [
            (
                group.n,
                group.runs,
                group.avg_coverage_time_s,
                group.avg_overvisited_cells,
                group.avg_excess_cell_entries,
                group.avg_movement_cm,
                group.total_movement_cm,
            )
            for group in groups
        ],
    )

    print('\nSpeedup (smallest N is the baseline)')
    _print_table(
        ('N', 'avg time (s)', 'speedup', 'parallel efficiency'),
        [
            (
                group.n,
                group.avg_coverage_time_s,
                group.speedup,
                group.parallel_efficiency,
            )
            for group in groups
        ],
    )


def save_charts(
    groups: typing.Sequence[GroupMetrics], output_path: pathlib.Path
) -> None:
    """Save a four-panel summary chart without opening a GUI window."""
    if not groups:
        raise ValueError('No completed experiments are available to plot')

    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    robot_counts = [group.n for group in groups]
    average_times = [group.avg_coverage_time_s or 0.0 for group in groups]
    overvisited = [group.avg_overvisited_cells or 0.0 for group in groups]
    movements_cm = [group.avg_movement_cm or 0.0 for group in groups]
    speedups = [group.speedup or 0.0 for group in groups]
    efficiency_groups = [
        group for group in groups if group.parallel_efficiency is not None
    ]
    efficiency_counts = [group.n for group in efficiency_groups]
    efficiencies = [group.parallel_efficiency for group in efficiency_groups]
    baseline_n = min(robot_counts)
    ideal_speedups = [count / baseline_n for count in robot_counts]

    figure, axes = pyplot.subplots(2, 2, figsize=(12, 8))
    figure.suptitle('Group A Experiment Analysis', fontsize=16)

    axes[0, 0].bar(robot_counts, average_times, color='#2878B5')
    axes[0, 0].set_title('Average coverage time')
    axes[0, 0].set_ylabel('Time (s)')

    axes[0, 1].plot(
        robot_counts,
        speedups,
        marker='o',
        linewidth=2,
        label='Measured',
        color='#7A5195',
    )
    axes[0, 1].plot(
        robot_counts,
        ideal_speedups,
        marker='o',
        linestyle='--',
        label='Ideal',
        color='#777777',
    )
    axes[0, 1].set_title(f'Speedup relative to N={baseline_n}')
    axes[0, 1].set_ylabel('Speedup')
    axes[0, 1].legend()

    movement_axis = axes[1, 0]
    movement_bars = movement_axis.bar(
        [count - 0.18 for count in robot_counts],
        movements_cm,
        width=0.36,
        label='Movement',
        color='#F28E2B',
    )
    overcoverage_axis = movement_axis.twinx()
    overcoverage_bars = overcoverage_axis.bar(
        [count + 0.18 for count in robot_counts],
        overvisited,
        width=0.36,
        label='Overvisited cells',
        color='#E15759',
    )
    movement_axis.set_title('Average movement and overcoverage')
    movement_axis.set_ylabel('Movement per run (cm)')
    overcoverage_axis.set_ylabel('Overvisited cells per run')
    movement_axis.legend(
        (movement_bars, overcoverage_bars),
        ('Movement', 'Overvisited cells'),
    )

    axes[1, 1].bar(efficiency_counts, efficiencies, color='#3BA272')
    axes[1, 1].set_title('Parallel efficiency (speedup / N)')
    axes[1, 1].set_ylabel('Speedup / N')
    axes[1, 1].set_ylim(0.0, 1.05)

    for axis in axes.flat:
        axis.set_xlabel('Robot count (N)')
        axis.set_xticks(robot_counts)
        axis.grid(axis='y', alpha=0.25)
    overcoverage_axis.set_xticks(robot_counts)
    axes[1, 1].set_xticks(efficiency_counts)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches='tight')
    pyplot.close(figure)


def _write_csv(
    path: pathlib.Path, records: typing.Sequence[dict]
) -> None:
    """Write dictionaries to a CSV when the user requests an export."""
    if not records:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def _serializable_run(run: RunMetrics) -> dict:
    """Convert one run to a CSV-ready dictionary."""
    return dataclasses.asdict(run)


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the Group A command-line interface."""
    parser = argparse.ArgumentParser(
        prog='Group A Analysis',
        description='Analyze obstacle-free arena 1 experiment results.',
    )
    parser.add_argument(
        'csv_path',
        nargs='?',
        type=pathlib.Path,
        default=default_results_path(),
    )
    parser.add_argument(
        '--params', type=pathlib.Path, default=default_params_path(),
        help='ROS parameter YAML containing the grid geometry',
    )
    parser.add_argument(
        '--movement-epsilon-m', type=float,
        default=DEFAULT_MOVEMENT_EPSILON_M,
        help='minimum accumulated displacement used for cell-entry tracking',
    )
    parser.add_argument(
        '--cell-hysteresis-m', type=float,
        default=DEFAULT_CELL_HYSTERESIS_M,
        help='distance a robot must enter a new cell before it counts',
    )
    parser.add_argument(
        '--summary-csv', type=pathlib.Path,
        help='optional path for the per-N summary table',
    )
    parser.add_argument(
        '--details-csv', type=pathlib.Path,
        help='optional path for per-run coverage and movement measurements',
    )
    parser.add_argument(
        '--chart-output',
        type=pathlib.Path,
        default=pathlib.Path('group_a_analysis.png'),
        help='PNG dashboard path (default: group_a_analysis.png)',
    )
    return parser


def main(argv: typing.Sequence[str] | None = None) -> int:
    """Run the Group A analysis and optional CSV exports."""
    args = build_argument_parser().parse_args(argv)
    if args.movement_epsilon_m < 0.0:
        raise ValueError('--movement-epsilon-m cannot be negative')

    grid = load_grid_config(args.params)
    if not 0.0 <= args.cell_hysteresis_m < grid.cell_size / 2.0:
        raise ValueError(
            '--cell-hysteresis-m must be non-negative and less than half '
            'a cell'
        )

    experiments = [
        experiment
        for experiment in load_experiments(args.csv_path)
        if experiment.status == 'completed' and experiment.valid
    ]
    if not experiments:
        raise ValueError('No valid completed experiments were found')
    runs = [
        analyze_experiment(
            experiment,
            grid,
            args.movement_epsilon_m,
            args.cell_hysteresis_m,
        )
        for experiment in experiments
    ]
    groups = aggregate_groups(runs)
    print_report(args.csv_path, grid, groups)
    save_charts(groups, args.chart_output)
    print(f'\nCharts: {args.chart_output.resolve()}')

    if args.summary_csv:
        _write_csv(
            args.summary_csv,
            [dataclasses.asdict(group) for group in groups],
        )
    if args.details_csv:
        _write_csv(args.details_csv, [_serializable_run(run) for run in runs])
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (
        FileNotFoundError,
        ImportError,
        KeyError,
        OSError,
        ValueError,
    ) as error:
        print(f'Group A Analysis error: {error}', file=sys.stderr)
        raise SystemExit(2)
