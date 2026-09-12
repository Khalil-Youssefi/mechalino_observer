#!/usr/bin/env python3

import argparse
import ast
import csv
import os
import tempfile
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import yaml

FIRST_ROBOT_ID = 15
COLORS = [
    '#e63946', '#277da1', '#43aa8b', '#f8961e', '#8338ec',
    '#ff006e', '#00a6a6', '#6a994e', '#bc6c25', '#495057',
]


def default_params_path():
    source_path = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
    if source_path.is_file():
        return source_path

    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory('mechalino_observer'))
        / 'config'
        / 'params.yaml'
    )


def load_grid_config(params_path=None):
    path = Path(params_path) if params_path else default_params_path()
    with path.open(encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']

    grid = {
        'columns': int(parameters['grid_m']),
        'rows': int(parameters['grid_n']),
        'cell_size': float(parameters['grid_k']),
        'offset_x': float(parameters['grid_offset_x']),
        'offset_y': float(parameters['grid_offset_y']),
    }
    if grid['columns'] < 1 or grid['rows'] < 1 or grid['cell_size'] <= 0.0:
        raise ValueError(f'Invalid grid configuration in {path}')
    return grid


def default_csv_path():
    for parent in Path(__file__).resolve().parents:
        if parent.name in ('src', 'install'):
            return parent.parent / 'experiment_results' / 'all_experiments.csv'
    return Path.cwd() / 'experiment_results' / 'all_experiments.csv'


def _literal(value, default):
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError, TypeError):
        return default


def _validity(value):
    if value is None or not str(value).strip():
        return True

    normalized = str(value).strip().lower()
    if normalized in ('true', '1', 'yes', 'valid'):
        return True
    if normalized in ('false', '0', 'no', 'invalid'):
        return False
    raise ValueError(f'Invalid valid value: {value!r}')


def normalize_obstacles(values, grid):
    cells = set()
    for value in values or []:
        try:
            row, column = (int(coordinate) for coordinate in value)
        except (TypeError, ValueError):
            continue
        if 0 <= row < grid['rows'] and 0 <= column < grid['columns']:
            cells.add((row, column))
    return sorted(cells)


def load_experiments(csv_path, grid=None):
    grid = grid or load_grid_config()
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))

    experiments = []
    for row_index, row in enumerate(rows):
        experiments.append({
            'row_index': row_index,
            'N': int(row.get('N') or 0),
            'date_time': row.get('date_time', ''),
            'total_time': float(row.get('total_time') or 0.0),
            'trajectories': _literal(row.get('trajectories'), []),
            'speeds': _literal(row.get('speeds'), []),
            'speeds_no_stop': _literal(row.get('speeds_no_stop'), []),
            'avg_speed': float(row.get('avg_robots_speeds') or 0.0),
            'avg_speed_no_stop': float(
                row.get('avg_robots_no_stop_speeds') or 0.0
            ),
            'status': row.get('status') or 'completed',
            'valid': _validity(row.get('valid')),
            'obstacles': normalize_obstacles(
                _literal(row.get('obstacles'), []), grid
            ),
        })
    return experiments


def set_experiment_validity(csv_path, row_index, valid):
    """Persist review validity for one data row in an aggregate CSV."""
    csv_path = Path(csv_path)
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))

    if not rows:
        raise ValueError(f'{csv_path} is empty')
    if row_index < 0 or row_index >= len(rows) - 1:
        raise IndexError(f'Experiment row {row_index} no longer exists')

    header = rows[0]
    if 'valid' in header:
        valid_column = header.index('valid')
    else:
        valid_column = len(header)
        header.append('valid')
        for row in rows[1:]:
            row.extend([''] * (valid_column - len(row)))
            row.append('true')

    target = rows[row_index + 1]
    target.extend([''] * (valid_column + 1 - len(target)))
    target[valid_column] = 'true' if valid else 'false'

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            newline='',
            prefix=f'.{csv_path.name}.',
            suffix='.tmp',
            dir=csv_path.parent,
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            writer = csv.writer(stream)
            writer.writerows(rows)
        os.chmod(temporary_path, csv_path.stat().st_mode)
        temporary_path.replace(csv_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def export_valid_completed_experiments(csv_path, output_path):
    """Export only completed experiments that have not been marked invalid."""
    csv_path = Path(csv_path)
    output_path = Path(output_path)
    if csv_path.resolve() == output_path.resolve():
        raise ValueError('Export to a different file than the source CSV')

    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.reader(stream))
    if not rows:
        raise ValueError(f'{csv_path} is empty')

    source_header = rows[0]
    header = list(source_header)
    status_column = (
        source_header.index('status') if 'status' in source_header else None
    )
    if 'valid' in source_header:
        valid_column = source_header.index('valid')
    else:
        valid_column = len(header)
        header.append('valid')

    exported_rows = []
    for source_row in rows[1:]:
        status = (
            source_row[status_column]
            if status_column is not None and status_column < len(source_row)
            else 'completed'
        )
        raw_validity = (
            source_row[valid_column]
            if valid_column < len(source_row)
            else None
        )
        if status.strip().lower() != 'completed' or not _validity(raw_validity):
            continue

        row = list(source_row)
        row.extend([''] * (len(header) - len(row)))
        row[valid_column] = 'true'
        exported_rows.append(row)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(exported_rows)
    return len(exported_rows)


class ExperimentBrowser:
    def __init__(self, root, csv_path, grid=None):
        self.root = root
        self.csv_path = Path(csv_path) if csv_path else None
        self.grid = grid or load_grid_config()
        self.experiments = []
        self.visible_experiments = []
        self.selected = None

        root.title('Mechalino Experiment Browser')
        root.geometry('1100x700')
        root.minsize(850, 550)

        top = ttk.Frame(root, padding=10)
        top.pack(fill='x')
        ttk.Button(top, text='Refresh', command=self.reload).pack(side='right')
        ttk.Button(
            top, text='Browse CSV...', command=self.choose_csv
        ).pack(side='right', padx=(0, 6))
        ttk.Button(
            top,
            text='Export completed + valid...',
            command=self.export_csv,
        ).pack(side='right', padx=(0, 6))
        self.csv_path_text = tk.StringVar(
            value=str(self.csv_path) if self.csv_path else 'No CSV selected'
        )
        ttk.Label(top, textvariable=self.csv_path_text).pack(
            side='left', fill='x', expand=True
        )

        body = ttk.Panedwindow(root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        left = ttk.LabelFrame(body, text='Experiments', padding=6)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=4)

        self.hide_incomplete = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            left,
            text='Hide incomplete / failed',
            variable=self.hide_incomplete,
            command=self.apply_filters,
        ).pack(fill='x', pady=(0, 6))

        self.listbox = tk.Listbox(left, width=34, exportselection=False)
        self.listbox.pack(fill='both', expand=True)
        self.listbox.bind('<<ListboxSelect>>', self.select_experiment)

        self.summary = tk.StringVar(value='Select an experiment')
        ttk.Label(
            right,
            textvariable=self.summary,
            font=('TkDefaultFont', 11, 'bold'),
        ).pack(fill='x', pady=(0, 6))

        review_bar = ttk.LabelFrame(right, text='Experiment validity', padding=6)
        review_bar.pack(fill='x', pady=(0, 8))
        ttk.Label(
            review_bar,
            text='Review the selected experiment:',
        ).pack(side='left')
        ttk.Button(
            review_bar,
            text='MARK VALID',
            width=16,
            command=lambda: self.mark_selected(True),
        ).pack(side='left', padx=(12, 6))
        ttk.Button(
            review_bar,
            text='MARK INVALID',
            width=16,
            command=lambda: self.mark_selected(False),
        ).pack(side='left')

        self.canvas = tk.Canvas(right, background='#fbfbfd', highlightthickness=1)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda event: self.draw())

        self.table = ttk.Treeview(
            right,
            columns=('robot', 'points', 'speed', 'moving_speed'),
            show='headings',
            height=6,
        )
        headings = {
            'robot': 'Robot',
            'points': 'Trajectory points',
            'speed': 'Speed (m/s)',
            'moving_speed': 'Speed, no stop (m/s)',
        }
        for column, heading in headings.items():
            self.table.heading(column, text=heading)
            self.table.column(column, anchor='center', width=130)
        self.table.pack(fill='x', pady=(8, 0))

        if self.csv_path is not None and self.csv_path.is_file():
            self.reload()
        else:
            root.after_idle(self.choose_csv)

    def choose_csv(self):
        """Let the user select an aggregate experiment CSV file."""
        candidate = self.csv_path or default_csv_path()
        initial_directory = candidate.parent
        if not initial_directory.is_dir():
            initial_directory = Path.cwd()
        selected_path = filedialog.askopenfilename(
            parent=self.root,
            title='Select experiment results CSV',
            initialdir=str(initial_directory),
            filetypes=(('CSV files', '*.csv'), ('All files', '*')),
        )
        if not selected_path:
            return

        self.csv_path = Path(selected_path).resolve()
        self.csv_path_text.set(str(self.csv_path))
        self.reload()

    def export_csv(self):
        """Prompt for a destination and export completed, valid rows."""
        if self.csv_path is None:
            messagebox.showwarning('No CSV selected', 'Select a CSV first')
            return

        output_path = filedialog.asksaveasfilename(
            parent=self.root,
            title='Export completed and valid experiments',
            initialdir=str(self.csv_path.parent),
            initialfile=f'{self.csv_path.stem}_completed_valid.csv',
            defaultextension='.csv',
            filetypes=(('CSV files', '*.csv'), ('All files', '*')),
        )
        if not output_path:
            return

        try:
            count = export_valid_completed_experiments(
                self.csv_path, Path(output_path)
            )
        except Exception as error:
            messagebox.showerror('Could not export CSV', str(error))
            return
        messagebox.showinfo(
            'Export complete',
            f'Exported {count} completed, valid experiments to:\n{output_path}',
        )

    def reload(self, selected_row_index=None):
        if self.csv_path is None:
            return
        try:
            self.experiments = load_experiments(self.csv_path, self.grid)
        except Exception as error:
            messagebox.showerror('Could not open CSV', str(error))
            return

        self.apply_filters(selected_row_index)

    def apply_filters(self, selected_row_index=None):
        """Update the list according to the incomplete-run filter."""
        if self.hide_incomplete.get():
            self.visible_experiments = [
                experiment
                for experiment in self.experiments
                if experiment['status'].strip().lower() == 'completed'
            ]
        else:
            self.visible_experiments = list(self.experiments)

        self.listbox.delete(0, 'end')
        for experiment in reversed(self.visible_experiments):
            validity = 'VALID' if experiment['valid'] else 'INVALID'
            self.listbox.insert(
                'end',
                f"{experiment['date_time']}  |  N={experiment['N']}  |  "
                f"{experiment['status']}  |  {validity}",
            )

        if self.visible_experiments:
            selected_index = 0
            if selected_row_index is not None:
                for index, experiment in enumerate(
                    reversed(self.visible_experiments)
                ):
                    if experiment['row_index'] == selected_row_index:
                        selected_index = index
                        break
            self.listbox.selection_set(selected_index)
            self.select_experiment()
        else:
            self.selected = None
            self.summary.set('No matching experiments')
            for item in self.table.get_children():
                self.table.delete(item)
            self.draw()

    def mark_selected(self, valid):
        """Persist validity for the selected experiment and refresh the UI."""
        if self.selected is None:
            return

        row_index = self.selected['row_index']
        try:
            set_experiment_validity(self.csv_path, row_index, valid)
        except Exception as error:
            messagebox.showerror('Could not update CSV', str(error))
            return
        self.reload(selected_row_index=row_index)

    def select_experiment(self, event=None):
        del event
        selection = self.listbox.curselection()
        if not selection:
            return
        self.selected = self.visible_experiments[-1 - selection[0]]
        experiment = self.selected
        validity = 'VALID' if experiment['valid'] else 'INVALID'
        self.summary.set(
            f"{validity}     {experiment['status'].upper()}     "
            f"Time: {experiment['total_time']:.2f} s     "
            f"Robots: {experiment['N']}     "
            f"Obstacles: {len(experiment['obstacles'])}     "
            f"Average: {experiment['avg_speed']:.3f} m/s     "
            f"Moving average: {experiment['avg_speed_no_stop']:.3f} m/s"
        )

        for item in self.table.get_children():
            self.table.delete(item)
        for index in range(experiment['N']):
            trajectory = self._value(experiment['trajectories'], index, [])
            speed = self._value(experiment['speeds'], index, 0.0)
            moving_speed = self._value(experiment['speeds_no_stop'], index, 0.0)
            self.table.insert('', 'end', values=(
                FIRST_ROBOT_ID + index,
                len(trajectory),
                f'{float(speed):.4f}',
                f'{float(moving_speed):.4f}',
            ))
        self.draw()

    @staticmethod
    def _value(values, index, default):
        return values[index] if index < len(values) else default

    def draw(self):
        self.canvas.delete('all')
        if self.selected is None:
            return

        width = max(self.canvas.winfo_width(), 400)
        height = max(self.canvas.winfo_height(), 250)
        margin = 45
        columns = self.grid['columns']
        rows = self.grid['rows']
        cell_size = self.grid['cell_size']
        xmin = self.grid['offset_x'] - cell_size / 2.0
        ymin = self.grid['offset_y'] - cell_size / 2.0
        xmax = xmin + columns * cell_size
        ymax = ymin + rows * cell_size
        scale = min(
            (width - 2 * margin) / (xmax - xmin),
            (height - 2 * margin) / (ymax - ymin),
        )
        ox = (width - (xmax - xmin) * scale) / 2.0
        oy = (height - (ymax - ymin) * scale) / 2.0

        def point(x, y):
            return ox + (x - xmin) * scale, height - oy - (y - ymin) * scale

        obstacles = set(self.selected['obstacles'])
        for row in range(rows):
            for column in range(columns):
                x1, y1 = point(
                    xmin + column * cell_size,
                    ymin + row * cell_size,
                )
                x2, y2 = point(
                    xmin + (column + 1) * cell_size,
                    ymin + (row + 1) * cell_size,
                )
                cell = (row, column)
                if cell in obstacles:
                    fill = '#f8c5c5'
                    outline = '#b42318'
                elif cell == (0, 0):
                    fill = '#d5d7dc'
                    outline = '#9da3ae'
                else:
                    fill = '#ffffff'
                    outline = '#ccd0d8'
                self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill=fill, outline=outline
                )
                if cell in obstacles:
                    inset = max(3.0, cell_size * scale * 0.22)
                    left, right = sorted((x1, x2))
                    top, bottom = sorted((y1, y2))
                    self.canvas.create_line(
                        left + inset,
                        top + inset,
                        right - inset,
                        bottom - inset,
                        fill='#b42318',
                        width=2,
                    )
                    self.canvas.create_line(
                        left + inset,
                        bottom - inset,
                        right - inset,
                        top + inset,
                        fill='#b42318',
                        width=2,
                    )

        for index, trajectory in enumerate(self.selected['trajectories']):
            coordinates = []
            for sample in trajectory:
                if len(sample) >= 2:
                    coordinates.extend(point(float(sample[0]), float(sample[1])))
            color = COLORS[index % len(COLORS)]
            if len(coordinates) >= 4:
                self.canvas.create_line(*coordinates, fill=color, width=3,
                                        smooth=True)
            if len(coordinates) >= 2:
                self.canvas.create_oval(
                    coordinates[0] - 5, coordinates[1] - 5,
                    coordinates[0] + 5, coordinates[1] + 5,
                    fill=color, outline='white', width=2,
                )
                self.canvas.create_rectangle(
                    coordinates[-2] - 5, coordinates[-1] - 5,
                    coordinates[-2] + 5, coordinates[-1] + 5,
                    fill='white', outline=color, width=3,
                )
                self.canvas.create_text(
                    ox + 10 + (index % 5) * 110,
                    18 + (index // 5) * 18,
                    text=f'Robot {FIRST_ROBOT_ID + index}', fill=color,
                    anchor='w', font=('TkDefaultFont', 10, 'bold'),
                )


def main():
    parser = argparse.ArgumentParser(description='Browse Mechalino experiment CSV')
    parser.add_argument(
        'csv_file', nargs='?', default=str(default_csv_path())
    )
    parser.add_argument(
        '--params-file', default=str(default_params_path()),
        help='ROS parameter file containing the shared grid geometry',
    )
    args = parser.parse_args()
    root = tk.Tk()
    ExperimentBrowser(
        root,
        Path(args.csv_file).expanduser().resolve(),
        load_grid_config(Path(args.params_file).expanduser().resolve()),
    )
    root.mainloop()


if __name__ == '__main__':
    main()
