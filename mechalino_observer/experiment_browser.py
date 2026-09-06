#!/usr/bin/env python3

import argparse
import ast
import csv
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk


GRID_M = 11
GRID_N = 4
GRID_K = 0.15
GRID_X0 = 0.15
GRID_Y0 = 0.15
FIRST_ROBOT_ID = 15
COLORS = [
    '#e63946', '#277da1', '#43aa8b', '#f8961e', '#8338ec',
    '#ff006e', '#00a6a6', '#6a994e', '#bc6c25', '#495057',
]


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


def normalize_obstacles(values):
    cells = set()
    for value in values or []:
        try:
            row, column = (int(coordinate) for coordinate in value)
        except (TypeError, ValueError):
            continue
        if 0 <= row < GRID_N and 0 <= column < GRID_M:
            cells.add((row, column))
    return sorted(cells)


def load_experiments(csv_path):
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))

    experiments = []
    for row in rows:
        experiments.append({
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
            'obstacles': normalize_obstacles(
                _literal(row.get('obstacles'), [])
            ),
        })
    return experiments


class ExperimentBrowser:
    def __init__(self, root, csv_path):
        self.root = root
        self.csv_path = csv_path
        self.experiments = []
        self.selected = None

        root.title('Mechalino Experiment Browser')
        root.geometry('1100x700')
        root.minsize(850, 550)

        top = ttk.Frame(root, padding=10)
        top.pack(fill='x')
        ttk.Label(top, text=str(csv_path)).pack(side='left')
        ttk.Button(top, text='Refresh', command=self.reload).pack(side='right')

        body = ttk.Panedwindow(root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        left = ttk.LabelFrame(body, text='Experiments', padding=6)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=4)

        self.listbox = tk.Listbox(left, width=34, exportselection=False)
        self.listbox.pack(fill='both', expand=True)
        self.listbox.bind('<<ListboxSelect>>', self.select_experiment)

        self.summary = tk.StringVar(value='Select an experiment')
        ttk.Label(
            right, textvariable=self.summary, font=('TkDefaultFont', 11, 'bold')
        ).pack(fill='x', pady=(0, 6))

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

        self.reload()

    def reload(self):
        try:
            self.experiments = load_experiments(self.csv_path)
        except Exception as error:
            messagebox.showerror('Could not open CSV', str(error))
            return

        self.listbox.delete(0, 'end')
        for experiment in reversed(self.experiments):
            self.listbox.insert(
                'end',
                f"{experiment['date_time']}  |  N={experiment['N']}  |  "
                f"{experiment['status']}",
            )

        if self.experiments:
            self.listbox.selection_set(0)
            self.select_experiment()

    def select_experiment(self, event=None):
        del event
        selection = self.listbox.curselection()
        if not selection:
            return
        self.selected = self.experiments[-1 - selection[0]]
        experiment = self.selected
        self.summary.set(
            f"{experiment['status'].upper()}     "
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
        xmin = GRID_X0 - GRID_K / 2.0
        ymin = GRID_Y0 - GRID_K / 2.0
        xmax = xmin + GRID_M * GRID_K
        ymax = ymin + GRID_N * GRID_K
        scale = min(
            (width - 2 * margin) / (xmax - xmin),
            (height - 2 * margin) / (ymax - ymin),
        )
        ox = (width - (xmax - xmin) * scale) / 2.0
        oy = (height - (ymax - ymin) * scale) / 2.0

        def point(x, y):
            return ox + (x - xmin) * scale, height - oy - (y - ymin) * scale

        obstacles = set(self.selected['obstacles'])
        for row in range(GRID_N):
            for column in range(GRID_M):
                x1, y1 = point(xmin + column * GRID_K, ymin + row * GRID_K)
                x2, y2 = point(
                    xmin + (column + 1) * GRID_K,
                    ymin + (row + 1) * GRID_K,
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
                    inset = max(3.0, GRID_K * scale * 0.22)
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
    args = parser.parse_args()
    root = tk.Tk()
    ExperimentBrowser(root, Path(args.csv_file).expanduser().resolve())
    root.mainloop()


if __name__ == '__main__':
    main()
