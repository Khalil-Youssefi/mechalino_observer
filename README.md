# mechalino_observer

## Coverage experiment supervisor

Start the observer launch file first so the `arena -> mechalino_ID` TF frames
are available. The default visualization supports up to ten consecutive robots
(IDs 15 through 24). Start an experiment with the desired `N` beginning at ID
15:

```bash
ros2 launch mechalino_observer experiment.launch.py N:=2
```

The supervisor waits until all selected TF frames are visible, sends `cmd=H`
to the robots in parallel to reset their memory, waits 200 ms, and then sends
`cmd=Q` to start the run. It marks a configured grid cell covered whenever a
robot is inside it. The observer-side grid is progress information only. The
supervisor polls every selected robot's `/debug` endpoint and ends the run when
the first robot reports global coverage completion. It then sends `cmd=S` to
every selected robot and exits.

For completion detection, each robot must first report a non-idle `goto_state`
after the experiment starts. A later transition to `goto_state=0` means the
firmware found no unvisited, non-obstacle cells. `goto_state=4` only means that
one route has finished and does not terminate the experiment. Requiring the
active-to-idle transition prevents a cached pre-start debug response from
ending the experiment immediately.

Cells listed in the `excluded_cells` parameter are omitted from the observer's
progress percentage. The value is a flattened sequence of `(row, column)`
pairs; for example, `[0, 0, 1, 2]` excludes cells `(0, 0)` and `(1, 2)`. The
default configuration excludes `(0, 0)`, which contains the table marker.
Robot-discovered obstacles do not have to be duplicated in this parameter for
completion detection, because the robots decide when coverage is complete.

Results are written under the workspace's `experiment_results` directory by
default, regardless of the terminal's current directory:

- `experiment_<timestamp>_N<N>_<status>.csv` has one row per robot, in ascending
  robot ID order, with `trajectory`, `speed`, `speed_no_stop`, `status`, and
  that robot's latest `obstacles` map.
- `all_experiments.csv` receives one appended summary row for completed and
  manually stopped runs. Its `obstacles` column contains the cell-wise union of
  the latest obstacle maps received from every robot.

Stop a running experiment at any time with Ctrl+C in its terminal or with:

```bash
ros2 service call /experiment/stop std_srvs/srv/Trigger "{}"
```

A manual stop sends `cmd=S` to every selected robot, saves the partial run, and
sets its CSV status to `failed`. A naturally completed run has status
`completed`.

`speed` is total distance divided by the experiment duration.
`speed_no_stop` is total distance divided by time intervals in which movement
was at least `movement_epsilon`.

Live trajectories are also published as
`/robots/mechalino_ID/trajectory` (`nav_msgs/Path`). Coverage state is published
on `/coverage/percentage` and `/coverage/finished`.

## Experiment browser

Open the aggregate results in a small Tkinter GUI:

```bash
ros2 run mechalino_observer experiment_browser experiment_results/all_experiments.csv
```

Select a run from the list to see its robot trajectories, aggregated obstacle
cells, status, duration, average speeds, and per-robot speed values. Older CSV
rows without obstacle data remain loadable and display an empty obstacle map.
