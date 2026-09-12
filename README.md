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
all selected robots report `GOTO_INACTIVE`. It then sends `cmd=S` to every
selected robot and exits.

For completion detection, each robot must first report a post-start, non-idle
`goto_state`. `goto_state=5` is `GOTO_INACTIVE`; a robot uses it when its
component is complete or when it yields scarce final cells to a closer peer.
`goto_state=4` only means that one route has finished and does not terminate the
experiment. Requiring a post-start sample prevents a cached pre-start response
from ending the experiment immediately.

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
  the latest obstacle maps received from every robot. New runs start with
  `valid=true`; this review flag is independent of the completion status.

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
on `/coverage/percentage` and `/coverage/finished`. At the beginning of a new
run, the supervisor publishes `/coverage_markers/reset`; the persistent marker
node clears its internal history and removes the previous run from RViz.

## Experiment browser

Open the aggregate results in a small Tkinter GUI:

```bash
ros2 run mechalino_observer experiment_browser experiment_results/all_experiments.csv
```

Select a run from the list to see its robot trajectories, aggregated obstacle
cells, status, validity, duration, average speeds, and per-robot speed values.
Use **Browse CSV...** to select a different aggregate results file; the file
picker opens automatically when the default CSV does not exist.
Use **Hide incomplete / failed** to limit the browser list to completed runs.
Use **Export completed + valid...** to save a separate CSV containing only
completed runs that have not been marked invalid.
Use **Mark invalid** to exclude a bad completed run from analysis, and **Mark
valid** to include it again. The choice is saved in `all_experiments.csv`.
Older CSV rows without validity metadata remain loadable and default to valid.
