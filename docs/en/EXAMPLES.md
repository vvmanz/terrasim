# Examples: copy, run, change

[Русская версия](../ru/EXAMPLES.md)

Every command on this page was run as written. Start from the one closest to what you need and change it. All
commands are run from the project folder (`cd ~/terrasim`). Without `--engine` they use the emulator (seconds, no
Gazebo); add `--engine harmonic --gui` to any `drive.sh` command to watch the same run in Gazebo.

Contents:
1. [First runs and where the results are](#1-first-runs-and-where-the-results-are)
2. [Robot + map + trajectory + parameters](#2-robot--map--trajectory--parameters)
3. [Your own robot, step by step](#3-your-own-robot-step-by-step)
4. [Your own surface, step by step](#4-your-own-surface-step-by-step)
5. [Your own map, step by step](#5-your-own-map-step-by-step)
6. [Everything of your own in one run](#6-everything-of-your-own-in-one-run)
7. [Falls and rollovers](#7-falls-and-rollovers)
8. [Robotino datasets](#8-robotino-datasets)
9. [Reading the results in Python](#9-reading-the-results-in-python)

Ready files used below: [`examples/robots/`](../../examples/robots) (three robots),
[`examples/maps/my_first_map.yaml`](../../examples/maps/my_first_map.yaml),
[`examples/my_surfaces.json`](../../examples/my_surfaces.json), [`examples/replay_twist.csv`](../../examples/replay_twist.csv).

## 1. First runs and where the results are

```bash
bash scripts/drive.sh --list                                    # what exists: robots, surfaces, trajectories, maps
bash scripts/drive.sh --robot husky --traj circle:R=2           # Husky, circle of radius 2 m, gray rubber floor
bash scripts/drive.sh --robot husky --traj circle:R=2 --surface sand_lete    # the same on sand
bash scripts/drive.sh --robot jackal --map hill@contour         # Jackal on a ready map (map hill, scenario contour)
```

Each command prints one line per run and the folder of the results, `runs_drive/<date>_<engine>/`:

```
runs_drive/20261116_101500_emulate/
  husky_circle_diff_closed_truth/figure.png     <- open this first: path, errors, currents, slip, IMU
  husky_circle_diff_closed_truth/log.csv        <- everything over time (50 Hz)
  husky_circle_diff_closed_truth/metrics.json   <- numbers of the run
  summary.csv, summary.md                        <- one row per run (useful for batches)
```

`--out my_folder` puts the results somewhere else. The columns are explained in [OUTPUTS](OUTPUTS.md).

## 2. Robot + map + trajectory + parameters

A command is put together from independent parts. Change one part at a time:

```
bash scripts/drive.sh  --robot husky  --map slope_15@up  --speed 0.8  --set robot.mass=70  --engine harmonic --gui
                       └─ who ──────┘ └─ where ───────┘  └─ how ────────────────────────┘ └─ engine ──────────────┘
```

| part | options | where to look |
|---|---|---|
| who | `--robot` (built-in name, robot file, comma list) | [ROBOTS](ROBOTS.md) |
| where | `--map NAME@scenario` or `--relief` + `--floor` + `--surface`, `--spawn` | [MAPS](MAPS.md) |
| what to drive | `--traj` (or the scenario of the map), `--speed`, `--accel`, `--heading` | [RUNNING §3](RUNNING.md#3-trajectories) |
| how | `--control`, `--localization`, `--motor-control`, `--imu-noise`, `--set ...` | [RUNNING](RUNNING.md) |
| physics | `--body-contact`, `--on-tip`, `--surface-file`, `--set <surface>.<param>` | [PHYSICS](PHYSICS.md), [SURFACES](SURFACES.md) |
| engine | `--engine emulate` (default) or `harmonic`, `--gui`, `--rtf`, `--jobs` | [RUNNING §9](RUNNING.md#9-engines-speed-and-reproducibility) |

### 2.1 Maps and scenarios
```bash
bash scripts/drive.sh --robot husky --map slope_15@up                        # up a 15° slope
bash scripts/drive.sh --robot husky,jackal --map "slope_15@*" --jobs 4       # 2 robots x 4 scenarios = 8 runs, 4 at a time
bash scripts/drive.sh --robot rover --map mars_yard                          # the rover on the Mars yard (its first scenario)
bash scripts/drive.sh --robot husky --map slope_10@up --surface-file configs/surfaces_husky.json --surface gravel_fit
#                                     the same map, but its default surface replaced by fitted gravel from the tyre library
```
`python3 tools/maps.py list` shows every map with its scenarios; pictures are in [`docs/maps/`](../maps).

### 2.2 Generated terrain instead of a map
```bash
bash scripts/drive.sh --robot burger --relief "bumps:n=40,h=0.01,w=0.15,flat0=0.5" \
     --floor "checkerboard:surfaces=gray+carpet,cell=0.5" --traj figure8:A=1 --map-size 3
#    40 random bumps of 1 cm, a gray / carpet checkerboard, a figure eight, map ±3 m
```

### 2.3 Trajectory, start pose, speed
```bash
bash scripts/drive.sh --robot husky --spawn=-2,1,45 --traj "spline:pts=1,0;2,1;3,0" --speed 0.8 --accel 1.0
#    start at x=-2, y=1 facing 45°; a spline through 3 points of the start frame; 0.8 m/s, 1 m/s²
bash scripts/drive.sh --robot youbot --traj "line:L=2,angle=90" --heading fixed
#    a holonomic robot drives 2 m sideways (to its left) without turning
```
Write negative spawn values with `=` (`--spawn=-2,1,45`), otherwise the shell takes them for an option.

### 2.4 Control, localisation, sensors (batches)
```bash
bash scripts/drive.sh --robot youbot --traj figure8:A=1.2 --heading fixed,tangent --control open,closed --jobs 4
#    2 headings x 2 control modes = 4 runs: compare open loop and the path controller
bash scripts/drive.sh --robot robotino --traj square:L=2,rc=0 --localization truth,odom,odom_imu --imu-noise high --jobs 3
#    the same square with three feedbacks: true pose, wheel odometry, odometry + gyro (noisy IMU)
bash scripts/drive.sh --robot husky --traj line:L=4 --motor-control voltage --surface clay_wet
#    motors without the speed loop (U = Ce·ω_set): the robot slows down in clay
```
Values separated by commas (or `|` for `--traj`, `--floor`, `--relief`) make a batch: one run per combination.
`summary.csv` compares them.

### 2.5 Robot parameters on the fly
```bash
bash scripts/drive.sh --robot husky --set robot.mass=80 --set robot.com_h=0.35 --map slope_20@across
#    a 30 kg payload mounted high, across a 20° slope
bash scripts/drive.sh --robot jackal --set motor.gear=30 --set wheel.patch=0.05 --traj square:L=3,rc=0.5
bash scripts/drive.sh --robot burger --set carpet.mu=0.6 --surface carpet --traj circle:R=0.5
#    --set also changes surface parameters: here carpet friction for this run only
```

### 2.6 Replaying recorded commands
```bash
bash scripts/drive.sh --robot youbot --replay examples/replay_twist.csv --replay-mode twist
#    the file has columns t, vx, vy, wz (body twist over time); --replay-mode wheels takes t, g0, g1, ... (wheel speeds)
```

## 3. Your own robot, step by step

You need: the wheel positions and radii, the mass and roughly where the centre of mass is, the motor data (or
reasonable guesses), and the outer size of the body.

**Step 1. Take an example.** Three ready robot files are in `examples/robots/`:

| file | what it is |
|---|---|
| `my_diffbot.json` | two driven wheels + a ball caster (like a TurtleBot / Pioneer) |
| `my_skidsteer.json` | four wheels, skid steering (like a Husky / Jackal) |
| `husky_payload.json` | a built-in robot with a few values changed (`"base": "husky"`) |

Copy the one closest to your robot into `configs/robots/` under your robot's name:
```bash
cp examples/robots/my_diffbot.json configs/robots/my_bot.json
```

**Step 2. Edit the numbers.** The file (abridged):
```json
{
 "mass": 6.0,                          "com": [-0.03, 0.0, 0.08],
 "I": [0.05, 0.07, 0.09],              "Jw": 0.004,
 "size": [0.34, 0.30, 0.12],           "body_off": [-0.04, 0.0, 0.09],
 "drive": ["diff"],
 "wheels": [
  {"label": "L", "x": 0.0,  "y": 0.16,  "r": 0.06, "b": 0.03, "type": "standard", "group": 0},
  {"label": "R", "x": 0.0,  "y": -0.16, "r": 0.06, "b": 0.03, "type": "standard", "group": 1},
  {"label": "C", "x": -0.20, "y": 0.0,  "r": 0.02, "type": "caster", "group": -1, "swivel": 0}
 ],
 "motor": {"R": 1.5, "Ce": 0.02, "kt": 0.02, "gear": 50.0, "eta": 0.75, "I0": 0.15, "Vmax": 12.0, "kp": 0.1, "ki": 1.0}
}
```
* The robot frame: origin on the ground under the robot centre, x forward, y left, z up, metres.
* `com` = [x, y, height] of the centre of mass; `size` + `body_off` = the box of the body (it touches the ground in
  falls; keep its bottom at the real ground clearance).
* Each wheel: position `x`, `y`, radius `r`, tread width `b`, `type` (`standard`, `omni`, `mecanum`, `caster`), and
  `group` = the motor number (wheels with one number share one motor; casters: −1).
* `motor`: from the datasheet; the formulas are in [ROBOTS §3.3](ROBOTS.md#33-things-to-know). Unknown values: keep the
  example's and check the currents in the figure.
* Every key: [ROBOTS §3](ROBOTS.md#3-your-own-robot-a-full-robot-file).

**Step 3. Look at it.**
```bash
python3 tools/robot_preview.py --robots my_bot --body --out /tmp/prev      # /tmp/prev/my_bot.png
```
Red circles = wheel radii used by the physics, blue cubes = points where the body touches the ground. Check that the
wheels stand on the ground and the body is where it should be.

**Step 4. First run on a flat floor, then harder.**
```bash
bash scripts/drive.sh --list | grep my_bot                    # it is listed with the built-in robots
bash scripts/drive.sh --robot my_bot --traj circle:R=1        # flat floor
bash scripts/drive.sh --robot my_bot --map slope_10@up        # a slope
bash scripts/drive.sh --robot my_bot --map "examples/maps/my_first_map.yaml@*"   # your terrain
bash scripts/drive.sh --robot my_bot --engine harmonic --gui --traj circle:R=1   # in Gazebo
```
A robot file can also be used directly by its path, without copying: `--robot examples/robots/my_diffbot.json`.

**Step 5 (alternative). Change a built-in robot instead of writing a new one.**
```bash
cp examples/robots/husky_payload.json configs/robots/husky_heavy.json
bash scripts/drive.sh --robot husky_heavy --map slope_20@up
```
`husky_payload.json` is only `"base": "husky"` plus what changes (mass, centre of mass, gear, patch, speed).

Typical mistakes:
* a wheel `z` other than its radius for a robot whose base origin is on the ground (leave `z` out: it defaults to `r`);
* both driven wheels in one `group` on a differential robot (it can then only drive straight);
* the `size` box reaching below the ground clearance: the body then scrapes the ground on every bump (see the blue cubes).

## 4. Your own surface, step by step

**Step 1.** Open [`examples/my_surfaces.json`](../../examples/my_surfaces.json). A surface is `"base"` (the closest
existing surface) plus the parameters that differ:
```json
"linoleum_soft": {"base": "table", "mu": 0.8, "f_lat": 0.012}
```
**Step 2.** Add yours (meaning of every parameter and how to choose values: [SURFACES](SURFACES.md#1-parameters)).

**Step 3.** Use it:
```bash
bash scripts/drive.sh --robot burger --surface-file examples/my_surfaces.json --surface rubber_tiles --traj circle:R=0.8
bash scripts/drive.sh --robot husky --surface-file examples/my_surfaces.json --floor "stripes:surfaces=gray+gravel_like,w=1" --traj line:L=5
#    stripes of gray rubber and your gravel-like surface, 1 m wide
```
In a map, use the name in a zone and pass the same `--surface-file`, or declare the surface inside the map (§5).

## 5. Your own map, step by step

**Step 1.** Run the example map as it is:
```bash
bash scripts/drive.sh --map examples/maps/my_first_map.yaml                      # scenario road, robots jackal + husky
bash scripts/drive.sh --map "examples/maps/my_first_map.yaml@*" --robot jackal --jobs 3   # all three scenarios
python3 tools/maps.py preview examples/maps/my_first_map.yaml --out /tmp          # its picture
```
**Step 2.** Copy it and edit ([the file](../../examples/maps/my_first_map.yaml) is commented line by line):
```bash
cp examples/maps/my_first_map.yaml configs/maps/my_map.yaml      # now --map my_map works
# or start from the full template with every zone and relief type:
python3 tools/maps.py new my_map
```
The parts of a map: `zones` (which surface where), `relief` (heights: hills, slopes, bumps, craters, ditches...),
`surfaces` (surfaces of this map only), `robots`, `scenarios` (start pose + trajectory). Every type:
[MAPS §2](MAPS.md#2-file-format).

**Step 3.** Check and run:
```bash
python3 tools/maps.py check my_map --robot jackal,husky       # is the relief smooth enough for these wheels?
bash scripts/drive.sh --map my_map
bash scripts/drive.sh --map my_map@hill --map-set relief.0.height=0.8 --robot husky    # try a value without editing
```

## 6. Everything of your own in one run

```bash
bash scripts/drive.sh --robot examples/robots/my_diffbot.json \
     --map examples/maps/my_first_map.yaml@bumps \
     --surface-file examples/my_surfaces.json --surface rubber_tiles
#    your robot, your map (scenario bumps), your surface as the default floor of the map
```

## 7. Falls and rollovers

The body touches the ground (`--body-contact on`, default): a robot scrapes its belly on a bump, falls off an edge and
rolls over. By default a run stops 0.2 s after the chassis tilts more than 50° to the ground (`--on-tip stop`);
`--on-tip continue` lets it roll on to the end of the run.

```bash
bash scripts/drive.sh --robot jackal --map rollover --on-tip continue     # 55° side slope: rolls down, ends on its roof
bash scripts/drive.sh --robot jackal --map rollover@up --on-tip continue  # straight up: flips over backwards
bash scripts/drive.sh --robot waffle --map washboard@across               # 1 cm ground clearance on 2 cm waves: scrapes
bash scripts/drive.sh --robot waffle --map washboard@across --body-contact off   # the same with the body passing through the ground
bash scripts/drive.sh --robot husky --map rollover --on-tip continue --engine harmonic --gui
```
The log has the tilt to the ground and the body contact force (`tilt_rel`, `body_N`, `body_pts`), see
[OUTPUTS](OUTPUTS.md#2-logcsv). How it works: [PHYSICS §2.7](PHYSICS.md#27-body-ground-contact-falls-and-rollovers).

## 8. Robotino datasets

```bash
bash scripts/robotino.sh --engine emulate --surfaces carpet --speeds 0.2 --quick         # 12 directions, pipeline check
bash scripts/robotino.sh --engine emulate --surfaces real                                # the 4 calibrated floors, full star
bash scripts/robotino.sh --engine emulate --surface-file examples/my_surfaces.json --surfaces linoleum_soft --speeds 0.1 --quick
```
Format and columns: [DATASETS](DATASETS.md).

## 9. Reading the results in Python

```python
import pandas as pd, json
run = 'runs_drive/<folder>/husky_circle_diff_closed_truth'
log = pd.read_csv(f'{run}/log.csv')                 # t, x, y, yaw, ..., w1_cur, w1_slip, ..., tilt_rel, body_N
print(log[['t', 'x', 'y', 'w1_cur', 'w1_slip']].describe())
m = json.load(open(f'{run}/metrics.json'))           # xtrack_rms_m, energy_J_per_m, slip_mean, odom_drift_m, ...
S = pd.read_csv('runs_drive/<folder>/summary.csv')   # one row per run of a batch
print(S[['run', 'status', 'progress', 'energy_J_per_m', 'slip_mean']])
```
