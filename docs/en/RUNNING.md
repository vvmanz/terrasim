# Running simulations

[Русская версия](../ru/RUNNING.md)

Three entry points:
* `scripts/drive.sh` — **sandbox**: any robot drives a trajectory over a terrain map, open or closed loop, with
  odometry and IMU; one log, figure and metrics per run (§1–§6);
* `scripts/robotino.sh` — **Robotino star experiments**: datasets in the format of the Robotino lab recordings
  ([DATASETS](DATASETS.md));
* `python3 tools/traverse.py` — **traversability matrix**: every robot × surface × slope (§8).

All scripts take `--engine harmonic` (Gazebo Harmonic) or `--engine emulate` (offline core, no Gazebo); see
[PHYSICS](PHYSICS.md#4-engines). ROS 2: [ROS2](ROS2.md).

## 1. Quick start

More examples, step by step (your own robot, surface and map): [EXAMPLES](EXAMPLES.md).

```bash
cd ~/terrasim
bash scripts/drive.sh --list                                   # robots, surfaces, trajectories, relief / floor specs, maps
# Robotino on a gray / carpet checkerboard, circle R = 1 m, emulator (seconds)
bash scripts/drive.sh --engine emulate --robot robotino --floor checkerboard:surfaces=gray+carpet,cell=0.5 --traj circle:R=1
# Husky on craters, rounded square, Gazebo window in real time
bash scripts/drive.sh --engine harmonic --robot husky --relief craters:n=5,R=1.5,depth=0.15,flat0=1.5 --traj square:L=4,rc=0.6 --gui
# youBot: figure eight sideways (fixed heading) and facing forward, without and with the controller (4 runs)
bash scripts/drive.sh --engine emulate --robot youbot --traj figure8:A=1.2 --heading fixed,tangent --control open,closed
# a ready map with its scenario and robots
bash scripts/drive.sh --map slope_ladder
# the example set: 9 small studies, a few minutes in the emulator
bash scripts/drive_examples.sh              # or: bash scripts/drive_examples.sh harmonic
```

The default engine of `drive.sh` is `emulate`, of `robotino.sh` and the ROS 2 launch files `harmonic`. `--gui` opens
the Gazebo window (real time by default; `--rtf 5` runs five times faster); without `--gui` Gazebo runs headless as
fast as possible.

## 2. Options of `scripts/drive.sh`

Values separated by commas (`--robot`, `--control`, `--localization`, `--heading`, `--drive`, `--motor-control`) or by
`|` (`--traj`, `--floor`, `--relief`) start one run per combination: a batch. Quote values that contain `;` or `|`:
`--traj "spline:pts=1,0.5;2,0" --relief "craters:n=4;bumps:n=20"`.

| option | default | meaning |
|---|---|---|
| `--robot` | robotino (with `--map`: the map's robots) | built-in robot, a robot file name in `configs/robots/`, or a path to a robot file ([ROBOTS](ROBOTS.md)) |
| `--engine` | emulate | `harmonic` or `emulate` |
| `--map` | – | map file: `NAME`, `NAME@scenario`, `"NAME@*"`, a path; comma list = batch ([MAPS](MAPS.md)) |
| `--map-set` | – | change a value of the map file: `relief.0.slope_deg=12` |
| `--relief`, `--floor` | flat, uniform | generated terrain instead of a map file ([MAPS](MAPS.md#6-generators-on-the-command-line---relief---floor)) |
| `--surface` | gray | surface of a uniform floor; with `--map` it replaces the map's default surface |
| `--surface-file` | – | extra surfaces (JSON, [SURFACES](SURFACES.md#4-adding-your-own-surface)) |
| `--map-size`, `--res` | per robot | half-width of the generated square map and its grid step, m |
| `--spawn` | 0,0,0 | x, y, heading (°) of the start in the map frame (negative values: `--spawn=-2,0,0`) |
| `--traj` | per robot | trajectory (§3) |
| `--speed`, `--accel` | per robot | cruise speed, m/s, and acceleration of the trapezoidal profile, m/s² |
| `--drive` | first of the robot | `holonomic` (omni, mecanum) or `diff` |
| `--heading` | tangent | holonomic only: `tangent` (face along the path) or `fixed` (keep the start heading, drive sideways) |
| `--control` | closed | `open` (no controller) or `closed` (path controller) (§4) |
| `--localization` | truth | feedback of the closed loop: `truth`, `odom`, `odom_imu` |
| `--motor-control` | speed | `speed` (PI speed loop) or `voltage` (U = Ce·ω_set, no motor loop) |
| `--imu-noise`, `--imu-rate`, `--seed` | default, 100 Hz, 11 | IMU noise preset (§5), IMU rate, random seed |
| `--ctrl-rate` | 50 Hz | rate of the path controller and the odometry |
| `--t-start`, `--t-settle` | 1.5 s, 2 s | standstill before the motion (gyro bias calibration) and after it |
| `--set` | – | `name.param=value`: robot, motor, wheel, caster, controller, imu, contact, trajectory, body (`k`, `c`, `mu`) or a surface ([ROBOTS](ROBOTS.md#21-for-one-run---set)) |
| `--kn-scale`, `--no-contact-scale` | 1, off | softer wheels (< 1); do not scale contact stiffness with the wheel load |
| `--body-contact` | on | the body touches the ground (falls, rollovers, a robot lying on a bump); `off` = it passes through ([PHYSICS §2.7](PHYSICS.md#27-body-ground-contact-falls-and-rollovers)) |
| `--on-tip`, `--tip-deg` | stop, 50 | when the chassis tilts more than `--tip-deg` to the ground: `stop` the run 0.2 s later or `continue` (the robot rolls over and the run goes on) |
| `--body-mu` | 0.6 | friction of the body on the ground |
| `--replay`, `--replay-mode`, `--truth`, `--v0` | – | replay recorded commands instead of a trajectory (§6) |
| `--gui`, `--rtf` | off, 1 with GUI | Gazebo window; real-time factor (0 = as fast as possible) |
| `--gz-sensors` | off | Harmonic: native gz IMU (`/imu`), OdometryPublisher, JointStatePublisher |
| `--ros`, `--gz-partition` | off | publish the running job to ROS 2 ([ROS2](ROS2.md)) |
| `--jobs` | 1 | runs in parallel |
| `--dt`, `--log-rate`, `--full-rate`, `--no-full` | 0.001 s, 50 Hz, 50 Hz | physics step, rate of `log.csv`, rate of `full.csv.gz` / switch it off |
| `--stall`, `--progress` | 120 s, 15 s | stop a run whose log stops growing; progress print interval |
| `--out`, `--no-figures` | `runs_drive/<date>_<engine>` | output folder; skip the figures |

The full list with the current defaults: `python3 tools/drive_run.py --help`. Outputs: [OUTPUTS](OUTPUTS.md).

## 3. Trajectories

A trajectory is given in the **start frame**: origin at the robot, +x where it looks. The same trajectory works at
any spawn pose.

| spec | parameters (defaults) | what it is |
|---|---|---|
| `line` | `L`=3, `angle`=0 | straight line; `angle` = direction in the start frame (with `--heading fixed` a holonomic robot drives sideways) |
| `circle` | `R`=1, `loops`=1 | circle turning left (R < 0: right), entered tangentially |
| `square` | `L`=2, `rc`=0.3, `loops` | counter-clockwise square with corners rounded by rc (rc = 0: sharp, holonomic robots only) |
| `rect` | `Lx`, `Ly`, `rc` | rectangle |
| `figure8` | `A`=1.5, `loops` | figure eight (Gerono lemniscate) of half-width A |
| `spline` | `pts=x1,y1;x2,y2;...` | Catmull–Rom spline through the points (start frame, m) |
| `slalom` | `L`=4, `A`=0.4, `n`=2 | slalom along +x |
| `spiral` | `R0`=0.3, `R1`=1.5, `turns`=2 | spiral outwards |

The speed follows a trapezoid (acceleration `--accel`, cruise `--speed`, braking); the reference point moves with time.
`python3 tools/trajectories.py "figure8:A=1.2" --plot` draws a trajectory.

## 4. Open loop and closed loop

**`--control open` — no controller.** The reference speed and curvature give the wheel speeds by inverse kinematics;
they go to the motor speed loops. The robot drives "blind": slip and skidding accumulate into a path error.

**`--control closed` — path controller** (50 Hz): feed-forward of the reference plus feedback.
* holonomic: P controller on position (`kp_xy`) and heading (`k_yaw`);
* diff: Kanayama law. With the along-track, cross-track and heading errors e_x, e_y, e_θ:
  v = v_r·cos e_θ + k_x·e_x, ω = ω_r + v_r·(k_y·e_y + k_θ·sin e_θ).

Feedback (`--localization`):
* `truth` — the true pose from the engine (perfect localisation);
* `odom` — wheel odometry from quantised encoders (least-squares over all wheels; skid steer with `odom_yaw_scale`
  like `wheel_separation_multiplier` of ROS diff_drive_controller);
* `odom_imu` — path from the wheels, heading from the gyro (bias averaged at standstill before the start).

After the end of the path the closed loop holds the end point; the open loop stops.

## 5. IMU and odometry

The IMU sits at the centre of mass of every robot: gyroscope (rad/s) and accelerometer (specific force, m/s²) in the
body frame, 100 Hz. Every sample is the average over the sample period plus white noise, a constant bias (random per
`--seed`) and a bias random walk.

| preset | gyro: noise / bias / walk | accelerometer: noise / bias / walk | class |
|---|---|---|---|
| `none` | 0 | 0 | ideal |
| `low` | 3·10⁻⁴ / 10⁻³ / 2·10⁻⁶ | 0.005 / 0.02 / 2·10⁻⁵ | industrial MEMS (ADIS16470) |
| `default` | 0.002 / 0.005 / 2·10⁻⁵ | 0.03 / 0.1 / 2·10⁻⁴ | consumer MEMS (MPU-6050, BMI088) |
| `high` | 0.01 / 0.02 / 10⁻⁴ | 0.2 / 0.3 / 10⁻³ | cheap IMU + vibration |

Noise per sample, random walk per √s. Single values: `--set imu.gyro_sigma=...`.

Three odometries are logged for every run: ideal (exact joint speeds), encoder (quantised counts) and encoder + gyro.
The native Gazebo OdometryPublisher (`--gz-sensors`) reports the true pose of the model, not a wheel odometry.

## 6. Replaying recorded commands

```bash
bash scripts/drive.sh --robot husky --map my_map --replay cmd.csv --replay-mode wheels   # t,g0,g1,...: wheel speeds of the motor groups, rad/s
bash scripts/drive.sh --robot youbot --surface gray --replay cmd.csv --replay-mode twist   # t,vx,vy,wz: body twist
#   --truth truth.csv   recorded path t,x,y,yaw in the start frame, used as the reference in the logs and metrics
#   --v0 vx,vy,wz       the recording starts while driving (emulator)
```
This is how a simulation is compared with a recording of your own robot: the same commands, the same surface, then
compare the logs.

## 7. Robotino star experiments (`scripts/robotino.sh`)

```bash
bash scripts/robotino.sh --list                                            # surface library with parameters
bash scripts/robotino.sh --engine emulate --surfaces real                  # the 4 calibrated floors, all speeds
bash scripts/robotino.sh --engine harmonic --surfaces synthetic --jobs 4   # synthetic floors, 4 worlds in parallel
bash scripts/robotino.sh --engine harmonic --surfaces carpet --speeds 0.2 --gui --rtf 5
bash scripts/robotino.sh --engine emulate --surfaces gray --quick          # 12 directions x 6 s, pipeline check
bash scripts/robotino.sh --engine emulate --surfaces carpet --sweep carpet:mu=0.6,0.8,1.0
bash scripts/robotino.sh --engine emulate --surface-file examples/my_surfaces.json --surfaces user
```

| option | default | meaning |
|---|---|---|
| `--engine` | harmonic | `harmonic`, `emulate`, `surrogate` (planar, for sweeps) |
| `--surfaces` | real | names and/or groups `real`, `synthetic`, `hard`, `extreme`, `user`, `all` |
| `--speeds` | per surface | e.g. `0.1,0.3` m/s |
| `--dirs` | 0:355:5 | directions of the rays: `start:stop:step` in degrees or a list |
| `--t-run`, `--t-pause` | 18/10/6 s, 1.5 s | duration of one ray (for 0.1/0.2/0.3 m/s), pause between rays |
| `--noise` | both | `clean`, `noisy` or `both` datasets |
| `--map`, `--return` | – | star on a terrain map; drive back to the centre after every ray |
| `--set`, `--sweep`, `--surface-file` | – | change parameters, sweep one, add surfaces |
| `--strict-format`, `--no-ext`, `--no-full`, `--no-figures` | | output options ([DATASETS](DATASETS.md)) |
| `--gui`, `--rtf`, `--jobs`, `--dt`, `--seed`, `--out` | | as in `drive.sh`; default output `datasets/<date>_<engine>` |

Merging and completing dataset folders:
```bash
python3 tools/robotino_merge.py datasets/<a> datasets/<b> --out datasets/<merged>   # the last folder wins on duplicates
bash scripts/robotino_missing.sh datasets/<folder> --engine emulate                # run the (surface, speed) pairs that are missing
```

## 8. Traversability study (`tools/traverse.py`)

Every robot × every surface × slope, the same task for all: a straight line with the path controller on the true pose.
```bash
python3 tools/traverse.py                                          # all robots, 12 surfaces, slopes 0/10/20/30°
python3 tools/traverse.py --robots burger,waffle --angles 0,5,10,15,20,25 --jobs 4
python3 tools/traverse.py --engine harmonic --robots burger --surfaces gray,sand_lete
```
Output (`runs/traverse_<date>`): `runs.csv` (progress, slip, current / stall current, sinkage, energy per metre,
verdict), `matrix_flat.png/.csv`, `max_slope.png/.csv`, `REPORT.md`. Verdicts: ok (progress ≥ 0.95), slow (≥ 0.5),
stuck.

## 9. Engines, speed and reproducibility
* `--engine emulate` uses the same core as Gazebo with a rigid chassis; it needs only `scripts/build.sh core`.
* Results are deterministic for a given `--seed` (IMU noise) and `field_seed` (surface heterogeneity).
* With `--jobs N` several worlds run in parallel; each Gazebo job gets its own partition (`GZ_PARTITION`), so their
  topics do not mix (see [ROS2](ROS2.md) to look at the topics of a run).
* A run is stopped if its log does not grow for `--stall` seconds; statuses are listed in [OUTPUTS](OUTPUTS.md#5-run-status).
