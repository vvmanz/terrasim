# Outputs: folders, log columns, metrics

[Русская версия](../ru/OUTPUTS.md)

This page covers `scripts/drive.sh`. The Robotino datasets of `scripts/robotino.sh` are described in
[DATASETS](DATASETS.md); their full per-run logs have the same columns as §3.

## 1. Output folder

`runs_drive/<date>_<engine>/` (or `--out`):

| file | content |
|---|---|
| `<run>/figure.png` | map (surfaces in colour, height contours) with the reference, true, odometry and odometry + IMU paths; tracking errors, currents, slips over time (background = surface under the robot); estimate drift; IMU |
| `<run>/log.csv` | everything over time at `--log-rate` (50 Hz), §2 |
| `<run>/full.csv.gz` | full per-run log at `--full-rate` (50 Hz), §3 |
| `<run>/metrics.json` | metrics of the run (§4) and per surface |
| `<run>/path.csv`, `path_world.csv` | planned path in the start frame / in the map frame |
| `<run>/experiment.ini`, `world.sdf`, `console.log` | what was run: plugin INI, Gazebo world (Harmonic), engine output |
| `summary.csv`, `summary.md`, `summary.png` | one row per run, comparison bars |
| `per_surface.csv` | per run and surface: share of time, current, longitudinal and lateral slip, sinkage |
| `maps/<map>/` | `map.dtm` (heights + surface type per node) and the Gazebo meshes of the terrain |

The run name says what was run: `<robot>_<map>_<trajectory>_<drive>[-<heading>]_<control>[_<localization>]`.

## 2. `log.csv`

Common columns:
* `t` (s), `phase` (0 standstill before, 1 motion, 2 after the end);
* reference: `ref_x, ref_y, ref_yaw, ref_v, ref_w`;
* truth: `x, y, z, roll, pitch, yaw` (pose of the body link), `vx, vy` (body frame), `wz`;
* estimates: `odo_x, odo_y, odo_yaw` (encoder odometry), `fus_x, fus_y, fus_yaw` (odometry + gyro heading);
* IMU: `imu_gx, imu_gy, imu_gz` (rad/s), `imu_ax, imu_ay, imu_az` (m/s²);
* command of the controller in the body frame: `cmd_vx, cmd_vy, cmd_wz`;
* errors relative to the reference point: `e_along, e_cross, e_yaw`;
* `surf` — surface under the robot centre;
* at the end of the row: `tilt_rel` (angle between the chassis z axis and the ground normal, °), `body_N` (total normal
  force of the body-ground contact, N), `body_pts` (number of body points touching the ground).

Per wheel i (in the order of the robot definition, 1…n) — `w{i}_...`:

| column | meaning |
|---|---|
| `wset`, `wenc` | wheel speed set point and encoder reading, rad/s |
| `cur`, `vol`, `pow` | current (A), voltage (V), electrical power (W) of the motor of this wheel; wheels of one motor show the same motor |
| `tau` | motor torque on this wheel, N·m |
| `N`, `Fx`, `Fy` | normal force; traction along the roller axis / rolling direction; force in the free direction (rollers) or lateral force (tyre), N |
| `slip`, `slip_lat` | longitudinal and lateral slip in the patch |
| `sink` | sinkage (rut), m |
| `surf` | surface index under the wheel (order as in `maps/*/map.dtm`) |

## 3. `full.csv.gz` (full per-run log)

Written for every robot and every run by the core (`sensors.hpp`, `FullLog`), so the columns are the same in Gazebo
and in the emulator. In the Robotino star it is `csv/<surface>_<speed>_full.csv.gz`.

Header: `t`, the run description (sandbox: `robot, control, localization, map`; star: `label, speedamp`), `phase`,
`run` and `dir` (ray number and direction in the star; −1 / empty in the sandbox).

| group | columns | meaning |
|---|---|---|
| truth | `x_true, y_true, z_true, roll_true, pitch_true, yaw_true` | body pose from the engine |
| | `qw_true, qx_true, qy_true, qz_true` | the same orientation as a quaternion |
| | `vx_true, vy_true, vz_true` | velocity in the world, m/s |
| | `vx_body, vy_body, vz_body` | the same in the body frame |
| | `wx_true, wy_true, wz_true` | angular velocity in the body frame, rad/s |
| IMU | `imu_ax, imu_ay, imu_az`, `imu_gx, imu_gy, imu_gz` | measured specific force (standing: `az ≈ +9.81`) and angular rate, with noise, bias and walk |
| | `imu_*_true` | the same without noise and bias (average over the IMU period) |
| odometry | `odo_ideal_x/y/yaw` | from the exact joint speeds (no quantisation): differs from the truth only by slip and terrain |
| | `odo_enc_x/y/yaw` | from quantised encoder counts |
| | `odo_imu_x/y/yaw` | encoders + gyro heading (bias estimated at standstill) |
| | | all three are reset to the true pose at the start of the motion (of every ray in the star) |
| control | `ref_x, ref_y, ref_yaw` | reference point (empty in the star) |
| | `cmd_vx, cmd_vy, cmd_wz` | command in the body frame |
| energy | `power_W`, `energy_J` | electrical power of all motors and its integral |
| | `surf` | surface name under the robot centre |

Per wheel `w<label>_...` (labels: `1, 2, 3` for Robotino, `FL, FR, RL, RR`, `L, R, C`, ...):

| column | meaning |
|---|---|
| `angle` | wheel angle since the start, rad |
| `omega`, `omega_set`, `omega_enc` | wheel speed: true, set point, encoder, rad/s |
| `omega_eff` | the wheel speed that the true body motion would need without slip, J_i·(vx, vy, ωz) |
| `cur`, `volt`, `power` | current, voltage, power of the motor of this wheel |
| `tau_motor` | motor torque on the wheel, N·m |
| `tau_contact` | torque of the contact patch about the wheel axle, N·m |
| `N`, `F_trac`, `F_free` | normal force, traction, free-direction or lateral force, N |
| `Fx, Fy, Fz` | contact force in the world, N |
| `slip`, `slip_lat` | longitudinal and lateral slip in the patch (as in the contact model) |
| `slip_kin` | kinematic slip as in the Robotino recordings: 1 − `omega_eff`/`omega` |
| `sink` | sinkage, m |
| `cx, cy` | contact point, m |
| `mu, K, f_roll, f_lat, mu_rb, kc, kphi` | surface parameters at the contact point, including the heterogeneity field |
| `surf` | surface name under the wheel |

Casters have no motor: `cur, volt, power, tau_motor` are 0, `F_free` is the rolling resistance, `omega` the spin of
the ball or wheel (v/r when rolling), `slip` the rim slip speed in m/s (≈ 0 when rolling).

## 4. Metrics (`summary.csv`, `metrics.json`)

| metric | meaning |
|---|---|
| `status` | §5 |
| `wall_s`, `sim_s`, `rtf` | wall time, simulated time, achieved real-time factor |
| `path_len_m`, `driven_m`, `T_move_s` | planned path length, distance actually driven, duration of the motion |
| `xtrack_rms_m`, `xtrack_max_m` | geometric deviation from the path (to its nearest point) |
| `track_err_rms_m`, `track_err_max_m` | error relative to the moving reference point |
| `final_err_m`, `yaw_err_rms_deg` | final pose relative to the end of the path; heading error |
| `odom_drift_m`, `odom_yaw_drift_deg` | drift of the encoder odometry at the end |
| `fused_drift_m`, `fused_yaw_drift_deg` | drift of odometry + IMU |
| `energy_J`, `energy_J_per_m`, `I_mean_A`, `I_max_A` | electrical energy, per metre, mean and peak current |
| `slip_mean`, `slip_lat_mean`, `sink_max_mm`, `sink_caster_max_mm` | mean slips of the driven wheels, peak sinkage |
| `tilt_max_deg`, `tilt_rel_max_deg` | largest body tilt; largest tilt relative to the ground normal |
| `progress`, `progress_max`, `driven_ratio` | position along the path at the end (share of its length; sliding back reduces it), farthest point, driven / planned length |
| `climb_m`, `descent_m`, `terrain_slope_max_deg`, `terrain_slope_end_deg` | relief under the path: height gained and lost, steepest and final slope under the robot |
| `surface_changes` | how often the surface under the robot changed |
| `tipped_t`, `left_map_t` | time of tipping over / leaving the map |

## 5. Run status
* `done` — finished;
* `tipped` — the chassis tilted more than `--tip-deg` (50°) to the ground. With `--on-tip stop` (default) the run ends
  0.2 s later and the metrics cover the run up to that moment; with `--on-tip continue` the run goes on to its end and
  the metrics cover all of it (`tipped_t` = when it happened);
* `left_map` — the robot drove or slid off the map (no relief there, the run is cut);
* `diverged`, `stalled`, `FAILED` — the simulation failed (numerical divergence, no log progress for `--stall` s,
  engine error; see `console.log`).
