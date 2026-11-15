# Synthetic Robotino datasets (`scripts/robotino.sh`)

[Русская версия](../ru/DATASETS.md)

The Robotino model was calibrated on laboratory recordings of a real Festo Robotino driving a "star". `robotino.sh`
repeats the same experiment on any surface and writes the result **in the same format as the lab recordings**
(the 41 columns of `dataset_belyaev.csv`, and the 90 columns of the extended `belyaev_kushnarev.csv`), so the same
analysis code can process real and simulated data. The recordings themselves are not part of this repository.

## 1. The experiment

* The robot drives open loop (the Robotino firmware speed controller, no path controller) along **rays**: a straight
  line in the direction `movedir` = 0°, 5°, …, 355° (72 rays, `--dirs`) at the speed `speedamp`, heading fixed.
* One ray lasts `t_run` = 18 / 10 / 6 s at 0.1 / 0.2 / 0.3 m/s (`--t-run`), with a pause of 1.5 s between rays.
* Logging at the camera rate (30 Hz), time `t` in ms, reset at the start of every ray.
* Optional: a terrain map (`--map`) and driving back to the centre after every ray (`--return`).
* Speeds per surface come from `configs/calibrated_params.json` → `robotino.experiment.speeds` (or `--speeds`).

```bash
bash scripts/robotino.sh --engine emulate --surfaces real                 # gray, green, table, red
bash scripts/robotino.sh --engine harmonic --surfaces all --jobs 4        # every library surface in Gazebo
bash scripts/robotino.sh --engine emulate --surfaces gray --speeds 0.1,0.2,0.3 --noise clean --out datasets/gray_clean
```

## 2. Output folder (`datasets/<date>_<engine>/`)

| file | content |
|---|---|
| `dataset_clean.csv`, `dataset_noisy.csv` | all runs together |
| `csv/<surface>_<speed>_clean.csv`, `_noisy.csv` | one surface and speed, written right after the run |
| `dataset_ext_clean.csv`, `dataset_ext_noisy.csv`, `csv/*_ext_*.csv` | extended 90-column format (§4); `--no-ext` skips it |
| `csv/<surface>_<speed>_full.csv.gz` | full per-run log, 50 Hz ([OUTPUTS](OUTPUTS.md#3-fullcsvgz-full-per-run-log)); `--full-rate`, `--no-full` |
| `summary.csv`, `summary.md` | per surface and speed: speed ratio \|v\|/v_cmd, yaw rate, slip and current of the driven wheels, Ke, sinkage, compaction resistance, RTF |
| `surfaces_constant.csv` | parameters of every surface used, and which of them vary along the path |
| `figures/` | slip vs motor current (clean and noisy) with linear fits, Ke vs direction; 0.5 s window tables |
| `params_used.json` | robot and surface parameters of this run |
| `runs/<surface>_<speed>/` | `experiment.ini`, world, raw logs, `console.log` |

**clean** = the true simulated signals sampled like the lab logger. **noisy** = the same run passed through the
measurement model of the lab bench (`tools/robotino_sensor.py`): a 32 Hz logger reading a 29.4 Hz camera (≈8 % of
frames repeat the previous camera frame, so speed = 0 in them), camera position / yaw noise, encoder speed quantised in
steps of 2.8274 rad/s at the motor, current noise and 1/39 A quantisation. These parameters were identified on the lab
recordings.

## 3. Columns (41 + surface columns)

Wheels i = 1, 2, 3 at θ = 60°, 180°, 300°; wheel radius r = 0.04 m, distance to the centre R = 0.125 m, gear 16.

| column | unit | meaning |
|---|---|---|
| `t` | ms | time since the start of the ray |
| `surf` | | surface name |
| `movedir`, `speedamp` | °, m/s | direction and commanded speed of the ray |
| `xsetspeed`, `ysetspeed` | m/s | commanded body velocity |
| `m{i}setvel`, `m{i}vel` | rad/s | motor speed set point and encoder reading (motor shaft) |
| `m{i}cur` | A | motor current (magnitude) |
| `m{i}vol`, `m{i}pow` | V, W | voltage U = sign(ω)·I·R + Ce·ω and power \|U·I\|, R = 5.3351 Ω, Ce = 0.0501 V·s |
| `xpos`, `ypos`, `ang` | m, m, ° | camera pose of the robot |
| `vx`, `vy`, `omega` | m/s, rad/s | body velocity from the camera (body frame) |
| `rpower`, `Ke` | W, – | total power Σ\|U·I\|; energy coefficient Ke = Σ\|U·I\| / (0.17·Σ\|U\|) |
| `w{i}vel` | rad/s | wheel speed from the encoder (`m{i}vel` / 16) |
| `w{i}effvel` | rad/s | wheel speed the camera motion would need without slip: (1/r)(−sin θᵢ·vx + cos θᵢ·vy + R·ω) |
| `w{i}slip` | – | relative slip 1 − `w{i}effvel` / `w{i}vel` (0 when the motor speed is within one encoder step of 0) |
| `w{i}linslip` | – | \|sin(φ − θᵢ)\|·`w{i}slip`, φ = direction of the wheel-odometry velocity |
| `w{i}_x, w{i}_y` | m | contact point of the wheel (simulation only) |
| `w{i}_mu, _K, _f_roll, _f_lat, _mu_rb, _kc, _kphi` | | surface parameters under the wheel at that sample, including heterogeneity (simulation only) |

`--strict-format` drops the `w{i}_...` surface columns and writes exactly the 41 columns of the lab files.

## 4. Extended format (90 columns)

The extended table adds 49 columns that the lab software computes from the base ones; every formula was identified
on the full lab table and is reproduced exactly (`tools/robotino_ext.py`, filter coefficients in
`configs/robotino_ext_filter.json`):

| columns | meaning |
|---|---|
| `exp_idx`, `run_idx` | ray number in the whole table / within one surface |
| `setvel` | always 0 (kept for the format) |
| `m{i}pos` | encoder counts since the start of the ray, 1970 counts per motor revolution |
| `xcur, ycur, rotcur` | M⁻¹·(signed currents), M rows (−sin θᵢ, cos θᵢ, 1) |
| `w{i}ddvel` | `w{i}vel − w{i}effvel`, slip speed of the wheel, rad/s |
| `ddvx, ddvy, ddomega`, `ddspeedvec` | slip speeds mapped to the body; \|v_odometry\| − \|v_camera\| |
| `*_kalman`, `m{i}esttor` | per-motor linear Kalman filter with the state [motor speed, load torque]; `m{i}esttor` = estimated load torque at the motor shaft, N·m; the `_kalman` columns repeat the base formulas with filtered values |

`python3 tools/robotino_ext.py --check <lab file>` rebuilds a lab table from its 41 base columns and compares all 90
(useful if you have such a recording).

## 5. Figures and further processing

```bash
python3 tools/robotino_figures.py datasets/<folder>/dataset_clean.csv --out /tmp/fig   # slip vs current, 0.5 s windows
python3 tools/robotino_merge.py datasets/<a> datasets/<b> --out datasets/<merged>     # merge folders (last wins)
bash scripts/robotino_missing.sh datasets/<folder> --engine emulate                   # simulate missing (surface, speed) pairs
python3 tools/robotino_sensor.py --dataset my_recording.csv --sim datasets/<folder>/dataset_clean.csv
#   frame-level statistics of your own recording next to simulated data passed through the measurement model
```

Windows of 0.5 s (first 1 s of every ray skipped): w_act = encoder wheel speed, w_eff = speed from the body motion,
slip s = 1 − w_eff/w_act, weighted slip s·\|sin(α − θᵢ)\|; the figures plot them against the motor current per wheel and
fit a line per surface.

## 6. Using the datasets
* **Surface classification / estimation:** current, slip and Ke per window; the `w{i}_mu, _K, ...` columns are the
  ground truth of the surface under every wheel.
* **Heterogeneity:** the spread inside a run comes from the surface fields (`cv_res`, `cv_grip`, `corr_len`); set them to
  0 with `--set <surface>.cv_res=0 --set <surface>.cv_grip=0` for an idealised floor.
* **Reproducibility:** the same command gives the same data (fixed `field_seed` and `--seed`); change `--seed` for a
  different noise realisation.
