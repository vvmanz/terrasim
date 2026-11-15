# How terrasim works

[Русская версия](../ru/PHYSICS.md)

terrasim adds wheel–terrain interaction (terramechanics), DC motors and on-board sensors to Gazebo Harmonic.
This page explains what is computed, where, and which parameter controls what. Parameter lists: [SURFACES](SURFACES.md),
[ROBOTS](ROBOTS.md).

## 1. Architecture

```
configs/calibrated_params.json, configs/surfaces_husky.json, configs/maps/*.yaml, configs/robots/*.json
        │   (surfaces, robots, maps: all numbers live in these files)
        ▼
tools/*.py          build the world (SDF), the plugin INI, the terrain map (DTM) and the meshes
        ▼
┌──────────────────────── engine (one of) ────────────────────────┐
│ Gazebo Harmonic (gz-sim 8, DART)   gz_harmonic/src/terramech_system.cpp
│ emulate: core/tests/vehicle3d.cpp, robotino3d.cpp  (same code, own rigid-body integrator)
│ surrogate: core/src/robotino2d.cpp (planar Robotino, for fast parameter sweeps)
└──────────────────────────────┬──────────────────────────────────┘
                               │ every step: wheel positions / velocities → forces, torques
                               ▼
core/include/terramech/   header-only C++17, no dependencies
   sim_logic.hpp       reads the INI, picks the mode, writes the logs
   omni.hpp            wheel–surface contact, surface parameters, heterogeneity fields, DC motor
   vehicle_logic.hpp   any wheeled robot: trajectory, controller, odometry, IMU, terrain map
   omni_logic.hpp      Robotino "star" experiment (datasets)
   sensors.hpp         IMU, encoders, odometry, full per-run log
   terramech.hpp       Bekker–Wong–Janosi wheel (MarsSim model), DTM map, INI and CSV utilities
```

The key design rule: **the engine only integrates rigid bodies.** Everything about the wheel–ground contact, the
motors, the controller and the logging is in the header-only core and is identical for every engine. The Gazebo
system plugin is a thin wrapper: it reads the wheel states from DART, calls the core and applies the returned
wrenches. The offline engines do exactly the same with their own integrator, so a run can be checked in seconds
without Gazebo and then repeated in Gazebo.

The terrain in Gazebo is **visual only**: nothing collides with the terrain mesh in DART. The plugin computes the
ground contact of the wheels **and of the body** from the terrain map (heights + surface type per grid node), so both
engines see exactly the same ground. Only a safety floor 0.6 m below the terrain has engine collisions.

## 2. Wheel–surface contact (`omni.hpp`, `OmniContact`)

At every step and for every wheel the core computes four force components.

| # | force | where it comes from | parameters |
|---|---|---|---|
| 1 | **normal force** N | the floor does not let the wheel sink | `kn`, `cn` (numerical spring-damper, not a floor property) |
| 2 | **traction** H | the rim slips relative to the ground; the ground (or the tread) is sheared and resists | `mu`, `K`, `c`, patch length |
| 3 | **rolling losses** | tread and rollers deform while rolling, roller bearings rub | `f_roll`, `f_lat`, `mu_rb` |
| 4 | **compaction resistance** | a soft layer is pressed down; this costs energy | `kc`, `kphi`, `n`, `layer`, `rebound` (Bekker) |

### 2.1 Normal force
Always a penalty spring-damper: N = kn·δ − cn·v_n, with δ the overlap of the wheel with the ground (along the
terrain normal). For robots other than Robotino kn and cn are scaled with the static wheel load so that the static
deflection stays the same (`--no-contact-scale` switches this off, `--kn-scale` makes all wheels softer).

### 2.2 Traction (Janosi–Hanamoto shear)
Traction is built up over the contact patch of length L:
τ(x) = (c + σ·tan φ)(1 − e^(−j/K)), with the shear displacement j = s·x, integrated over the patch. The maximum is
H_max = c·A + N·tan φ, with tan φ = `mu` and A the patch area.

* Small slip: H grows linearly with s, the slope is about μ·N·L/(2K). **K is the main slip parameter** in ordinary
  driving: a softer shear (larger K) needs more slip for the same force.
* Large slip: H saturates at H_max and the wheel spins.
* `slip_norm` chooses what is sheared: 1 = the ground element (carpet pile, foam, soil; slip divided by the ground
  speed), 0 = the wheel element (classic rigid-wheel derivation). The Robotino calibration selected 1.
* Slip is filtered with a relaxation length of about the patch length (1 cm).

Which direction carries traction depends on the wheel type:

| type | traction along | free direction |
|---|---|---|
| `omni` (γ = 90°) | rolling direction of the wheel | along the axle (rollers) |
| `mecanum` (γ = ±45°) | the roller axis (diagonal) | perpendicular to the roller axis |
| `standard` | rolling direction; the lateral force follows the same Janosi law and shares one friction limit with traction (`lat_grip` scales the lateral part) | none |
| `caster` | none (passive) | every direction; resistance f_roll·N + compaction |

### 2.3 Rolling losses
* `f_roll` — rolling resistance of the wheel itself: a moment f_roll·N·r on the wheel, paid by **its own motor**.
* `f_lat` — rolling resistance of the rollers (omni, mecanum). It is a force on the hub, so it is paid by the
  **other** wheels through their traction. That is why the driven wheels of an omni robot slip.
* `mu_rb` — roller bearing friction. A property of the wheel (robot), not of the floor; a wheel can override it.
* For mecanum wheels the rollers stand still when the robot drives straight and spin when it drives sideways,
  so sideways driving costs much more energy (as on a real youBot).

### 2.4 Bekker layer (deformable surfaces only)
The layer is active when `kc > 0` or `kphi > 0`. It only adds what a firm floor does not have:
* sinkage z of a rigid wheel under its load from p = (kc/b + kphi)·zⁿ (lagged by 50 ms), limited by `layer` and by
  `zmax_r`·r;
* compaction resistance R = b·(kc/b + kphi)·z^(n+1)/(n+1) against the motion, applied at the hub; only the share
  (1 − `rebound`) is lost (pile and foam spring back);
* a longer patch L = max(patch, √(2rz)) and a larger cohesive area b·L, so traction grows on soft ground;
* the wheel sits in its rut; climbing out onto firmer ground works like a ramp of the rut wall, never as a jump.

The normal force stays the spring-damper of §2.1. On a firm floor (kc = kphi = 0) the Bekker layer is simply off:
no sinkage, no compaction, and the parameters `kc`, `kphi`, `n`, `layer`, `rebound` are not used.

### 2.5 Heterogeneity
Real floors are not uniform. Every surface can carry Gaussian random fields along the ground
(correlation length `corr_len`, realisation `field_seed`):
* `cv_res` — coefficient of variation of the losses f_roll, f_lat, mu_rb;
* `cv_grip` — of the shear modulus K;
* `cv_soil` — of kc, kphi (soils).

They produce the spread of currents and slips inside one run. The field is fixed in space, so driving over the same
spot gives the same values.

### 2.6 Terrain map
A map (DTM) stores the height and the surface type at every grid node. Between nodes the heights are
interpolated and the surface parameters are blended (friction through tan φ, softness through the compliance
1/k, so the sinkage fades out smoothly towards a firm surface). The ground normal comes from the height gradient.

The wheel touches the ground **at one point below its centre**, on the tangent plane of the terrain. This is exact
for slopes and smooth hills and an approximation for bumps narrower than the wheel; `tools/maps.py check` measures
the error for every robot ([MAPS](MAPS.md#4-is-the-relief-valid-for-the-contact-model)).

### 2.7 Body-ground contact, falls and rollovers
With `--body-contact on` (default) the chassis touches the ground at points of its collision shapes: every corner, edge
midpoint and face centre of a body box (26 points), two rings of a cylinder body, and four points on the outer face of
every wheel (a wheel lying on its side rests on its face). The shapes of every robot are listed in
[ROBOTS](ROBOTS.md#35-body-shape-for-the-ground-contact); `python3 tools/robot_preview.py --body` draws the points.

At every point below the ground (the same terrain map as the wheels):
* normal force F_n = k·d − c·v_n ≥ 0 (d = depth below the ground plane, v_n = normal speed of the point);
* friction with real sticking: a tangential spring-damper to an anchor point while |F_t| ≤ μ·F_n; above that the point
  slides with |F_t| = μ·F_n and the anchor follows. A robot lying on a slope does not creep.

k is chosen so that the robot resting on 4 points sinks by 1 mm (k = m·g / (4 · 1 mm)), c = 0.4 of critical damping
for a quarter of the mass, μ = 0.6 (`--body-mu`); `--set body.k=…`, `body.c`, `body.mu` change them. The resulting force
and torque are applied to the chassis link: in Gazebo through `Link::AddWorldWrench`, in the emulator by its
integrator. The body is rigid against soft soils (no body sinkage).

When the body contact is on, a strongly tilted wheel touches the ground with its rim (contact radius r·√(1 − (a·n)²),
a = axle, n = ground normal; no rim contact beyond 76° tilt). For an upright wheel nothing changes, so runs on
ordinary terrain give the same numbers as before. `--body-contact off` restores the old behaviour exactly (the body
passes through the ground).

Tipping over: the angle between the chassis z axis and the ground normal is logged (`tilt_rel`). Above `--tip-deg`
(50°) the run is marked `tipped`; with `--on-tip stop` (default) it ends 0.2 s later, with `--on-tip continue` the
robot rolls, lands and the run goes on to its end.

## 3. Motors and control

Every motor is a DC motor with a gearbox (`omni.hpp`, `DCMotor`), ω_m = gear·ω_wheel:
* controller voltage V = ff·Ce·ω_set + kp·(ω_set − ω_m) + ki·∫(ω_set − ω_m), limited to ±`Vmax`;
* current I = (V − Ce·ω_m − driver drop `Vd`) / R;
* motor torque τ_m = kt·I − kt·I0·tanh(ω_m / w_eps) (`I0` = no-load current: friction of motor and gearbox);
* wheel torque = gear·η·τ_m when the motor drives, gear·τ_m/η when it is driven back.

`--motor-control voltage` replaces the speed loop by V = Ce·ω_set (no feedback: the motor slows down under load).
Wheels with the same `group` share one motor (skid-steer sides of Husky and Jackal are chain-coupled).

The logs compute voltage, electrical power and energy the same way as the Robotino lab recordings:
U = sign(ω)·I·R + Ce·ω, P = |U·I|.

Above the motors, the vehicle mode (`vehicle_logic.hpp`) adds:
* a trajectory with a trapezoidal speed profile;
* open loop (inverse kinematics only, no feedback) or closed loop (path controller, 50 Hz) on the true pose, wheel
  odometry or wheel odometry + gyro heading;
* an IMU at the centre of mass (noise, bias, bias random walk) and quantised wheel encoders.

Details and options: [RUNNING](RUNNING.md).

## 4. Engines

| engine | what it is | use it for |
|---|---|---|
| `harmonic` | Gazebo Harmonic (gz-sim 8, DART) with the terrasim system plugin | full 3D dynamics, real suspension (rover rocker-bogie), GUI, ROS 2 |
| `emulate` | the same core with its own rigid-body integrator (`vehicle3d`, `robotino3d`), rigid chassis | fast checks and large batches (10–100× real time and more) |
| `surrogate` | planar Robotino (`robotino2d`) | parameter sweeps of the Robotino star experiment only |

On flat ground the emulator and Gazebo give the same numbers within a few percent. On rough terrain they differ
where the chassis matters: the emulator has a rigid chassis (no rocker-bogie for the rover, a rigid four-wheel base
rests on a diagonal pair of wheels on twisted terrain).

## 5. Units and conventions
* SI everywhere: m, kg, N, Pa, rad, s. kc in N/m^(n+1), kphi in N/m^(n+2), c in Pa, K in m.
* Body frame: x forward, y left, z up. Wheel `heading_deg` = rolling direction for positive spin in the body frame.
* Map frame: x, y in metres, z up; trajectories are given in the start frame of the robot ([RUNNING](RUNNING.md#3-trajectories)).

## 6. Limitations
* **The body is a set of points on simple shapes** (boxes, cylinders, wheel faces). Thin parts that are not in the
  shapes (antennas, a lidar mast) do not touch the ground; add shapes for them in the robot file if they matter.
* **No rigid obstacles or steps** (single-point wheel contact). Use a steep ramp or a berm instead of a step or a wall.
* **The body is rigid against soft ground**: it does not sink into sand like a wheel does.
* **Surfaces were calibrated on Robotino** (8 cm polyurethane omni wheels). For tyres the same law is used with the
  wheel's own radius, width and patch; K and f_roll of the hard surfaces were not checked on tyre data except
  where stated (the `_fit` surfaces of `configs/surfaces_husky.json`).
* **The emulator has a rigid chassis.** Look at the rover on rough terrain in Gazebo.
* **Odometry + IMU fusion is simple** (path from the wheels, heading from the gyro), no Kalman filter. All raw
  signals are in the logs if you want to build your own estimator.
