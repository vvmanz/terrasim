# Surfaces: parameters, library, your own surfaces

[Русская версия](../ru/SURFACES.md)

A surface (a floor covering or a soil) is one set of numbers for the contact law of [PHYSICS](PHYSICS.md#2-wheelsurface-contact-omnihpp-omnicontact).
The law is the same for every surface and every robot; only the numbers differ.

Where surfaces live:
* `configs/calibrated_params.json` → `robotino.surfaces` — the base library (18 surfaces). Every robot can use it.
* `configs/surfaces_husky.json` — 27 surfaces for robots with ordinary tyres (built for the Clearpath Husky; any robot
  can use them): `--surface-file configs/surfaces_husky.json`.
* your own JSON files (`--surface-file my.json`), map files (`surfaces:` in a map) and `--set` on the command line.

List the library with its parameters: `bash scripts/robotino.sh --list` (base library) or `bash scripts/drive.sh --list`.

## 1. Parameters

### 1.1 Traction (Janosi shear law)
| parameter | unit | meaning | typical values |
|---|---|---|---|
| `mu` | – | friction limit tan φ: rubber on the covering, or the internal friction of a soil | wet smooth 0.3–0.4; carpets, rubber 0.8–1.4; soils 0.2–0.7 |
| `phi_deg` | ° | the same as a friction angle; only used if `mu` is not set (every library surface sets `mu`, so change `mu`) | |
| `c` | Pa | cohesion / adhesion, acts on the patch area | coverings 0; sand 1–1.5 kPa; clay 4 kPa |
| `K` | m | shear deformation modulus: how much shear is needed before friction is mobilised. **The main slip parameter** at ordinary loads: s ≈ 2K·H/(μ·N·L) | hard 3–7 mm; pile 5–10 mm; soils 10–25 mm |
| `patch_len` | m | patch length on a firm base (tread deformation); a wheel's own `patch` overrides it | 0.01 |
| `slip_norm` | 0/1 | 1: the ground element is sheared (pile, foam, soil); 0: the wheel element | 1 (identified on Robotino) |

On a firm floor at ordinary traction only the ratio μ·L/(2K) is visible (the traction–slip stiffness). μ alone shows up
only when the wheels approach the friction limit (acceleration, slopes, heavy robots).

### 1.2 Losses (what the motor current pays for)
| parameter | unit | meaning | typical values |
|---|---|---|---|
| `f_roll` | – | rolling resistance of the wheel: moment f_roll·N·r through its own motor | 0–0.09 (Robotino rollers), tyres 0.01–0.05 |
| `f_lat` | – | rolling resistance of the rollers (omni / mecanum), force on the hub | table 0.006; EVA 0.13; pile / rubber 0.3–0.5 |
| `mu_rb` | – | roller bearing friction (a wheel property; a wheel's `mu_rb` overrides it) | 0.049 (Robotino) |

### 1.3 Bekker layer (deformable surfaces only)
Active when `kc` or `kphi` > 0. Pressure–sinkage p = (kc/b + kphi)·zⁿ.
| parameter | unit | meaning | typical values |
|---|---|---|---|
| `kc`, `kphi` | N/m^(n+1), N/m^(n+2) | Bekker moduli | soils: Wong's tables; pile / foam: from the wanted sinkage (§4) |
| `n` | – | sinkage exponent | soils 0.5–1.1; foam, pile 1 |
| `b_contact` | m | tread width; a wheel's own `b` overrides it | 0.03 (Robotino) |
| `layer` | m | thickness of a soft layer on a rigid base; the sinkage never exceeds it | pile 6–15 mm, mat 20 mm; soils: not set |
| `rebound` | 0…1 | elastic recovery behind the wheel: only (1 − rebound) of the compaction work is lost | soils 0; pile 0.7–0.75; foam 0.8–0.9 |

### 1.4 Heterogeneity
| parameter | meaning |
|---|---|
| `cv_res` | coefficient of variation of f_roll, f_lat, mu_rb |
| `cv_grip` | coefficient of variation of K |
| `cv_soil` | coefficient of variation of kc, kphi (soils) |
| `corr_len` | correlation length, m (Robotino floors 1 cm, soils 5 cm, Husky library 0.1–0.2 m) |
| `field_seed` | realisation of the field |

Set all `cv_*` to 0 for a perfectly uniform surface.

### 1.5 Numerical
`kn`, `cn` — contact spring-damper (2·10⁵ N/m, 300 N·s/m). Not floor properties; leave them.

Other keys in the files: `_kind` / `_group` (group used by `--surfaces real|synthetic|...`), `_doc` (description),
`_src` (where every number comes from), `_color` (colour in Gazebo and pictures). Keys starting with `_` are ignored by
the physics.

## 2. Base library (`configs/calibrated_params.json`)

| name | group | mu | K, mm | f_roll | f_lat | Bekker | what it is |
|---|---|---|---|---|---|---|---|
| `gray` | calibrated | 1.351 | 4.57 | 0.0102 | 0.434 | – | gray rubber mat |
| `green` | calibrated | 0.544 | 5.40 | 0.0886 | 0.309 | – | green plastic turf |
| `table` | calibrated | 1.0* | 7.06 | ≈0 | 0.0059 | – | chipboard table |
| `red` | calibrated | 1.0* | 3.87 | ≈0 | 0.131 | – | red EVA mat |
| `table_wet` | synthetic | 0.35 | 7.06 | ≈0 | 0.0059 | – | table with a water film |
| `carpet` | synthetic | 1.0 | 6 | 0.02 | 0.35 | layer 6 mm | carpet, 6 mm pile |
| `carpet_shag` | synthetic | 0.9 | 10 | 0.03 | 0.5 | layer 15 mm | shag carpet |
| `foam` | synthetic | 1.2 | 4 | 0 | 0.13 | layer 20 mm | polyurethane foam mat |
| `sand_lete` | synthetic | 0.603 | 12 | 0 | 0.02 | soil | dense LETE sand (Wong) |
| `sandy_loam` | extreme | 0.554 | 25 | 0 | 0.02 | soil | sandy loam, 15 % moisture (Wong) |
| `clay_wet` | extreme | 0.231 | 10 | 0 | 0.02 | soil | wet clay, 38 % moisture (Wong) |
| `mars_soil` | extreme | 0.700 | 15 | 0 | 0.02 | soil | MarsSim soil simulant (Zhou et al. 2023, Table II) |
| `tile` | hard | 0.6 | 7.06 | ≈0 | 0.0059 | – | glazed tile (table + handbook friction) |
| `linoleum` | hard | 0.7 | 7.06 | ≈0 | 0.0059 | – | linoleum |
| `concrete` | hard | 0.8 | 7.06 | ≈0 | 0.0059 | – | trowelled concrete |
| `gray_wet` | hard | 0.5 | 4.57 | 0.0102 | 0.434 | – | wet gray mat |
| `green_wet` | hard | 0.3 | 5.40 | 0.0886 | 0.309 | – | wet green turf |
| `ice` | hard | 0.1 | 7.06 | ≈0 | 0.0059 | – | ice near 0 °C |

\* On `table` and `red` the Robotino wheels never approached the friction limit, so μ could not be identified and was
fixed at 1.0; only μ/K is known there.

Groups: **calibrated** — identified on recorded runs of a real Robotino; **synthetic**, **hard**, **extreme** — built
from handbook / literature values on top of a calibrated surface. On the **extreme** soils a small robot like Robotino
(8 cm wheels, 11 kg) gets stuck, as the Bekker theory predicts for small wheels.

## 3. Tyre library (`configs/surfaces_husky.json`)

| group | surfaces | origin |
|---|---|---|
| hard | `asphalt_dry`, `asphalt_wet`, `concrete_wet`, `gravel_road`, `earth_dry`, `earth_wet`, `snow_packed`, `ice_road`, `indoor_floor` | tyre–road adhesion and rolling resistance tables (Wong, Theory of Ground Vehicles, Tables 1.1, 1.3) |
| soil | `lete_sand`, `sandy_loam_15`, `sandy_loam_32`, `clay_38`, `grenville_loam`, `snow_us`, `snow_sweden`, `mars_soil` | Bekker–Janosi sets (Wong, Table 2.3; MarsSim) |
| intermediate | `asphalt_damp`, `slush`, `loam_moist` | interpolation between two sourced entries |
| fitted | `paved_fit`, `flooring_fit`, `ice_fit`, `sandy_loam_fit`, `meadow_fit`, `gravel_fit` | fitted on recorded runs of a real Husky A200 (public datasets BorealTC and RoboNav) |

Every entry records the source of each number in `_src`: `[H]` handbook / literature, `[F]` fitted on recorded runs,
`[E]` estimate in a plausible range, `[S]` synthetic. The spread (`cv_*`, `corr_len`) of every surface was identified
from the fluctuation of the side motor currents of the real Husky (`_spread_doc` in the file).

```bash
bash scripts/drive.sh --robot husky --surface-file configs/surfaces_husky.json --surface gravel_fit --traj square:L=4,rc=0.6
bash scripts/drive.sh --robot jackal --surface-file configs/surfaces_husky.json \
     --floor "voronoi:surfaces=asphalt_dry+meadow_fit+gravel_fit,cell=2" --traj circle:R=2.5
```

## 4. Adding your own surface

### 4.1 A surface file
Write a JSON file. `base` takes every parameter of an existing surface; you change only what differs. A ready
example is [`examples/my_surfaces.json`](../../examples/my_surfaces.json):

```json
{
 "surfaces": {
  "linoleum_soft": {"base": "table", "mu": 0.8, "f_lat": 0.012, "_doc": "soft linoleum"},
  "gravel_like":   {"base": "sand_lete", "cv_soil": 0.4, "cv_res": 0.2, "corr_len": 0.10}
 },
 "speeds": {"linoleum_soft": [0.1, 0.2, 0.3]}
}
```

The top-level `"surfaces"` key is optional (a plain `{name: {...}}` dictionary works too). `speeds` is only used by
the Robotino star experiment. Then:

```bash
bash scripts/drive.sh --surface-file my.json --surface linoleum_soft --robot burger
bash scripts/drive.sh --surface-file my.json --floor "half:surfaces=gray+linoleum_soft" --robot robotino --traj line:L=3
bash scripts/robotino.sh --engine emulate --surface-file my.json --surfaces linoleum_soft,gravel_like
```

Surfaces from a file get the group `user` (`--surfaces user` in `robotino.sh`).

### 4.2 Without a file
* change one value for one run: `--set carpet.K=0.008 --set carpet.mu=0.7`;
* sweep a value (Robotino star): `bash scripts/robotino.sh --engine emulate --surfaces carpet --sweep carpet:mu=0.6,0.8,1.0`
  creates `carpet_mu0.6`, `carpet_mu0.8`, `carpet_mu1.0`;
* a surface that exists only on one map: `surfaces:` in the map file ([MAPS](MAPS.md#2-file-format)).

### 4.3 A physically sound recipe
1. **Start from the closest measured surface** (`"base": "table"`, `"gray"`, `"sand_lete"`...). Everything you do not
   change stays measured.
2. **Change one physical axis at a time:**
   * friction (water, dust, tread material) → `mu`;
   * shear compliance of pile or soil → `K`;
   * roller losses → `f_lat`; tyre or roller hysteresis → `f_roll`;
   * a soft layer → `kphi`, `n`, `layer`, `rebound`;
   * heterogeneity → `cv_*`, `corr_len`.
3. **A soft layer from the wanted sinkage.** For n = 1, wheel load N and tread width b, a rigid wheel of radius r sinks
   by z when kphi ≈ 3N / (2·b·√(2r)·z^1.5). Robotino (N ≈ 36 N, b = 0.03 m, r = 0.04 m): kphi ≈ 6400 / z^1.5
   (z = 2.5 mm → 5.1·10⁷, z = 6 mm → 1.4·10⁷ N/m³). Check: p = kphi·z should be a realistic contact pressure.
4. **Soils: take a complete published set** (kc, kphi, n, c, φ, K) — Wong, *Theory of Ground Vehicles*; Bekker;
   Chrono SCM databases. Do not mix values from different soils.
5. **Check the summary** of a test run: sinkage below `layer` and well below 0.3·r, compaction resistance / N below
   ~0.3, speed ratio above 0.1 if the surface is meant to be passable.
6. **Sweep the uncertain parameter** (`--sweep`, or a `--set` batch) to see which outputs it moves.

## 5. Which parameter moves what

| you want | change |
|---|---|
| more slip in ordinary driving | `K` ↑ |
| the robot spins its wheels when accelerating or climbing | `mu` ↓ |
| more current on a straight line, same slip | `f_roll` ↑ |
| an omni robot slips more and yaws more | `f_lat` ↑ |
| a deeper rut, more current on soft ground | `kphi` ↓ or `layer` ↑ |
| carpet cheaper in energy | `rebound` ↑ |
| more spread inside a run | `cv_res`, `cv_grip` ↑ |
