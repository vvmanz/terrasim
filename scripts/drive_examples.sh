#!/usr/bin/env bash
# Example set of the multi-robot sandbox (docs/en/RUNNING.md). One output folder per study under runs_drive/examples_<engine>/.
#   bash scripts/drive_examples.sh            # emulator (no Gazebo, a few minutes)
#   bash scripts/drive_examples.sh harmonic   # the same in Gazebo Harmonic (headless; ~20-40 min)
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENGINE="${1:-emulate}"; J="${JOBS:-4}"
OUT="$ROOT/runs_drive/examples_$ENGINE"
D="bash $ROOT/scripts/drive.sh --engine $ENGINE --jobs $J"
# 1. Robotino crosses flat surface mosaics: trajectory, deviation, currents, slip on every surface change
$D --robot robotino --floor "checkerboard:surfaces=gray+carpet+table_wet+green,cell=0.8|voronoi_mosaic:surfaces=gray+green+table+red+carpet,cell=0.6,seed=4" \
   --traj "line:L=3,angle=20|circle:R=1" --heading fixed --control open,closed --out "$OUT/1_robotino_mosaics"
# 2. Robotino: omni (fixed heading) vs tangent heading vs diff-drive mode, controller on truth / wheel odometry / odometry+IMU
$D --robot robotino --floor "stripes:surfaces=gray+carpet,w=0.7,angle=60" --traj "square:L=2,rc=0.3" --drive holonomic,diff --heading fixed,tangent \
   --localization truth,odom,odom_imu --out "$OUT/2_robotino_modes"
# 3. youBot (mecanum) on flat mosaics: figure eight, open vs closed loop
$D --robot youbot --floor "voronoi:surfaces=gray+table+carpet,cell=0.8" --traj "figure8:A=1.2" --heading fixed,tangent --control open,closed \
   --localization odom_imu --out "$OUT/3_youbot_figure8"
# 4. Husky on craters and sand/carpet patches: square and spline, open vs closed (odometry / odometry+IMU)
$D --robot husky --relief "craters:n=6,R=1.5,depth=0.15,flat0=1.5" --floor "voronoi:surfaces=gray+sand_lete+carpet,cell=2.5" \
   --traj "square:L=5,rc=0.8|spline:pts=2,1;4,-1;6,0.5" --control open,closed --localization odom,odom_imu --out "$OUT/4_husky_craters"
# 5. MarsSim rover on rough Mars-like terrain with soil patches (Gazebo: real rocker-bogie; emulator: rigid chassis)
$D --robot rover --relief "rough:A=0.04,L=3,flat0=1" --floor "voronoi:surfaces=sand_lete+mars_soil+gray,cell=3" --traj "circle:R=3|slalom:L=8,A=1,n=2" \
   --localization truth,odom_imu --out "$OUT/5_rover_rough"
# 6. All four robots, the same task: line through a half gray / half carpet floor (surface step under the robot)
$D --robot robotino,youbot,husky,rover --floor "half:surfaces=gray+carpet,offset=1.5" --traj "line:L=4" --localization odom_imu --out "$OUT/6_all_robots_step"
# 7. TurtleBot3 Burger / Waffle Pi (ball casters), Jackal, Pioneer 3-DX across gray -> shag carpet -> sand stripes
$D --robot burger,waffle,jackal,pioneer --floor "stripes:surfaces=gray+carpet_shag+sand_lete,w=1.0,angle=0" --traj "line:L=4" \
   --localization truth,odom_imu --out "$OUT/7_new_robots_stripes"
# 8. Ready map files (configs/maps, docs/en/MAPS.md): every robot up the 5..30 deg "slope ladder", and Fig. 15 mounds of growing height
$D --map slope_ladder --out "$OUT/8_slope_ladder"
$D --map marssim_mound_series --out "$OUT/8_mound_series"
# 9. Falls and rollovers: the body touches the ground; Husky and Jackal on a 55 deg slope, rolling on to the end of the run
$D --map "rollover@*" --on-tip continue --out "$OUT/9_rollover"
echo "examples: $OUT/*/summary.md"
