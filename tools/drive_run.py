"""Multi-robot sandbox: Robotino / KUKA youBot / Husky / MarsSim rover drive a trajectory over a terrain map
(relief + mosaic of surfaces), with or without a controller, with wheel odometry and an IMU at the CoM.

Normally started through scripts/drive.sh (sets up the Gazebo/ROS environment) or ros2/drive.launch.py.

  scripts/drive.sh --robot robotino --floor checkerboard:surfaces=gray+carpet,cell=0.5 --traj circle:R=1
  scripts/drive.sh --robot youbot --traj figure8:A=1.2 --heading fixed --control open,closed
  scripts/drive.sh --robot husky --relief craters:n=5,R=1.5,depth=0.15,flat0=1.5 --traj square:L=4,rc=0.6
  scripts/drive.sh --robot rover --relief rough:A=0.04,L=3,flat0=1 --floor voronoi:surfaces=sand_lete+mars_soil+gray,cell=2
  scripts/drive.sh --robot robotino,youbot,husky --traj line:L=3 --localization truth,odom,odom_imu --engine emulate
  scripts/drive.sh --robot robotino --engine harmonic --gui --rtf 1 --traj "spline:pts=1,0.5;2,-0.5;3,0"
  scripts/drive.sh --map slope_15                          # ready map file (configs/maps), its default scenario and robots
  scripts/drive.sh --map "slope_15@*" --robot husky        # every scenario of the map
  scripts/drive.sh --map bump_rows@diagonal --robot jackal --engine harmonic --gui
  scripts/drive.sh --list                                  # robots, surfaces, relief/floor specs, maps

Engines: harmonic (Gazebo Harmonic, gz-sim 8, DART), emulate (offline C++ core core/build/vehicle3d, no Gazebo, rigid chassis).
All run the same VehicleLogic (core/include/terramech/vehicle_logic.hpp).

Comma-separated values of --robot, --traj, --control, --localization, --motor-control, --heading, --floor,
--relief start one run per combination (batch).

Output (--out, default runs_drive/<date>_<engine>):
  <run>/log.csv          everything at 50 Hz (columns: docs/en/OUTPUTS.md); <run>/path_world.csv the planned path
  <run>/full.csv.gz      full per-run log (--full-rate, default 50 Hz): true pose, quaternion, world/body velocities,
                         IMU (noisy + true), ideal / encoder / encoder+gyro odometry, energy, and per wheel: angle,
                         speeds, current, voltage, torques, forces, slips (incl. dataset-style kinematic slip),
                         sinkage, local mu/K/f_roll, surface (docs/en/OUTPUTS.md)
  <run>/figure.png       map + reference/true/odometry/fused trajectories; errors, currents, slips vs time (surfaces shaded); IMU
  <run>/metrics.json     tracking errors, odometry drift, energy, per-surface currents and slips
  summary.csv / summary.md   one row per run;  per_surface.csv  per run and surface
  maps/<map>/            map.dtm (heights + surface types) and meshes for Gazebo
"""
import os, sys, json, copy, time, math, argparse, subprocess, signal, re, datetime, itertools, shutil
import numpy as np, pandas as pd
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robots as RB
import trajectories as TJ
import gen_maps
import maps as MP

EXE = os.path.join(ROOT, 'core', 'build', 'vehicle3d')

IMU_PRESETS = {    # gyro rad/s, accelerometer m/s^2: white noise sigma per sample (at --imu-rate), turn-on bias (random per seed;
                   # the gyro bias is averaged at standstill before the start and removed), bias random walk per sqrt(s)
    'none':    dict(gyro_sigma=0, gyro_bias=0, gyro_rw=0, acc_sigma=0, acc_bias=0, acc_rw=0),
    'low':     dict(gyro_sigma=3e-4, gyro_bias=1e-3, gyro_rw=2e-6, acc_sigma=0.005, acc_bias=0.02, acc_rw=2e-5),   # industrial MEMS (ADIS16470 class)
    'default': dict(gyro_sigma=0.002, gyro_bias=0.005, gyro_rw=2e-5, acc_sigma=0.03, acc_bias=0.1, acc_rw=2e-4),   # consumer MEMS (MPU-6050 / BMI088 class)
    'high':    dict(gyro_sigma=0.01, gyro_bias=0.02, gyro_rw=1e-4, acc_sigma=0.2, acc_bias=0.3, acc_rw=1e-3),      # cheap IMU + chassis vibration
}
DEFAULT_TRAJ = {'robotino': 'circle:R=1', 'youbot': 'circle:R=1', 'husky': 'circle:R=2.5', 'rover': 'circle:R=2.5',
                'burger': 'circle:R=0.8', 'waffle': 'circle:R=1', 'jackal': 'circle:R=2.5', 'pioneer': 'circle:R=1.5'}
W_REF = 11.0 * 9.81 / 3          # Robotino static wheel load (the surface library was calibrated with it)


# ------------------------------------------------------------------------------------------- params
def parse_value(v):
    try: return float(v)
    except ValueError: return v


def csv_list(s):
    return [x.strip() for x in str(s).split(',') if x.strip()] if s else []


def surfaces_lib(a):
    P = json.load(open(os.path.join(ROOT, 'configs', 'calibrated_params.json')))['robotino']
    S = copy.deepcopy(P['surfaces'])
    if a.surface_file:
        X = json.load(open(a.surface_file)); X = X.get('surfaces', X)
        for name, par in X.items():
            if name.startswith('_'): continue
            base = copy.deepcopy(S[par['base']]) if 'base' in par else {}
            base.update({k: v for k, v in par.items() if k != 'base'}); base.setdefault('_kind', 'user'); S[name] = base
    return S, P['omni_contact']


def apply_set(a, robot, S, extra):
    """--set name.param=value: name = robot | motor | wheel (all wheels) | caster (casters only) | controller | imu | contact | trajectory | body | <surface>"""
    for kv in a.set:
        key, v = kv.split('=', 1); name, par = key.split('.', 1); v = parse_value(v)
        if name == 'robot':
            if par == 'mass': robot['mass'] = v
            elif par in ('com_x', 'com_y', 'com_h'): c = list(robot['com']); c['xyz'.index(par[-1]) if par != 'com_h' else 2] = v; robot['com'] = tuple(c)
            else: robot[par] = v
        elif name == 'motor': robot['motor'][par] = v
        elif name == 'wheel':
            for w in robot['wheels']: w[par] = v
        elif name == 'caster':
            for w in robot['wheels']:
                if w['type'] == 'caster': w[par] = v
        elif name in ('controller', 'imu', 'contact', 'trajectory', 'body'): extra.setdefault(name, {})[par] = v
        elif name in S: S[name][par] = v
        else: raise SystemExit(f'--set {kv}: unknown target "{name}" (robot, motor, wheel, caster, controller, imu, contact, trajectory, body or a surface)')


def kv_lines(d):
    return [f'{k} = {v}' for k, v in d.items() if not str(k).startswith('_') and not isinstance(v, (list, dict))]


# ------------------------------------------------------------------------------------------- one job
def replay_path_array(truth):
    """recorded trajectory (t, x, y, yaw in the start frame) -> path samples s, x, y, yaw, kappa"""
    T = pd.read_csv(truth)
    x, y, yaw = T.iloc[:, 1].values, T.iloc[:, 2].values, T.iloc[:, 3].values
    s = np.r_[0.0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    keep = np.r_[True, np.diff(s) > 1e-4]
    x, y, yaw, s = x[keep], y[keep], yaw[keep], s[keep]
    if len(s) < 2: x, y, yaw, s = np.r_[0.0, 1e-3], np.r_[0.0, 0.0], np.r_[0.0, 0.0], np.r_[0.0, 1e-3]
    return np.c_[s, x, y, yaw, np.zeros_like(s)]


def write_replay_path(truth, path):
    A = replay_path_array(truth) if truth else np.c_[[0.0, 1e-3], [0.0, 1e-3], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    with open(path, 'w') as f:
        f.write('s,x,y,yaw,kappa\n')
        for r in A: f.write(','.join(f'{v:.6f}' for v in r) + '\n')


def job_name(j):
    t = j['traj'].split(':')[0]
    parts = [os.path.splitext(os.path.basename(j['robot']))[0], t, j['drive'] + ('' if j['drive'] == 'diff' else '-' + j['heading']), j['control']]
    if j['control'] == 'closed': parts.append(j['loc'])
    if j['motor'] != 'speed': parts.append('volt')
    if j.get('map_tag'): parts.insert(1, j['map_tag'])
    return '_'.join(parts)


def write_ini(j, a, robot, S, OC, extra, gazebo):
    d = j['dir']; mi = j['map']
    W = robot['mass'] * 9.81 / len(robot['wheels'])
    scale = (1.0 if (robot['name'] == 'robotino' or a.no_contact_scale) else W / W_REF) * a.kn_scale
    L = ['[general]', f"log_file = {os.path.join(d, 'log.csv')}", f"chassis_link = {robot['chassis']}", f'log_rate = {a.log_rate}',
         f'full_rate = {0 if a.no_full else a.full_rate}', f"full_log_file = {os.path.join(d, 'log_full.csv')}"]
    if getattr(a, 'ros', False): L.append('flush_log = 1')                    # --ros: every log row reaches the file at once (ROS 2 bridge follows it)
    rl = RB.ini_sections(robot, links=gazebo)
    rl.insert(rl.index('[robot]') + 2, f"drive = {j['drive']}")
    L += rl
    L += ['[omni_contact]'] + kv_lines({**OC, **extra.get('contact', {})})
    for name, s in S.items():
        s = dict(s)
        if scale != 1.0: s['kn'] = s.get('kn', 2e5) * scale; s['cn'] = s.get('cn', 300) * scale
        L += [f'[surface {name}]'] + kv_lines(s)
    L += ['[vehicle]', f"surface = {mi['surfaces'][0]}", f"spawn_x = {j['spawn'][0]}", f"spawn_y = {j['spawn'][1]}", f"spawn_yaw_deg = {j['spawn'][2]}"]
    if j.get('replay') and any(j['replay']['v0']):
        L += [f"v0_x = {j['replay']['v0'][0]}", f"v0_y = {j['replay']['v0'][1]}", f"v0_wz = {j['replay']['v0'][2]}"]
    L += ['[map]', f"dtm_file = {mi['dtm']}"]
    tr = dict(path_file=os.path.join(d, 'path.csv'), speed=j['speed'], accel=j['accel'], heading=j['heading'], t_start=a.t_start,
              t_settle=a.t_settle)
    rp = j.get('replay')
    if rp:
        tr['t_start'] = rp['t_start']
        if rp['truth']: tr['truth_file'] = rp['truth']
    tr.update(extra.get('trajectory', {}))
    L += ['[trajectory]'] + kv_lines(tr)
    ct = dict(type=j['control'], localization=j['loc'], motor=j['motor'], rate=a.ctrl_rate, v_max=robot['v_max'], w_max=robot['w_max'], **robot['ctrl'])
    if rp: ct.update(type='replay', replay_file=rp['file'], replay_mode=rp['mode'])
    ct.update(extra.get('controller', {}))
    L += ['[controller]'] + kv_lines(ct)
    im = dict(IMU_PRESETS[a.imu_noise], rate=a.imu_rate, seed=a.seed); im.update(extra.get('imu', {}))
    L += ['[imu]'] + kv_lines(im)
    L += RB.body_section(robot, on=a.body_contact == 'on', mu=a.body_mu, on_tip=a.on_tip, tip_deg=a.tip_deg, extra=extra.get('body'))
    ini = os.path.join(d, 'experiment.ini'); open(ini, 'w').write('\n'.join(L) + '\n')
    return ini


def map_height(mi, x, y):
    if 'ys' in mi: return float(MP.height_at(mi, x, y))
    return gen_maps.height_at(mi, x, y)


def spawn_pose(mi, robot, sx, sy, syaw_deg):
    """chassis pose on the terrain: tilted to the least-squares plane through the ground under the wheels (as
    core/tests/vehicle3d.cpp does), model origin base_h above that plane -> z, roll, pitch, yaw (SDF fixed-axis rpy)"""
    yaw = math.radians(syaw_deg); c, s_ = math.cos(yaw), math.sin(yaw)
    P = [(c * w['x'] - s_ * w['y'], s_ * w['x'] + c * w['y']) for w in robot['wheels']]
    h = [map_height(mi, sx + px, sy + py) for px, py in P]
    A = np.c_[np.ones(len(P)), np.array(P)]
    if len(P) >= 3 and np.linalg.matrix_rank(A) == 3: z0, bx, by = np.linalg.lstsq(A, np.array(h), rcond=None)[0]
    else: z0, bx, by = map_height(mi, sx, sy), 0.0, 0.0
    n = np.array([-bx, -by, 1.0]); n /= np.linalg.norm(n)
    hd = np.array([c, s_, 0.0]); ex = hd - n * hd.dot(n); ex /= np.linalg.norm(ex); ey = np.cross(n, ex)
    roll, pitch, yaw2 = math.atan2(ey[2], n[2]), math.asin(max(-1.0, min(1.0, -ex[2]))), math.atan2(ex[1], ex[0])
    lift = max([0.0] + [hi - (z0 + bx * px + by * py) for (px, py), hi in zip(P, h)]) / n[2]   # curved ground: no wheel in the ground
    return float(z0 + robot['base_h'] / n[2] + lift), roll, pitch, yaw2


def make_world(j, a, robot, ini):
    import gz_sdf as GE
    mi = j['map']; flavor = a.engine
    col = mi.get('colors', GE.SURF_COLOR)
    floor = [GE.mesh_visual_model(f'map_{s}', obj, col.get(s, '0.5 0.5 0.5 1')) for obj, s in mi['meshes']]
    sz = mi['size']
    floor.append(GE.static_box('floor_outside', (0, 0, float(mi['H'].min()) - 0.02), (2 * sz + 40, 2 * sz + 40, 0.01), '0.4 0.4 0.4 1', collide=False))
    floor.append(GE.static_box('safety_floor', (0, 0, float(mi['H'].min()) - 0.6), (2 * sz + 40, 2 * sz + 40, 0.1), '0.3 0.3 0.3 1'))
    sx, sy, syaw = j['spawn']
    z, roll, pitch, yaw = spawn_pose(mi, robot, sx, sy, syaw)
    model = RB.sdf_model(flavor, robot, ini, (sx, sy, z, yaw, roll, pitch))
    if a.gz_sensors and flavor == 'harmonic':
        imu, odom = RB.gz_sensors(robot)
        k = model.index('</link>'); model = model[:k] + imu + model[k:]
        model = model.replace('</model>', odom + '</model>')
    w = GE.world(flavor, f"drive_{robot['name']}", floor + [model], dt=a.dt, rtf=a.rtf)
    if a.gz_sensors and flavor == 'harmonic':
        w = w.replace('<plugin filename="gz-sim-physics-system"', '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><plugin filename="gz-sim-physics-system"', 1)
    wf = os.path.join(j['dir'], 'world.sdf'); open(wf, 'w').write(w)
    errs = GE.validate_world(wf)
    if errs: raise RuntimeError('; '.join(errs))
    return wf


def kill_group(p):
    if p is None or p.poll() is not None: return
    try:
        os.killpg(p.pid, signal.SIGTERM); time.sleep(2)
        if p.poll() is None: os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError: pass


def last_time(log):
    try:
        with open(log, 'rb') as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 8192)); L = f.read().decode(errors='ignore').strip().split('\n')
        return float(L[-1].split(',')[0])
    except Exception: return 0.0


def tail(path, n=12):
    try:
        L = open(path, errors='ignore').read().splitlines()
        L = [l for l in L if re.search(r'rr|error|terramech|vehicle', l, re.I)] or L
        return ' | '.join(L[-n:])[-1500:]
    except Exception: return ''


def ros_bridges(a, robot, log, env, con):
    """--ros: ROS 2 topics of a running job.
    * terrasim_bridge.py follows <run>/log.csv and publishes /terrasim/... (ground truth, encoder odometry, IMU, wheel
      speeds, torques, motor currents, slip, sinkage, contact forces) -- any engine; /clock from the log for the
      offline core;
    * Gazebo Harmonic: ros_gz_bridge for the native gz topics (/clock, /imu, odometry, joint states) in the fixed
      GZ_PARTITION (default 'terrasim'), so that `GZ_PARTITION=terrasim gz topic -l` lists them too."""
    out = []
    if shutil.which('ros2') is None:
        print('   --ros: ros2 not found (source /opt/ros/jazzy/setup.bash first); topics are not published', flush=True); return out
    py = [sys.executable, os.path.join(ROOT, 'ros2', 'terrasim_bridge.py'), '--ros-args', '-p', f'log:={log}',
          '-p', f'clock:={"false" if a.engine == "harmonic" else "true"}', '-p', 'wheel_labels:=' + ','.join(w['label'] for w in robot['wheels'])]
    out.append(subprocess.Popen(py, stdout=con, stderr=subprocess.STDOUT, env=env, start_new_session=True))
    if a.engine == 'harmonic':
        name = robot['name']
        topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                  f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry', '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model']
        out.append(subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + topics, stdout=con, stderr=subprocess.STDOUT, env=env,
                                    start_new_session=True))
    print(f'   --ros: topics /terrasim/* ' + ('+ /clock /imu /joint_states /model/<robot>/odometry (GZ_PARTITION=' + env.get('GZ_PARTITION', '') + ')'
                                              if a.engine == 'harmonic' else '+ /clock'), flush=True)
    return out


def run_job(j, a, robot, S, OC, extra, slot):
    d = j['dir']; os.makedirs(d, exist_ok=True)
    log = os.path.join(d, 'log.csv'); done = log + '.done'
    for f in (log, done, os.path.join(d, 'log_full.csv'), os.path.join(d, 'full.csv.gz')):
        if os.path.exists(f): os.remove(f)
    if j.get('replay'): write_replay_path(j['replay']['truth'], os.path.join(d, 'path.csv'))
    else: TJ.write(j['traj'], os.path.join(d, 'path.csv'))
    gazebo = a.engine == 'harmonic'
    ini = write_ini(j, a, robot, j.get('S', S), OC, extra, gazebo)
    t_end = j.get('replay', {}).get('t_start', a.t_start) + j['T'] + a.t_settle + 0.05
    env = dict(os.environ, TM_LOG=log, TM_DONE=done)
    con = open(os.path.join(d, 'console.log'), 'w'); procs = []; t0 = time.time()
    if gazebo:
        wf = make_world(j, a, robot, ini)
        iters = int(t_end / a.dt) + 500
        env['GZ_PARTITION'] = a.gz_partition or f'terrasim_{os.getpid()}_{slot}'
        cmd = ['gz', 'sim', '-r', '-v', '3', wf] if a.gui else ['gz', 'sim', '-s', '-r', '-v', '3', '--iterations', str(iters), wf]
        procs.append(subprocess.Popen(cmd, stdout=con, stderr=subprocess.STDOUT, env=env, start_new_session=True))
    else:
        procs.append(subprocess.Popen([EXE, ini, '--dt', str(a.dt)], stdout=con, stderr=subprocess.STDOUT, env=env, start_new_session=True))
    if a.ros: procs += ros_bridges(a, robot, log, env, con)
    status, last_size, last_change, last_print = 'finished', -1, time.time(), time.time()
    stall = 1e9 if a.gui else a.stall
    while procs[0].poll() is None:
        time.sleep(1.0 if gazebo else 0.1)
        now = time.time()
        if os.path.exists(done): status = 'done'; time.sleep(0.5 if gazebo else 0.0); break
        size = os.path.getsize(log) if os.path.exists(log) else 0
        if size != last_size: last_size, last_change = size, now
        if now - last_change > stall: status = 'stalled'; break
        if now - last_print >= a.progress:
            last_print = now; ts = last_time(log)
            print(f'   [{j["name"]}] sim {ts:6.1f} / {t_end:.0f} s, wall {now - t0:5.0f} s, RTF {ts / max(now - t0, 1e-9):5.1f}', flush=True)
    for p in procs[::-1]: kill_group(p)
    con.close()
    if os.path.exists(done) and status == 'finished': status = 'done'
    wall = time.time() - t0; ts = last_time(log)
    res = dict(run=j['name'], robot=j['robot'], traj=j['traj'], drive=j['drive'], heading=j['heading'], control=j['control'],
               localization=j['loc'] if j['control'] == 'closed' else '-', motor=j['motor'], map=j['map']['label'], speed=j['speed'],
               relief=j.get('relief', ''), floor=j.get('floor', ''), scenario=j.get('scenario', ''),
               status=status, wall_s=round(wall, 1), sim_s=round(ts, 1), rtf=round(ts / max(wall, 1e-9), 1))
    if not os.path.exists(log) or os.path.getsize(log) < 2000:
        res['status'] = 'FAILED'; res['error'] = tail(os.path.join(d, 'console.log')); return res, None
    if 'diverged' in open(os.path.join(d, 'console.log'), errors='ignore').read(): res['status'] = 'diverged'
    raw_full = os.path.join(d, 'log_full.csv')
    if not a.no_full and os.path.exists(raw_full):
        try:
            res['full_rows'] = export_full(raw_full, os.path.join(d, 'full.csv.gz'), j)
            os.remove(raw_full)
        except Exception as e:
            res['error'] = f'full log: {e!r}'
    try:
        m, ps = analyse(j, d)
        res.update(m)
        if m.get('tipped_t') is not None and res['status'] in ('done', 'finished', 'diverged'): res['status'] = 'tipped'
        elif m.get('left_map_t') is not None and res['status'] in ('done', 'finished', 'diverged'): res['status'] = 'left_map'
    except Exception as e:
        res['error'] = f'analysis: {e!r}'; ps = None
    return res, ps


def export_full(src, dst, j):
    """raw log_full.csv -> <run>/full.csv.gz with the run description in front; a truncated last line is dropped"""
    F = pd.read_csv(src, on_bad_lines='skip')
    F = F[pd.to_numeric(F.t, errors='coerce').notna()].dropna(subset=[F.columns[-1]])
    meta = pd.DataFrame(dict(robot=j['robot'], control=j['control'], localization=j['loc'] if j['control'] == 'closed' else '-',
                             map=j['map']['label']), index=F.index)
    F = pd.concat([F.iloc[:, :1], meta, F.iloc[:, 1:]], axis=1)
    F.to_csv(dst, index=False, float_format='%.7g', compression='gzip')
    return len(F)


# ------------------------------------------------------------------------------------------- analysis
def world_path(j, D):
    P = pd.read_csv(os.path.join(j['dir'], 'path.csv'))
    g = D[D.phase >= 1]
    x0, y0, yaw0 = (g.x.iloc[0], g.y.iloc[0], g.yaw.iloc[0]) if len(g) else (j['spawn'][0], j['spawn'][1], math.radians(j['spawn'][2]))
    c, s = math.cos(yaw0), math.sin(yaw0)
    return x0 + c * P.x - s * P.y, y0 + s * P.x + c * P.y


TIP_DEG = 50.0      # default --tip-deg: chassis tilted this much relative to the ground normal = tipped over


def tilt_rel_deg(D, mi):
    """angle between the chassis z axis (log roll/pitch/yaw, ZYX) and the terrain normal under the chassis origin"""
    G = mi if 'ys' in mi else dict(mi, ys=mi['xs'])
    gy, gx = np.gradient(G['H'], G['ys'], G['xs'])
    hx = MP.height_at(dict(G, H=gx), D.x.values, D.y.values); hy = MP.height_at(dict(G, H=gy), D.x.values, D.y.values)
    n = np.c_[-hx, -hy, np.ones(len(D))]; n /= np.linalg.norm(n, axis=1, keepdims=True)
    cr, sr, cp, sp, cy, sy = np.cos(D.roll), np.sin(D.roll), np.cos(D.pitch), np.sin(D.pitch), np.cos(D.yaw), np.sin(D.yaw)
    zb = np.c_[cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr]
    return np.degrees(np.arccos(np.clip((zb * n).sum(axis=1), -1, 1)))


def along_path(X, Y, px, py):
    """arc-length position on the reference path of every sample, followed continuously from the start (works for
    loops, robots that stop, slide back or leave the path sideways)"""
    S = np.r_[0.0, np.cumsum(np.hypot(np.diff(X), np.diff(Y)))]
    out = np.zeros(len(px)); k = 0
    ds = max(float(S[-1]) / max(len(S) - 1, 1), 1e-4); w = int(0.5 / ds) + 2
    for t in range(len(px)):
        a, b = max(0, k - w), min(len(S), k + w)
        k = a + int(np.argmin((X[a:b] - px[t]) ** 2 + (Y[a:b] - py[t]) ** 2)); out[t] = S[k]
    return out, float(S[-1])


def analyse(j, d):
    from scipy.spatial import cKDTree
    D = pd.read_csv(os.path.join(d, 'log.csv'), on_bad_lines='skip')
    D = D[pd.to_numeric(D.t, errors='coerce').notna()].dropna(subset=[D.columns[-1]]).reset_index(drop=True).copy()   # last line may be cut when the engine is killed
    mi0 = j['map']
    xmin, xmax, ymin, ymax = mi0.get('bounds') or (mi0['xs'][0], mi0['xs'][-1], mi0['xs'][0], mi0['xs'][-1])
    out = (D.x < xmin) | (D.x > xmax) | (D.y < ymin) | (D.y > ymax)
    t_out = float(D.t[out].iloc[0]) if out.any() else None                       # beyond the map the ground is extrapolated
    if t_out is not None: D = D[D.t <= t_out].reset_index(drop=True).copy()
    tr = tilt_rel_deg(D, mi0); tip_deg = j.get('tip_deg', TIP_DEG)
    t_tip = float(D.t[tr > tip_deg].iloc[0]) if (tr > tip_deg).any() else None
    if t_tip is not None and j.get('on_tip', 'stop') == 'stop':                  # --on-tip stop: the run ends where the robot tipped over
        D = D[D.t <= t_tip].reset_index(drop=True).copy(); tr = tr[:len(D)]
    nw = len([c for c in D.columns if re.fullmatch(r'w\d+_cur', c)])
    types = [w['type'] for w in RB.robot(j['robot'])['wheels']][:nw]
    drv = [i for i in range(1, nw + 1) if i > len(types) or types[i - 1] != 'caster']      # driven wheels (casters excluded)
    cas = [i for i in range(1, nw + 1) if i not in drv]
    names = j['map']['surfaces']
    X, Y = world_path(j, D)
    pd.DataFrame(dict(x=X, y=Y)).to_csv(os.path.join(d, 'path_world.csv'), index=False)
    tree = cKDTree(np.c_[X, Y])
    mv = D[D.phase == 1]; af = D[D.phase >= 1]
    if len(af) < 2:                          # tipped over / slid off the map before the motion started
        m = dict(n_wheels=nw, progress=0.0, progress_max=0.0, driven_ratio=0.0, tipped_t=round(t_tip, 2) if t_tip is not None else None,
                 left_map_t=round(t_out, 2) if (t_out is not None and t_tip is None) else None, tilt_rel_max_deg=round(float(tr.max()), 1),
                 note='before the start of the motion')
        json.dump(dict(m, job={k: v for k, v in j.items() if k not in ('map', 'S')}), open(os.path.join(d, 'metrics.json'), 'w'), indent=1, default=str)
        return m, []
    dist, _ = tree.query(af[['x', 'y']].values)
    D['xtrack'] = np.nan; D.loc[af.index, 'xtrack'] = dist
    et = np.hypot(mv.x - mv.ref_x, mv.y - mv.ref_y)
    dt = float(np.median(np.diff(D.t))) if len(D) > 2 else 0.02
    groups = {}
    for i in drv:
        key = (round(float(D[f'w{i}_cur'].iloc[-1]), 9), round(float(D[f'w{i}_vol'].iloc[-1]), 9), round(float(D[f'w{i}_cur'].mean()), 9))
        groups.setdefault(key, i)
    motors = sorted(groups.values())
    E = sum(float(af[f'w{i}_pow'].sum()) * dt for i in motors)
    dist_driven = float(np.hypot(np.diff(af.x), np.diff(af.y)).sum()) if len(af) > 1 else 0.0
    if len(af) > 1:
        Xa, Ya = np.asarray(X), np.asarray(Y)
        s_af, L_path = along_path(Xa, Ya, af.x.values, af.y.values)
        s_af = s_af - s_af[0]
    else: s_af, L_path = np.zeros(1), 1.0
    fin = D.iloc[-1]
    m = dict(
        n_wheels=nw, n_motors=len(motors), path_len_m=round(float(np.hypot(np.diff(X), np.diff(Y)).sum()), 3), driven_m=round(dist_driven, 3),
        T_move_s=round(float(mv.t.iloc[-1] - mv.t.iloc[0]) if len(mv) else 0.0, 2),
        track_err_rms_m=round(float(np.sqrt(np.mean(et ** 2))), 4) if len(mv) else None, track_err_max_m=round(float(et.max()), 4) if len(mv) else None,
        xtrack_rms_m=round(float(np.sqrt(np.mean(dist ** 2))), 4), xtrack_max_m=round(float(dist.max()), 4),
        final_err_m=round(float(math.hypot(fin.x - X.iloc[-1], fin.y - Y.iloc[-1])), 4),
        yaw_err_rms_deg=round(float(np.degrees(np.sqrt(np.mean(mv.e_yaw ** 2)))), 2) if len(mv) else None,
        odom_drift_m=round(float(math.hypot(fin.odo_x - fin.x, fin.odo_y - fin.y)), 4),
        odom_yaw_drift_deg=round(float(np.degrees(abs(math.remainder(fin.odo_yaw - fin.yaw, 2 * math.pi)))), 2),
        fused_drift_m=round(float(math.hypot(fin.fus_x - fin.x, fin.fus_y - fin.y)), 4),
        fused_yaw_drift_deg=round(float(np.degrees(abs(math.remainder(fin.fus_yaw - fin.yaw, 2 * math.pi)))), 2),
        energy_J=round(E, 1), energy_J_per_m=round(E / max(dist_driven, 1e-6), 1),
        I_mean_A=round(float(np.mean([mv[f'w{i}_cur'].mean() for i in motors])), 3) if len(mv) else None,
        I_max_A=round(float(max(mv[f'w{i}_cur'].max() for i in motors)), 3) if len(mv) else None,
        slip_mean=round(float(np.mean([mv[f'w{i}_slip'].abs().mean() for i in drv])), 4) if len(mv) else None,
        slip_lat_mean=round(float(np.mean([mv[f'w{i}_slip_lat'].abs().mean() for i in drv])), 4) if len(mv) else None,
        sink_caster_max_mm=round(float(max(D[f'w{i}_sink'].max() for i in cas) * 1000), 2) if cas else None,
        progress=round(float(s_af[-1] / max(L_path, 1e-9)), 3),              # along the path at the end (sliding back counts negative)
        progress_max=round(float(s_af.max() / max(L_path, 1e-9)), 3),
        driven_ratio=round(float(dist_driven / max(L_path, 1e-9)), 3),        # distance driven / path (v7 'progress')
        tipped_t=round(t_tip, 2) if t_tip is not None else None, left_map_t=round(t_out, 2) if (t_out is not None and t_tip is None) else None,
        tilt_rel_max_deg=round(float(tr.max()), 1),
        sink_max_mm=round(float(max(D[f'w{i}_sink'].max() for i in drv) * 1000), 2),
        tilt_max_deg=round(float(np.degrees(max(D.roll.abs().max(), D.pitch.abs().max()))), 2),
        surface_changes=int((mv.surf != mv.surf.shift()).sum() - 1) if len(mv) else 0)
    mi = j['map']
    if len(af) > 1:                          # terrain under the driven path (centre of the chassis frame)
        G = mi if 'ys' in mi else dict(mi, ys=mi['xs'])
        zt = np.asarray(MP.height_at(G, af.x.values, af.y.values))
        gy, gx = np.gradient(G['H'], G['ys'], G['xs'])
        sl = MP.height_at(dict(G, H=np.degrees(np.arctan(np.hypot(gx, gy)))), af.x.values, af.y.values)
        m.update(climb_m=round(float(zt.max() - zt[0]), 3), descent_m=round(float(zt[0] - zt.min()), 3),
                 terrain_slope_max_deg=round(float(np.max(sl)), 1),
                 terrain_slope_end_deg=round(float(sl[-1]), 1))
    # per surface (wheel samples while moving)
    rows = []
    for k, sname in enumerate(names):
        cur, sl, sll, sk, n = [], [], [], [], 0
        for i in drv:
            g = mv[mv[f'w{i}_surf'] == k]
            if not len(g): continue
            cur += list(g[f'w{i}_cur']); sl += list(g[f'w{i}_slip'].abs()); sll += list(g[f'w{i}_slip_lat'].abs()); sk += list(g[f'w{i}_sink']); n += len(g)
        if n: rows.append(dict(run=j['name'], surface=sname, share=round(n / max(len(drv) * len(mv), 1), 3), I_mean_A=round(float(np.mean(cur)), 3),
                               slip_mean=round(float(np.mean(sl)), 4), slip_lat_mean=round(float(np.mean(sll)), 4), sink_mm=round(float(np.mean(sk)) * 1000, 2)))
    json.dump(dict(m, per_surface=rows, job={k: v for k, v in j.items() if k not in ('map', 'S')}), open(os.path.join(d, 'metrics.json'), 'w'), indent=1, default=str)
    D.to_pickle(os.path.join(d, '.log.pkl'))
    return m, rows


def _surf_rgb(name):
    import gz_sdf as GE
    return [float(v) for v in GE.SURF_COLOR.get(name, '0.6 0.6 0.6 1').split()[:3]]


def _shade(ax, D, names):
    mv = D[D.phase >= 1]
    if not len(mv): return
    s = mv.surf.values; t = mv.t.values; k0 = 0
    for k in range(1, len(s) + 1):
        if k == len(s) or s[k] != s[k0]:
            ax.axvspan(t[k0], t[min(k, len(s) - 1)], color=_surf_rgb(s[k0]), alpha=0.18, lw=0); k0 = k


def figure(j, d, m):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    D = pd.read_pickle(os.path.join(d, '.log.pkl'))
    mi = j['map']; names = mi['surfaces']; nw = m['n_wheels']
    W = pd.read_csv(os.path.join(d, 'path_world.csv'))
    fig = plt.figure(figsize=(19, 10.5)); gs = fig.add_gridspec(3, 3, width_ratios=[1.35, 1, 1])
    ax = fig.add_subplot(gs[:, 0])
    xs = mi['xs']; ys = mi.get('ys', xs); T = mi['T']; H = mi['H']
    col = mi.get('colors', {})
    rgb = np.array([[float(v) for v in col[s].split()[:3]] if s in col else _surf_rgb(s) for s in names])[T]
    ext = [xs[0], xs[-1], ys[0], ys[-1]]
    ax.imshow(rgb, origin='lower', extent=ext, alpha=0.75, interpolation='nearest')
    if H.max() - H.min() > 1e-4:
        cs = ax.contour(xs, ys, H * 1000, levels=10, colors='k', linewidths=0.4, alpha=0.6); ax.clabel(cs, fontsize=6, fmt='%.0f')
    ax.plot(W.x, W.y, 'k--', lw=1.4, label='reference path')
    mvv = D[D.phase >= 1]
    ax.plot(mvv.x, mvv.y, '-', color='#d62728', lw=1.8, label='true (engine)')
    ax.plot(mvv.odo_x, mvv.odo_y, '-', color='#1f77b4', lw=1, label='wheel odometry')
    ax.plot(mvv.fus_x, mvv.fus_y, '-', color='#2ca02c', lw=1, label='odometry + IMU (heading)')
    ax.plot(*j['spawn'][:2], 'ko', ms=6)
    xx = np.r_[W.x, mvv.x]; yy = np.r_[W.y, mvv.y]
    cx, cy = 0.5 * (xx.min() + xx.max()), 0.5 * (yy.min() + yy.max())
    hw = 0.5 * max(xx.max() - xx.min(), yy.max() - yy.min()) * 1.12 + 0.3      # square view around the paths
    ax.set_xlim(cx - hw, cx + hw); ax.set_ylim(cy - hw, cy + hw)
    ax.set_aspect('equal'); ax.grid(alpha=.25); ax.set_xlabel('x, m'); ax.set_ylabel('y, m')
    h, l = ax.get_legend_handles_labels()
    h += [Patch(color=_surf_rgb(s), alpha=.75) for s in names]; l += names
    ax.legend(h, l, fontsize=8, loc='upper left', framealpha=.85)
    ax.set_title(f"{j['robot']}: {j['traj']} | {j['drive']}/{j['heading']} | {j['control']}"
                 + (f"/{j['loc']}" if j['control'] == 'closed' else '') + f" | {j['motor']}\nmap {mi['label']} (contours: height, mm)", fontsize=10)
    t = D.t; mv = D.phase >= 1
    a1 = fig.add_subplot(gs[0, 1]); _shade(a1, D, names)
    a1.plot(t[mv], D.xtrack[mv] * 100, 'k', lw=1.2, label='distance to the path, cm')
    a1.plot(t[mv], D.e_cross[mv] * 100, 'r', lw=.8, label='cross-track to the reference point, cm')
    a1.plot(t[mv], D.e_along[mv] * 100, 'b', lw=.8, label='along-track (lag), cm')
    a1b = a1.twinx(); a1b.plot(t[mv], np.degrees(D.e_yaw[mv]), 'g', lw=.7); a1b.set_ylabel('heading error, °', color='g')
    a1.set_title(f"tracking errors: RMS distance to the path {m['xtrack_rms_m'] * 100:.1f} cm, max {m['xtrack_max_m'] * 100:.1f} cm", fontsize=9)
    a1.legend(fontsize=7); a1.grid(alpha=.3)
    a2 = fig.add_subplot(gs[1, 1]); _shade(a2, D, names)
    seen = set()
    drv = [i for i in range(1, nw + 1) if (i > len(RB.robot(j['robot'])['wheels'])) or RB.robot(j['robot'])['wheels'][i - 1]['type'] != 'caster']
    for i in drv:
        key = (round(D[f'w{i}_cur'].mean(), 6), round(D[f'w{i}_vol'].mean(), 6))
        if key in seen: continue
        seen.add(key); a2.plot(t, D[f'w{i}_cur'], lw=.8, label=f'motor of wheel {i}')
    a2.set_title(f"motor currents, A (energy {m['energy_J']:.0f} J, {m['energy_J_per_m']:.0f} J/m)", fontsize=9); a2.legend(fontsize=7, ncol=2); a2.grid(alpha=.3)
    a3 = fig.add_subplot(gs[2, 1]); _shade(a3, D, names)
    for i in drv: a3.plot(t[mv], D[f'w{i}_slip'][mv], lw=.7, label=f'wheel {i}')
    lo, hi = np.nanpercentile(np.r_[[D[f'w{i}_slip'][mv] for i in drv]], [0.5, 99.5]) if mv.any() else (-1, 1)
    a3.set_ylim(min(-0.2, lo * 1.2), max(0.6, min(hi * 1.2, 3.0)))
    a3.set_title('longitudinal slip in the contact patch', fontsize=9); a3.legend(fontsize=7, ncol=3); a3.grid(alpha=.3); a3.set_xlabel('t, s')
    a4 = fig.add_subplot(gs[0, 2]); _shade(a4, D, names)
    a4.plot(t[mv], np.hypot(D.odo_x - D.x, D.odo_y - D.y)[mv] * 100, 'b', lw=1, label='odometry, cm')
    a4.plot(t[mv], np.hypot(D.fus_x - D.x, D.fus_y - D.y)[mv] * 100, 'g', lw=1, label='odometry + IMU, cm')
    a4b = a4.twinx()
    a4b.plot(t[mv], np.degrees(np.unwrap(D.odo_yaw[mv]) - np.unwrap(D.yaw[mv])), 'b:', lw=.8)
    a4b.plot(t[mv], np.degrees(np.unwrap(D.fus_yaw[mv]) - np.unwrap(D.yaw[mv])), 'g:', lw=.8); a4b.set_ylabel('heading error (dotted), °')
    a4.set_title(f"position estimate drift: odometry {m['odom_drift_m'] * 100:.1f} cm, odometry + IMU {m['fused_drift_m'] * 100:.1f} cm", fontsize=9)
    a4.legend(fontsize=7); a4.grid(alpha=.3)
    a5 = fig.add_subplot(gs[1, 2])
    a5.plot(t, D.imu_gz, color='0.6', lw=.5, label='gyro z (IMU, with noise)'); a5.plot(t, D.wz, 'k', lw=1, label='true ωz')
    a5.plot(t, D.cmd_wz, 'r--', lw=.8, label='command ωz'); a5.set_title('IMU at the CoM: angular rate, rad/s', fontsize=9); a5.legend(fontsize=7); a5.grid(alpha=.3)
    a6 = fig.add_subplot(gs[2, 2])
    a6.plot(t, D.imu_ax, lw=.5, label='accelerometer x'); a6.plot(t, D.imu_ay, lw=.5, label='accelerometer y')
    a6.plot(t, D.imu_az - 9.81, lw=.5, label='accelerometer z − g'); a6.set_ylim(-3, 3)
    a6b = a6.twinx(); a6b.plot(t, np.hypot(D.vx, D.vy), 'k', lw=1); a6b.plot(t, D.ref_v, 'k--', lw=.8); a6b.set_ylabel('speed (actual / reference), m/s')
    a6.set_title('IMU: specific force, m/s²', fontsize=9); a6.legend(fontsize=7, loc='lower left'); a6.grid(alpha=.3); a6.set_xlabel('t, s')
    plt.tight_layout(); plt.savefig(os.path.join(d, 'figure.png'), dpi=75); plt.close(fig)


def batch_figure(out, R):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    R = R[R.xtrack_rms_m.notna()] if 'xtrack_rms_m' in R else R
    if len(R) < 2: return
    fig, ax = plt.subplots(1, 3, figsize=(18, max(3.5, 0.32 * len(R) + 1.5)))
    y = np.arange(len(R))
    ax[0].barh(y, R.xtrack_rms_m * 100, color='k', alpha=.7, label='RMS'); ax[0].barh(y, R.xtrack_max_m * 100, color='k', alpha=.2, label='max')
    ax[0].set_yticks(y); ax[0].set_yticklabels(R.run, fontsize=7); ax[0].set_xlabel('deviation from the path, cm'); ax[0].legend(fontsize=8)
    ax[1].barh(y - .2, R.odom_drift_m * 100, .4, label='odometry'); ax[1].barh(y + .2, R.fused_drift_m * 100, .4, label='odometry + IMU')
    ax[1].set_yticks(y); ax[1].set_yticklabels([]); ax[1].set_xlabel('estimate drift at the end, cm'); ax[1].legend(fontsize=8)
    ax[2].barh(y, R.energy_J_per_m, color='tab:orange'); ax[2].set_yticks(y); ax[2].set_yticklabels([]); ax[2].set_xlabel('energy, J/m')
    for a_ in ax: a_.grid(alpha=.3, axis='x'); a_.invert_yaxis()
    plt.tight_layout(); plt.savefig(os.path.join(out, 'summary.png'), dpi=80); plt.close(fig)


# ------------------------------------------------------------------------------------------- main
def list_all():
    S, _ = surfaces_lib(argparse.Namespace(surface_file=None))
    print('robots:')
    for n in list(RB.ROBOTS) + RB.user_robots():
        r = RB.robot(n)
        nc = sum(w['type'] == 'caster' for w in r['wheels']); cs = f' + {nc} caster' if nc else ''
        print(f"  {n:9s} {r['mass']:6.1f} kg, {len(r['wheels']) - nc} wheels{cs} ({r['wheels'][0]['type']}, r = {r['wheels'][0]['r']} m), "
              f"drive {'/'.join(r['drive'])}, default speed {r['speed']} m/s, map half-size {r['map_size']} m")
    print('surfaces:', ', '.join(f"{k} ({v.get('_kind', '')})" for k, v in S.items()))
    print('trajectories:', TJ.__doc__.split('Specs')[1].split('Every')[0])
    print('relief:', gen_maps.relief.__doc__, '\n  flat | hills:A,L,r0,r1 | bumps:n,h,w,signed | craters:n,R,depth,rim | waves:A,L,angle | ramp:angle,x0,x1 | rough:A,L,beta,octaves'
          '\n  common: seed, flat0 (flat disc of this radius around the map centre)')
    print('floor: uniform:surface=S | checkerboard:surfaces=A+B,cell | voronoi:surfaces=A+B+C,cell,seed | stripes:surfaces,w,angle | rings:surfaces,w,r0 |'
          ' sectors:surfaces | half:surfaces=A+B,angle,offset   (r0start=R: surface 0 within R of the centre)')
    print('IMU noise presets:', ', '.join(f'{k} {v}' for k, v in IMU_PRESETS.items()))
    print('maps (--map NAME[@scenario], configs/maps; details: python3 tools/maps.py list / show NAME):')
    for n in MP.all_names():
        M, _ = MP.load(n); print(f"  {n:22s} {M['description'][:70]:70s} scenarios: {', '.join(M['scenarios'])}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default=None, help='robotino, youbot, husky, rover, burger, waffle, jackal, pioneer or a robot file (configs/robots/NAME.json, path.json) (comma list = batch); '
                    'default robotino, with --map the robots listed in the map file')
    ap.add_argument('--engine', default='emulate', choices=['harmonic', 'emulate'])
    ap.add_argument('--map', default='', help='map file(s) from configs/maps or a path: NAME, NAME@scenario, "NAME@*" (all scenarios); '
                    'comma list = batch. Replaces --relief/--floor; spawn, trajectory and speed come from the scenario unless given')
    ap.add_argument('--replay', default='', help='replay recorded commands (dataset): CSV t,g0,g1,... (wheel speeds per motor group, '
                    'rad/s) or t,vx,vy,wz (--replay-mode twist); replaces --traj')
    ap.add_argument('--replay-mode', default='wheels', choices=['wheels', 'twist'])
    ap.add_argument('--truth', default='', help='with --replay: recorded trajectory t,x,y,yaw in the start frame (reference in the logs)')
    ap.add_argument('--v0', default='', help='with --replay: initial body velocity vx,vy,wz (the recording starts while driving; emulator)')
    ap.add_argument('--map-set', action='append', default=[], help='change a value of the map file: relief.0.slope_deg=12, scenarios.up.traj=line:L=3')
    ap.add_argument('--relief', default='flat', help="relief spec; ';' adds reliefs, '|' separates batch values (quote it in the shell)")
    ap.add_argument('--floor', default='', help='floor pattern spec (default uniform:surface=<--surface>); "|" separates batch values')
    ap.add_argument('--surface', default=None, help='surface of a uniform floor / default surface (gray); with --map: replaces the '
                    "map's default surface, its zones stay")
    ap.add_argument('--map-size', type=float, default=0, help='half-width of the square map, m (0 = robot default, enlarged to fit the path)')
    ap.add_argument('--res', type=float, default=0, help='map node spacing, m (0 = robot default)')
    ap.add_argument('--spawn', default=None, help='x,y,yaw_deg of the spawn point (map frame); default 0,0,0 or the map scenario')
    ap.add_argument('--traj', default='', help='trajectory spec (tools/trajectories.py); "|" separates batch values; default per robot')
    ap.add_argument('--speed', type=float, default=0, help='cruise speed, m/s (0 = robot default)')
    ap.add_argument('--accel', type=float, default=0, help='acceleration of the speed profile, m/s^2 (0 = robot default)')
    ap.add_argument('--drive', default='', help='holonomic | diff (default: holonomic for robotino/youbot, diff for husky/rover)')
    ap.add_argument('--heading', default='tangent', help='holonomic only: tangent (faces along the path) | fixed (keeps spawn yaw, drives sideways)')
    ap.add_argument('--control', default='closed', help='open (feed-forward, NO controller) | closed (path controller); comma list = batch')
    ap.add_argument('--localization', default='truth', help='closed loop feedback: truth | odom | odom_imu; comma list = batch')
    ap.add_argument('--motor-control', default='speed', help='speed (motor PI speed loop) | voltage (U = Ce*w_set, no motor loop)')
    ap.add_argument('--imu-noise', default='default', choices=list(IMU_PRESETS))
    ap.add_argument('--imu-rate', type=float, default=100)
    ap.add_argument('--ctrl-rate', type=float, default=50)
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--t-start', type=float, default=1.5, help='standstill before the motion (IMU bias calibration), s')
    ap.add_argument('--t-settle', type=float, default=2.0)
    ap.add_argument('--set', action='append', default=[], help='name.param=value (robot, motor, wheel, caster, controller, imu, contact, trajectory, <surface>)')
    ap.add_argument('--surface-file', default=None)
    ap.add_argument('--body-contact', choices=['on', 'off'], default='on',
                    help='body-ground contact of the chassis (falls, rollovers, robot lying on a bump); off = the body passes through the ground')
    ap.add_argument('--on-tip', choices=['stop', 'continue'], default='stop',
                    help='when the chassis tilts more than --tip-deg to the ground normal: stop the run (default) or let it go on (rollover)')
    ap.add_argument('--tip-deg', type=float, default=50.0, help='tilt of the chassis to the ground normal that counts as tipped over, deg')
    ap.add_argument('--body-mu', type=float, default=0.6, help='friction coefficient of the body on the ground')
    ap.add_argument('--no-contact-scale', action='store_true', help='do not scale the penalty contact kn/cn with the wheel load')
    ap.add_argument('--kn-scale', type=float, default=1.0, help='extra factor on the normal contact stiffness/damping (tyre compliance; <1 = softer wheels)')
    ap.add_argument('--gui', action='store_true'); ap.add_argument('--rtf', type=float, default=None)
    ap.add_argument('--gz-sensors', action='store_true', help='Harmonic: also add native gz IMU sensor (/imu) + OdometryPublisher (/model/<robot>/odometry)')
    ap.add_argument('--gz-partition', default='', help='Harmonic: fixed GZ_PARTITION (to connect ros_gz_bridge / gz topic); default unique per job')
    ap.add_argument('--ros', action='store_true', help='publish the running job to ROS 2 (/terrasim/*; Harmonic: also native gz topics through '
                    'ros_gz_bridge, GZ_PARTITION=terrasim unless --gz-partition); implies --jobs 1 and, on Harmonic, --gz-sensors')
    ap.add_argument('--dt', type=float, default=0.001); ap.add_argument('--log-rate', type=float, default=50)
    ap.add_argument('--jobs', type=int, default=1); ap.add_argument('--stall', type=float, default=120); ap.add_argument('--progress', type=float, default=15)
    ap.add_argument('--out', default=None); ap.add_argument('--no-figures', action='store_true')
    ap.add_argument('--full-rate', type=float, default=50, help='rate of <run>/full.csv.gz (full per-run log), Hz')
    ap.add_argument('--no-full', action='store_true', help='do not write <run>/full.csv.gz')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list: list_all(); return
    if a.gui and a.rtf is None: a.rtf = 1.0
    if a.gui: a.jobs = 1
    if a.ros:
        a.jobs = 1
        if a.engine == 'harmonic': a.gz_sensors = True; a.gz_partition = a.gz_partition or 'terrasim'
    if a.engine == 'emulate' and not os.path.exists(EXE): raise SystemExit('core/build/vehicle3d missing: bash scripts/build.sh core')
    a.out = os.path.abspath(a.out or os.path.join(ROOT, 'runs_drive', datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + a.engine))
    os.makedirs(a.out, exist_ok=True)
    S, OC = surfaces_lib(a)
    spawn_cli = None
    if a.spawn: sp = [float(v) for v in a.spawn.split(',')] + [0, 0, 0]; spawn_cli = tuple(sp[:3])
    jobs, robots_cfg, maps = [], {}, {}

    def robot_cfg(rname):
        if rname not in robots_cfg:
            robot = RB.robot(rname); extra = {}
            apply_set(a, robot, S, extra)
            robots_cfg[rname] = (robot, extra)
        return robots_cfg[rname]

    def add_jobs(rname, robot, trajs, spawn, mi, rel, flo_spec, speed0=0.0, scen='', Sj=None):
        drives = csv_list(a.drive) or [robot['drive'][0]]
        for drv in drives:
            if drv not in robot['drive']:
                raise SystemExit(f'{rname}: drive "{drv}" not possible (supported: {", ".join(robot["drive"])}; skid-steer robots cannot move sideways)')
        for tr, drv, ctl, mot in itertools.product(trajs, drives, csv_list(a.control), csv_list(a.motor_control)):
            headings = csv_list(a.heading) if drv == 'holonomic' else ['tangent']
            locs = csv_list(a.localization) if ctl == 'closed' else ['truth']
            for hd, loc in itertools.product(headings, locs):
                v = a.speed or speed0 or robot['speed']; acc = a.accel or robot['accel']
                if tr == 'replay':
                    T = float(pd.read_csv(a.replay).iloc[-1, 0]); ctl = 'replay'; loc = 'truth'
                else:
                    L = float(TJ.build(tr)[-1, 0])
                    Ta = v / acc; T = (L / v + Ta) if L > v * Ta else 2 * math.sqrt(L / acc)
                j = dict(robot=rname, traj=tr, drive=drv, heading=hd, control=ctl, loc=loc, motor=mot, map=mi, spawn=spawn, speed=v, accel=acc, T=T,
                         relief=rel, floor=flo_spec, scenario=scen, on_tip=a.on_tip, tip_deg=a.tip_deg, body=a.body_contact)
                if Sj is not None: j['S'] = Sj
                if tr == 'replay':
                    v0 = [float(x) for x in a.v0.split(',')] + [0, 0, 0] if a.v0 else [0, 0, 0]
                    j['replay'] = dict(file=os.path.abspath(a.replay), mode=a.replay_mode, truth=os.path.abspath(a.truth) if a.truth else '',
                                       v0=v0[:3], t_start=0.0 if any(v0[:3]) else a.t_start)
                jobs.append(j)

    warned = set()
    if a.map:                                # ---- ready map files (configs/maps)
        if a.relief != 'flat': print('NOTE: --map given: --relief is ignored (the relief comes from the map; change it with --map-set)')
        map_set = list(a.map_set)
        if a.floor:                          # --floor X with a map: the whole floor becomes X (zones removed)
            fk, _, fr = a.floor.partition(':')
            fs = fk if not fr else (gen_maps._opts(fr).get('surface') if fk == 'uniform' else None)
            if not fs or '|' in a.floor:
                raise SystemExit(f'--floor {a.floor} with --map: only one surface can replace the floor of a map (--floor table); '
                                 'for a pattern edit the zones of the map file or use --map-set (docs/en/MAPS.md)')
            map_set += ['zones=[]', f'default_surface={fs}']
            print(f'NOTE: --floor {fs}: the whole floor of the map is {fs} (its zones are dropped)')
        elif a.surface:                      # --surface X with a map: default surface X, zones kept
            map_set += [f'default_surface={a.surface}']
            print(f'NOTE: --surface {a.surface}: default surface of the map is {a.surface} (its zones stay; --floor {a.surface} replaces everything)')
        lib = S
        for spec in csv_list(a.map):
            M, scen = MP.load(spec, map_set)
            Sm = MP.resolve_surfaces(M, lib)
            for nm in Sm:
                if nm not in S: S[nm] = Sm[nm]
                elif Sm[nm] != S[nm] and nm in MP.custom_surfaces(M): raise SystemExit(f'surface {nm} is defined differently by two maps')
            key = (M['file'], tuple(map_set), a.res)
            if key not in maps:
                t0 = time.time()
                mi = MP.build(M, os.path.join(a.out, 'maps', M['name']), res=a.res or None, meshes=a.engine != 'emulate')
                mi['tag'] = M['name']; mi['M'] = M; maps[key] = mi
                for s_ in mi['surfaces']:
                    if s_ not in S: raise SystemExit(f'map {M["name"]}: surface "{s_}" not in the library ({", ".join(S)}); define it in the map (surfaces:) or --surface-file')
                print(f"map {M['name']}: {M['bounds'][1] - M['bounds'][0]:g} x {M['bounds'][3] - M['bounds'][2]:g} m @ {mi['res']} m, surfaces {mi['surfaces']}, "
                      f"h {mi['H'].min() * 1000:.0f}..{mi['H'].max() * 1000:.0f} mm ({time.time() - t0:.1f} s)  [{M['file']}]")
            mi = maps[key]
            scens = list(M['scenarios']) if scen == '*' else [scen]
            for sn in scens:
                sc = MP.scenario(M, sn)
                rnames = csv_list(a.robot) or sc['robots'] or M['robots'] or ['robotino']
                for rname in rnames:
                    robot, extra = robot_cfg(rname)
                    trajs = ['replay'] if a.replay else ([t for t in a.traj.split('|')] if a.traj else ([sc['traj']] if sc['traj'] else [DEFAULT_TRAJ.get(rname, 'circle:R=1')]))
                    spawn = spawn_cli or sc['spawn']
                    if (M['name'], rname) not in warned:
                        warned.add((M['name'], rname))
                        rows, routes = MP.check(M, mi, [rname]); r0 = rows[0]
                        if r0['verdict'] != 'ok':
                            print(f"{'WARNING' if r0['verdict'] == 'invalid' else 'note'} {M['name']} / {rname}: single-point wheel contact error up to "
                                  f"{r0['err_max_mm']} mm = {r0['err_over_r'] * 100:.0f} % of r ({r0['wheel_r_mm']} mm) -> {r0['verdict']} (docs/en/MAPS.md)")
                    for tr in trajs:
                        if tr == 'replay': continue
                        P = MP.route_world(dict(spawn=spawn, traj=tr))
                        xmin, xmax, ymin, ymax = M['bounds']
                        if len(P) and ((P[:, 0] < xmin).any() or (P[:, 0] > xmax).any() or (P[:, 1] < ymin).any() or (P[:, 1] > ymax).any()):
                            print(f"WARNING {M['name']}: trajectory {tr} from {spawn} leaves the map bounds")
                    add_jobs(rname, robot, trajs, spawn, mi, '', '', sc['speed'], f"{M['name']}@{sn}" if sn else M['name'])
    for rname in ([] if a.map else (csv_list(a.robot) or ['robotino'])):
        robot, extra = robot_cfg(rname)
        spawn = spawn_cli or (0.0, 0.0, 0.0)
        trajs = ['replay'] if a.replay else ([t for t in a.traj.split('|')] if a.traj else [DEFAULT_TRAJ.get(rname, 'circle:R=1')])
        floors = [f for f in a.floor.split('|')] if a.floor else ['']
        reliefs = [r for r in a.relief.split('|')]
        for rel, flo in itertools.product(reliefs, floors):
            if flo and ':' not in flo and flo in S: flo = f'uniform:surface={flo}'      # --floor table = uniform table floor
            flo_spec = flo or f'uniform:surface={a.surface or "gray"}'
            fo = gen_maps._opts(flo_spec.partition(':')[2])
            for s in fo.get('surfaces', [fo.get('surface', a.surface or 'gray')]):
                if s not in S: raise SystemExit(f'surface "{s}" not in the library ({", ".join(S)}); add it with --surface-file')
            # map size: robot default, enlarged so that every trajectory fits
            half = a.map_size or robot['map_size']
            for tr in trajs:
                if tr == 'replay':
                    A = replay_path_array(a.truth) if a.truth else np.zeros((2, 5))
                else: A = TJ.build(tr)
                x, y = A[:, 1], A[:, 2]
                c, s_ = math.cos(math.radians(spawn[2])), math.sin(math.radians(spawn[2]))
                ext = max(np.abs(spawn[0] + c * x - s_ * y).max(), np.abs(spawn[1] + s_ * x + c * y).max()) + 1.0
                if not a.map_size: half = max(half, math.ceil(ext))
                elif ext > a.map_size: print(f'WARNING: trajectory {tr} leaves the map (needs half-size {ext:.1f} m)')
            res = a.res or robot['map_res']
            key = (rel, flo_spec, half, res)
            if key not in maps:
                tag = f'm{len(maps)}'
                t0 = time.time()
                mi = gen_maps.build_world(rel if rel != 'flat' else '', flo_spec, half, res, os.path.join(a.out, 'maps', tag), default_surface=a.surface or 'gray',
                                          mesh_step=max(1, int(round(0.04 / res))) if a.engine != 'emulate' else 8)
                mi['tag'] = tag; maps[key] = mi
                print(f"map {tag}: {mi['label']} ({rel}), {2 * half:.0f} x {2 * half:.0f} m @ {res} m, surfaces {mi['surfaces']}, "
                      f"h {mi['H'].min() * 1000:.0f}..{mi['H'].max() * 1000:.0f} mm  ({time.time() - t0:.1f} s)")
            add_jobs(rname, robot, trajs, spawn, maps[key], rel, flo_spec)
    nmaps = len(maps)
    for j in jobs:
        j['map_tag'] = (j['scenario'].replace('@', '-') if j.get('scenario') else (j['map']['tag'] if nmaps > 1 else ''))
        j['name'] = job_name(j)
    names = [j['name'] for j in jobs]
    for j in jobs:
        if names.count(j['name']) > 1:
            j['name'] += '_' + str(abs(hash(j['traj'])) % 1000)
        j['dir'] = os.path.join(a.out, j['name'])
    print(f"{len(jobs)} run(s), engine {a.engine}, output {a.out}")
    results, per_surface = [], []

    def go(args):
        k, j = args
        robot, extra = robots_cfg[j['robot']]
        print(f"-> [{k + 1}/{len(jobs)}] {j['name']}  ({j['T']:.0f} s of motion)", flush=True)
        r, ps = run_job(j, a, copy.deepcopy(robot), S, OC, extra, k % max(a.jobs, 1))
        msg = f"   {j['name']}: {r['status']}, wall {r['wall_s']} s, RTF {r['rtf']}"
        if 'xtrack_rms_m' in r:
            msg += (f" | dev RMS {r['xtrack_rms_m'] * 100:.1f} cm max {r['xtrack_max_m'] * 100:.1f} cm, final {r['final_err_m'] * 100:.1f} cm"
                    f" | odom drift {r['odom_drift_m'] * 100:.1f} cm, +IMU {r['fused_drift_m'] * 100:.1f} cm | I {r['I_mean_A']} A, slip {r['slip_mean']}")
        if r.get('error'): msg += ' | ' + r['error']
        print(msg, flush=True)
        return r, ps

    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        for r, ps in ex.map(go, enumerate(jobs)):
            results.append(r); per_surface += ps or []
    R = pd.DataFrame(results); R.to_csv(os.path.join(a.out, 'summary.csv'), index=False)
    if per_surface: pd.DataFrame(per_surface).to_csv(os.path.join(a.out, 'per_surface.csv'), index=False)
    cols = [c for c in ('run', 'status', 'rtf', 'progress', 'climb_m', 'terrain_slope_max_deg', 'xtrack_rms_m', 'xtrack_max_m', 'final_err_m', 'yaw_err_rms_deg', 'odom_drift_m', 'fused_drift_m',
                        'I_mean_A', 'slip_mean', 'slip_lat_mean', 'sink_max_mm', 'energy_J_per_m') if c in R]
    with open(os.path.join(a.out, 'summary.md'), 'w') as f:
        f.write(f'# drive runs ({a.engine})\n\n' + R[cols].to_markdown(index=False) + '\n')
        if per_surface: f.write('\n## per surface (wheel samples while moving)\n\n' + pd.DataFrame(per_surface).to_markdown(index=False) + '\n')
    if not a.no_figures:                     # matplotlib is not thread-safe: figures in the main thread
        for j, r in zip(jobs, results):
            if os.path.exists(os.path.join(j['dir'], '.log.pkl')):
                try: figure(j, j['dir'], r)
                except Exception as e: print(f"figure {j['name']}: {e!r}")
        try: batch_figure(a.out, R)
        except Exception as e: print('summary figure:', e)
    for j in jobs:
        p = os.path.join(j['dir'], '.log.pkl')
        if os.path.exists(p): os.remove(p)
    print(f'\nsummary: {os.path.join(a.out, "summary.md")}')
    bad = R[~R.status.isin(['done', 'tipped', 'left_map'])]
    lm = R[R.status == 'left_map']
    if len(lm): print('left the map (slid or drove off; the ground beyond the bounds is not modelled):', ', '.join(lm.run))
    tip = R[R.status == 'tipped']
    if len(tip): print('tipped over (chassis tilted > %g deg to the ground; --on-tip %s):' % (a.tip_deg, a.on_tip), ', '.join(tip.run))
    if len(bad): print('NOT OK:', ', '.join(f'{r.run} ({r.status})' for r in bad.itertuples())); sys.exit(1)


if __name__ == '__main__':
    main()
