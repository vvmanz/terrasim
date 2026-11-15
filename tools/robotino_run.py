"""Robotino "star" experiments on any set of surfaces -> datasets in the format of the Robotino lab recordings
(dataset_belyaev.csv, 41 columns; the column list is in docs/en/DATASETS.md).

Normally started through scripts/robotino.sh (sets up the Gazebo/ROS environment) or
ros2/robotino.launch.py (ROS 2). Examples:

  scripts/robotino.sh --engine harmonic --surfaces real                      # 4 calibrated floors, all speeds
  scripts/robotino.sh --engine harmonic --surfaces synthetic --jobs 4        # synthetic floors, 4 in parallel
  scripts/robotino.sh --engine harmonic --surfaces carpet --speeds 0.2 --gui --rtf 5   # watch it 5x faster
  scripts/robotino.sh --engine emulate  --surfaces all                       # no Gazebo (3D emulator, ~1000x)
  scripts/robotino.sh --engine harmonic --surfaces gray --dirs 0:90:15 --t-run 8
  scripts/robotino.sh --engine emulate --surfaces carpet --sweep carpet:mu=0.6,0.8,1.0
  scripts/robotino.sh --engine emulate --surface-file my_floors.json --surfaces my_floor
  scripts/robotino.sh --engine emulate --surfaces carpet --set carpet.f_lat=0.4 --set carpet.K=0.008
  scripts/robotino.sh --list                                                 # surfaces and their parameters

Engines: harmonic (Gazebo Harmonic, gz-sim 8, DART), emulate (the same plugin logic in the offline C++ core,
robotino3d, no Gazebo), surrogate (planar robotino2d, used for calibration). All three run the same OmniLogic code.

Output (--out, default datasets/<date>_<engine>):
  csv/<surface>_<speed>_clean.csv   written right after each run: exact dataset_belyaev.csv columns,
  csv/<surface>_<speed>_noisy.csv   clean = true simulated signals, noisy = through the bench measurement model
  csv/<surface>_<speed>_ext_clean.csv, _ext_noisy.csv  the 90 columns of belyaev_kushnarev.csv (encoder counts,
                                    current/slip-speed projections, the bench Kalman filter and its torque estimate;
                                    docs/en/DATASETS.md); dataset_ext_clean.csv / dataset_ext_noisy.csv all runs. --no-ext
  csv/<surface>_<speed>_full.csv.gz full per-run log, 50 Hz: true pose/quaternion/velocities, IMU (noisy + true),
                                    ideal / encoder / encoder+gyro odometry, energy and every wheel quantity
                                    (angle, speeds, current, voltage, torques, forces, slips, sinkage, mu, K, ...);
                                    columns in docs/en/OUTPUTS.md (full log). --full-rate, --no-full
  dataset_clean.csv, dataset_noisy.csv   all runs together (like the original file)
  figures/  slip vs motor current scatter plots + linear fits (clean and noisy), Ke vs direction
  summary.csv / summary.md   per surface and speed: speed ratio, currents, slips, Ke, sinkage, RTF achieved
  runs/<surface>_<speed>/   experiment.ini, world, raw logs (log.csv, log_detail.csv), console.log
                            (the raw full log log_full.csv is moved to csv/*_full.csv.gz)
  params_used.json          the robot/surface parameters of this run
"""
import os, sys, json, copy, time, argparse, subprocess, signal, re, datetime
import numpy as np, pandas as pd
from concurrent.futures import ThreadPoolExecutor
import threading
STOP = threading.Event()        # Ctrl+C: running worlds are stopped, finished runs are still written and combined
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robotino_model as M
import robotino_dataset as DS
import robotino_ext as EXT
import gen_maps

EXE = {'emulate': os.path.join(ROOT, 'core', 'build', 'robotino3d'), 'surrogate': os.path.join(ROOT, 'core', 'build', 'robotino2d')}
TH = np.radians([60.0, 180.0, 300.0])


# ------------------------------------------------------------------------------------------- params
def parse_value(v):
    try: return float(v)
    except ValueError: return v


def build_params(a):
    P = M.load_params()
    S = P['surfaces']; SP = P['experiment']['speeds']
    if a.surface_file:
        X = json.load(open(a.surface_file))
        surf = X.get('surfaces', X)
        for name, par in surf.items():
            if name.startswith('_'): continue
            base = copy.deepcopy(S[par['base']]) if 'base' in par else {}
            base.update({k: v for k, v in par.items() if k != 'base'})
            base.setdefault('_kind', 'user'); S[name] = base
            SP.setdefault(name, X.get('speeds', {}).get(name, [0.1, 0.2, 0.3]))
    for kv in a.set:
        key, v = kv.split('=', 1)
        name, par = key.split('.', 1)
        if name == 'robot': P['robot'][par] = parse_value(v)
        elif name == 'motor': P['motor'][par] = parse_value(v)
        else: S[name][par] = parse_value(v)
    for sw in a.sweep:              # carpet:mu=0.6,0.8,1.0
        name, rest = sw.split(':', 1); par, vals = rest.split('=', 1)
        for v in vals.split(','):
            new = f'{name}_{par}{v}'
            S[new] = copy.deepcopy(S[name]); S[new][par] = parse_value(v)
            S[new]['_kind'] = 'sweep'; S[new]['_doc'] = f'{name} with {par} = {v}'
            SP[new] = SP.get(name, [0.1, 0.2, 0.3])
    return P


def select(P, spec, sweeps):
    S = P['surfaces']
    out = []
    for tok in spec.split(','):
        tok = tok.strip()
        if tok == 'all': out += [k for k in S if S[k].get('_kind') != 'sweep']
        elif tok == 'real': out += [k for k in S if S[k].get('_kind') == 'calibrated']
        elif tok in ('synthetic', 'extreme', 'user', 'hard'): out += [k for k in S if S[k].get('_kind') == tok]
        elif tok in S: out.append(tok)
        elif tok: raise SystemExit(f'unknown surface or group "{tok}"; known: {", ".join(S)}')
    for sw in sweeps:
        name, rest = sw.split(':', 1); par, vals = rest.split('=', 1)
        out += [f'{name}_{par}{v}' for v in vals.split(',')]
    seen = set(); return [x for x in out if not (x in seen or seen.add(x))]


def parse_dirs(s):
    if not s: return None
    if ':' in s:
        a, b, c = (float(x) for x in s.split(':'))
        return [round(v, 6) for v in np.arange(a, b + 1e-9, c)]
    return [float(x) for x in s.split(',')]


def list_surfaces(P):
    S = P['surfaces']
    keys = ['mu', 'c', 'K', 'f_roll', 'f_lat', 'kphi', 'kc', 'n', 'layer', 'rebound', 'slip_norm', 'cv_res', 'cv_grip']
    rows = []
    for name, s in S.items():
        r = dict(surface=name, kind=s.get('_kind', ''))
        for k in keys: r[k] = s.get(k, '')
        rows.append(r)
    print(pd.DataFrame(rows).to_string(index=False))
    print('\nDescriptions:')
    for name, s in S.items(): print(f'  {name:12s} {s.get("_doc", "")}')


# ------------------------------------------------------------------------------------------- one job
def make_world(flavor, d, P, surface, ini, dt, rtf, mapinfo=None):
    import gz_sdf as GE
    if mapinfo:
        floor = [GE.mesh_visual_model(f'map_{s}', obj, GE.SURF_COLOR.get(s, '0.5 0.5 0.5 1')) for obj, s in mapinfo['meshes']]
        floor.append(GE.static_box('floor_outside', (0, 0, -0.006), (80, 80, 0.01), '0.4 0.4 0.4 1', collide=False))
    else:
        floor = [GE.static_box('floor_visual', (0, 0, -0.005), (80, 80, 0.01), GE.SURF_COLOR.get(surface, '0.5 0.5 0.5 1'), collide=False)]
    models = floor + [GE.static_box('safety_floor', (0, 0, -0.35), (80, 80, 0.1), '0.3 0.3 0.3 1'), GE.robotino_model_sdf(flavor, P, ini)]
    wf = os.path.join(d, 'world.sdf')
    open(wf, 'w').write(GE.world(flavor, f'robotino_{surface}', models, dt=dt, rtf=rtf))
    errs = GE.validate_world(wf)
    if errs: raise RuntimeError('; '.join(errs))
    return wf


def kill_group(p):
    if p is None or p.poll() is not None: return
    try:
        os.killpg(p.pid, signal.SIGTERM); time.sleep(2)
        if p.poll() is None: os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError: pass


def run_job(job, a, P, slot):
    surf, sp, label = job['surface'], job['speed'], job['label']
    d = job['dir']; os.makedirs(d, exist_ok=True)
    log = os.path.join(d, 'log.csv'); det = os.path.join(d, 'log_detail.csv'); done = log + '.done'
    full = os.path.join(d, 'log_full.csv')
    for f in (log, det, done, full):
        if os.path.exists(f): os.remove(f)
    gazebo = a.engine == 'harmonic'
    ini = os.path.join(d, 'experiment.ini')
    mapinfo = job.get('map')
    ret = a.ret or bool(mapinfo)
    open(ini, 'w').write(M.ini_text(P, surf, sp, log, t_run=job['t_run'], t_pause=a.t_pause, directions=job['dirs'], links=gazebo,
                                    label=label, map_file=mapinfo['dtm'] if mapinfo else None, return_to_start=ret,
                                    full_rate=0 if a.no_full else a.full_rate))
    t_ret = (3.0 * job['t_run'] + 3.0) if ret else 0.0               # = OmniLogic t_return_max (upper bound)
    t_end = P['experiment']['t_start'] + len(job['dirs']) * (job['t_run'] + a.t_pause + t_ret) + 0.05
    env = dict(os.environ, TM_LOG=log, TM_DONE=done)
    procs = []
    con = open(os.path.join(d, 'console.log'), 'w')
    t0 = time.time()
    if gazebo:
        world = make_world(a.engine, d, P, surf, ini, a.dt, a.rtf, mapinfo)
        iters = int(t_end / a.dt) + 500
        env['GZ_PARTITION'] = f'terrasim_{os.getpid()}_{slot}'
        cmd = ['gz', 'sim', '-r', '-v', '3', world] if a.gui else ['gz', 'sim', '-s', '-r', '-v', '3', '--iterations', str(iters), world]
        procs.append(subprocess.Popen(cmd, stdout=con, stderr=subprocess.STDOUT, env=env, start_new_session=True))
    else:
        procs.append(subprocess.Popen([EXE[a.engine], ini, '--dt', str(a.dt)], stdout=con, stderr=subprocess.STDOUT, env=env,
                                      start_new_session=True))
    status, last_size, last_change, last_print = 'finished', -1, time.time(), time.time()
    stall = 1e9 if a.gui else a.stall
    while procs[0].poll() is None:
        time.sleep(1.0 if gazebo else 0.1)
        now = time.time()
        if os.path.exists(done): status = 'done'; time.sleep(1.0); break
        if STOP.is_set(): status = 'interrupted'; break
        size = os.path.getsize(det) if os.path.exists(det) else 0
        if size != last_size: last_size, last_change = size, now
        if now - last_change > stall: status = 'stalled'; break
        if now - last_print >= a.progress:
            last_print = now
            ts = sim_time(det)
            print(f'   [{label} {sp}] sim {ts:7.1f} / {t_end:.0f} s, wall {now - t0:5.0f} s, RTF {ts / max(now - t0, 1e-9):5.1f}', flush=True)
    for p in procs[::-1]: kill_group(p)
    con.close()
    wall = time.time() - t0
    if status == 'done': t_end = sim_time(det)
    res = dict(surface=label, speed=sp, status=status, wall_s=round(wall, 1), sim_s=round(t_end, 1), rtf=round(t_end / max(wall, 1e-9), 1))
    if status == 'interrupted':
        res['status'] = 'FAILED'; res['error'] = '   interrupted (Ctrl+C), series not written'; return res
    if not os.path.exists(log) or os.path.getsize(log) < 1000:
        res['status'] = 'FAILED'; res['error'] = tail(os.path.join(d, 'console.log')); return res
    raw = pd.read_csv(log, on_bad_lines='skip').dropna(subset=['t', 'w3linslip'])
    c = DS.clean(raw, strict=a.strict_format); n = DS.noisy(raw, seed=a.seed, strict=a.strict_format)
    os.makedirs(os.path.join(a.out, 'csv'), exist_ok=True)
    stem = os.path.join(a.out, 'csv', f'{label}_{sp}')
    if a.noise in ('clean', 'both'): DS.write(c, stem + '_clean.csv')
    if a.noise in ('noisy', 'both'): DS.write(n, stem + '_noisy.csv')
    if not a.no_ext:
        ec = EXT.extend(c)
        if a.noise in ('clean', 'both'): EXT.write(ec, stem + '_ext_clean.csv')
        if a.noise in ('noisy', 'both'): EXT.write(EXT.extend(n, pos=EXT.pos_like(ec, n)), stem + '_ext_noisy.csv')
    if not a.no_full and os.path.exists(full):
        export_full(full, stem + '_full.csv.gz', label, sp); os.remove(full)
    res.update(run_summary(c, det, P['surfaces'][surf]))
    if mapinfo: res['kind'] = 'map'
    return res


CONST_KEYS = ['mu', 'c', 'K', 'patch_len', 'f_roll', 'f_lat', 'mu_rb', 'kn', 'cn', 'n', 'kc', 'kphi', 'b_contact', 'layer', 'rebound',
              'slip_norm', 'cv_res', 'cv_grip', 'cv_soil', 'corr_len', 'field_seed']
VARIES = {'cv_res': ['f_roll', 'f_lat', 'mu_rb'], 'cv_grip': ['K'], 'cv_soil': ['kc', 'kphi']}


def surfaces_constant(P, jobs, out):
    """surfaces_constant.csv: one row per surface of the dataset with every parameter of the contact model (base
    values, before the heterogeneity field) and which parameters vary from row to row (then their values per
    wheel and per sample are in the w{i}_* columns of the datasets: base value x (1 + cv x field))"""
    names = []
    for j in jobs:
        for s in [j['surface']] + list((j.get('map') or {}).get('surfaces', [])):
            if s not in names and s in P['surfaces']: names.append(s)
    rows = []
    dflt = dict(c=0.0, n=1.0, kc=0.0, kphi=0.0, b_contact=0.03, layer='none', rebound=0.0, cv_soil=0.0, corr_len=0.02, field_seed=7,
                slip_norm=P.get('omni_contact', {}).get('slip_norm', 1), patch_len=0.01, mu_rb=0.0)    # core defaults (omni.hpp FirmSurface)
    for s in names:
        q = P['surfaces'][s]; r = dict(surface=s, kind=q.get('_kind', ''))
        for k in CONST_KEYS: r[k] = q.get(k, dflt.get(k, ''))
        soil = float(q.get('kc', 0) or 0) > 0 or float(q.get('kphi', 0) or 0) > 0
        var = [p for cv, ps in VARIES.items() if float(q.get(cv, 0) or 0) > 0 and (cv != 'cv_soil' or soil) for p in ps]
        r['varies_per_row'] = ' '.join(var); r['constant'] = ' '.join(k for k in ('mu', 'c', 'K', 'f_roll', 'f_lat', 'mu_rb', 'kc', 'kphi', 'n') if k not in var)
        r['description'] = q.get('_doc', '')
        rows.append(r)
    pd.DataFrame(rows).to_csv(os.path.join(out, 'surfaces_constant.csv'), index=False)


def labels_order(jobs):
    out = []
    for j in jobs:
        if j['label'] not in out: out.append(j['label'])
    return out


def export_full(src, dst, label, speed):
    """raw log_full.csv (plugin / emulator) -> csv/<label>_<speed>_full.csv.gz; a truncated last line is dropped"""
    F = pd.read_csv(src, on_bad_lines='skip')
    F = F.dropna(subset=[F.columns[0], F.columns[-1]])
    F = pd.concat([F.iloc[:, :1], pd.DataFrame({'label': label, 'speedamp': speed}, index=F.index), F.iloc[:, 1:]], axis=1)
    F.to_csv(dst, index=False, float_format='%.7g', compression='gzip')
    return len(F)


def sim_time(det):
    try:
        with open(det, 'rb') as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 4096)); lines = f.read().decode(errors='ignore').strip().split('\n')
        return float(lines[-1].split(',')[0])
    except Exception:
        return 0.0


def tail(path, n=12):
    try:
        L = open(path, errors='ignore').read().splitlines()
        L = [l for l in L if re.search(r'rr|error|terramech|omni', l, re.I)] or L
        return '\n'.join(L[-n:])
    except Exception:
        return ''


def run_summary(c, det, S):
    R = M.run_level(c)
    out = dict(kind=S.get('_kind', ''), runs=int(len(R)), v_ratio=round(float(R.ratio.mean()), 3), yaw_dps=round(float(R.yawrate.abs().mean()), 2))
    if len(R):
        a = np.radians(R.dir.values)
        sl = np.concatenate([R[f's{i}'].values[np.abs(np.sin(a - TH[i - 1])) > 0.6] for i in (1, 2, 3)])
        cur = np.concatenate([R[f'I{i}'].values[np.abs(np.sin(a - TH[i - 1])) > 0.6] for i in (1, 2, 3)])
        out.update(slip_driven=round(float(np.nanmean(sl)), 3), I_driven=round(float(np.nanmean(cur)), 3))
    out['Ke_mean'] = round(float(c.Ke[c.t > 1500].mean()), 2)
    try:
        D = pd.read_csv(det, usecols=lambda k: k in ('phase', 'z1', 'z2', 'z3', 'Rc1', 'Rc2', 'Rc3'))
        D = D[D.phase == 1]
        out['sinkage_mm'] = round(float(D[['z1', 'z2', 'z3']].mean().mean() * 1000), 2)
        out['Rcomp_N'] = round(float(D[['Rc1', 'Rc2', 'Rc3']].mean().mean()), 2)
    except Exception:
        pass
    return out


# ------------------------------------------------------------------------------------------- figures
def map_figure(job, out_png):
    """map (heights, surfaces) with the trajectories; current and slip along the first ray"""
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    mi = job['map']
    with open(mi['dtm']) as f: toks = f.read().split()
    nx, ny, x0, y0, res, nt = int(toks[2]), int(toks[3]), float(toks[4]), float(toks[5]), float(toks[6]), int(toks[7])
    p = 8 + nt * 10
    H = np.array(toks[p:p + nx * ny], float).reshape(ny, nx); T = np.array(toks[p + nx * ny:p + 2 * nx * ny], int).reshape(ny, nx)
    D = pd.read_csv(os.path.join(job['dir'], 'log_detail.csv'))
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    ext = [x0, x0 + (nx - 1) * res, y0, y0 + (ny - 1) * res]
    ax[0].imshow(T, origin='lower', extent=ext, cmap='Pastel1', vmin=0, vmax=8, alpha=.9)
    if H.max() > 0: ax[0].contour(np.linspace(ext[0], ext[1], nx), np.linspace(ext[2], ext[3], ny), H * 1000, levels=6, colors='k', linewidths=.4)
    for r, g in D[D.phase == 1].groupby('run'): ax[0].plot(g.x, g.y, '-', lw=1)
    lim = max(0.5, np.abs(D[['x', 'y']].values).max() * 1.1); ax[0].set_xlim(-lim, lim); ax[0].set_ylim(-lim, lim); ax[0].set_aspect('equal')
    ax[0].set_title(f"{mi['label']}: " + ', '.join(f'{k}={s}' for k, s in enumerate(mi['surfaces'])) + ' (contours: height, mm)')
    g = D[(D.phase == 1) & (D.run == D[D.phase == 1].run.min())]
    tt = g.t - g.t.iloc[0]
    for i, c in zip((1, 2, 3), ('r', 'g', 'b')):
        ax[1].plot(tt, g[f'I{i}'].abs(), c, lw=1, label=f'I{i}'); ax[2].plot(tt, g[f's{i}'], c, lw=1, label=f's{i} (contact)')
    ax2 = ax[1].twinx(); ax2.plot(tt, np.hypot(g.x, g.y), 'k--', lw=1); ax2.set_ylabel('distance from the centre, m')
    ax[1].set_title(f'ray {g.dir.iloc[0]:.0f}°: currents'); ax[2].set_title('slip in the contact patch, per wheel')
    for a_ in ax[1:]: a_.set_xlabel('t, s'); a_.grid(alpha=.3); a_.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(out_png, dpi=80); plt.close(fig)


def ke_figure(a, surfaces):
    """energy coefficient Ke vs direction of motion (run means after 1.5 s), per surface and speed"""
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    import robotino_figures as RR
    rows = []
    for kind in ('clean', 'noisy'):
        f = os.path.join(a.out, f'dataset_{kind}.csv')
        if not os.path.exists(f): continue
        d = pd.read_csv(f, usecols=['t', 'surf', 'movedir', 'speedamp', 'Ke'])
        d = d[d.t > 1500]
        g = d.groupby(['surf', 'speedamp', 'movedir']).Ke.mean().reset_index(); g['kind'] = kind; rows.append(g)
    if not rows: return
    G = pd.concat(rows); G.to_csv(os.path.join(a.out, 'figures', 'Ke_vs_direction.csv'), index=False)
    speeds = sorted(G.speedamp.unique())
    fig, ax = plt.subplots(1, len(speeds), figsize=(6 * len(speeds), 4.8), squeeze=False)
    for j, sp in enumerate(speeds):
        for s in surfaces:
            for kind, ls in (('noisy', '-'), ('clean', ':')):
                h = G[(G.surf == s) & np.isclose(G.speedamp, sp) & (G.kind == kind)].sort_values('movedir')
                if not h.empty: ax[0, j].plot(h.movedir, h.Ke, ls, color=RR.surf_color(s), lw=1.4, label=RR.surf_name(s) if kind == 'noisy' else None)
        ax[0, j].set_title(f'Ke(α), {sp} m/s (solid: with the measurement model, dotted: clean)'); ax[0, j].set_xlabel('direction of motion α, °')
        ax[0, j].set_xticks(range(0, 361, 60)); ax[0, j].grid(alpha=.3)
    ax[0, 0].set_ylabel('Ke = Σ|U I| / Σ|U I_n|'); ax[0, 0].legend(fontsize=7)
    plt.tight_layout(); plt.savefig(os.path.join(a.out, 'figures', 'Ke_vs_direction.png'), dpi=80); plt.close(fig)


def figures(a, surfaces, P):
    """slip-vs-current figures (tools/robotino_figures.py) and Ke vs direction for the clean / noisy datasets"""
    import robotino_figures as RF
    fd = os.path.join(a.out, 'figures'); os.makedirs(fd, exist_ok=True)
    for kind in ('clean', 'noisy'):
        f = os.path.join(a.out, f'dataset_{kind}.csv')
        if not os.path.exists(f): continue
        W = RF.windows(pd.read_csv(f))
        W.to_csv(os.path.join(fd, f'windows_0p5s_{kind}.csv.gz'), index=False)
        RF.all_figures(W, fd, tag=kind, surfaces=surfaces)
    ke_figure(a, surfaces)


# ------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--engine', choices=['harmonic', 'emulate', 'surrogate'], default='harmonic')
    ap.add_argument('--surfaces', default='real', help='names and/or groups real, synthetic, hard, extreme, user, all (comma separated)')
    ap.add_argument('--speeds', default='', help='e.g. 0.1,0.3 (default: per surface from the config)')
    ap.add_argument('--dirs', default='', help='start:stop:step in degrees (default 0:355:5) or a list 0,90,180')
    ap.add_argument('--t-run', type=float, default=None, help='duration of one direction, s (default 18/10/6 s for 0.1/0.2/0.3 m/s)')
    ap.add_argument('--t-pause', type=float, default=1.5)
    ap.add_argument('--quick', action='store_true', help='12 directions, 6 s each (pipeline check)')
    ap.add_argument('--gui', action='store_true', help='show Gazebo Harmonic; default RTF 1 then')
    ap.add_argument('--rtf', type=float, default=None, help='target real-time factor (0 = as fast as possible; default 0 headless, 1 with --gui)')
    ap.add_argument('--jobs', type=int, default=1, help='surfaces/speeds simulated in parallel')
    ap.add_argument('--dt', type=float, default=0.001, help='physics step, s')
    ap.add_argument('--noise', choices=['clean', 'noisy', 'both'], default='both')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--surface-file', default='', help='JSON with additional surfaces {"name": {"base": "carpet", "mu": 0.8, ...}}')
    ap.add_argument('--set', action='append', default=[], help='override: surface.param=value, robot.param=value, motor.param=value')
    ap.add_argument('--sweep', action='append', default=[], help='surface:param=v1,v2,... -> surfaces surface_param<v>')
    ap.add_argument('--out', default='')
    ap.add_argument('--stall', type=float, default=180, help='stop a run if the log does not grow for this many seconds')
    ap.add_argument('--progress', type=float, default=20)
    ap.add_argument('--no-figures', action='store_true')
    ap.add_argument('--full-rate', type=float, default=50.0, help='rate of the full per-run log csv/*_full.csv.gz, Hz')
    ap.add_argument('--no-full', action='store_true', help='do not write the full per-run log')
    ap.add_argument('--no-ext', action='store_true', help='do not write the extended (belyaev_kushnarev.csv format) datasets')
    ap.add_argument('--strict-format', action='store_true', help='datasets with exactly the columns of the real files: without the surface '
                    'under each wheel (w{i}_x, w{i}_y, w{i}_mu, w{i}_K, w{i}_f_roll, w{i}_f_lat, w{i}_mu_rb, w{i}_kc, w{i}_kphi)')
    ap.add_argument('--map', default='', help='terrain map: hills:A=0.01,L=0.4 | rings:surfaces=gray+carpet,w=0.5 | '
                    'sectors:surfaces=a+b+c | patches:surfaces=a+b,cell=0.4 | file:/path/map.dtm | a map file NAME (configs/maps, '
                    'tools/maps.py list; the star starts at the map origin) (see tools/gen_maps.py)')
    ap.add_argument('--return', dest='ret', action='store_true', help='drive back to the star centre after every ray (on by default with --map)')
    ap.add_argument('--list', action='store_true', help='print the surface library and exit')
    a = ap.parse_args()
    P = build_params(a)
    if a.list: list_surfaces(P); return
    if a.rtf is None: a.rtf = 1.0 if a.gui else 0.0
    if a.gui and a.jobs > 1: print('[robotino] --gui: running one world at a time'); a.jobs = 1
    if a.map and a.engine == 'surrogate': raise SystemExit('--map needs a 3D engine: harmonic or emulate')
    if a.engine in ('emulate', 'surrogate') and not os.path.exists(EXE[a.engine]):
        raise SystemExit(f'{EXE[a.engine]} not built: bash scripts/build.sh core')
    surfaces = select(P, a.surfaces, a.sweep)
    dirs = parse_dirs(a.dirs) or [float(x) for x in range(0, 360, 5)]
    if a.quick: dirs = [float(x) for x in range(0, 360, 30)]
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    a.out = os.path.abspath(a.out or os.path.join(ROOT, 'datasets', f'{stamp}_{a.engine}'))
    os.makedirs(os.path.join(a.out, 'runs'), exist_ok=True)
    json.dump(P, open(os.path.join(a.out, 'params_used.json'), 'w'), indent=1, ensure_ascii=False)
    jobs = []
    groups = []                     # (label, surface for [experiment], map info)
    if a.map:
        kind, opts = gen_maps.parse_spec(a.map)
        if kind == 'hills':
            for s in surfaces:
                mi = gen_maps.build(kind, opts, os.path.join(a.out, 'maps', f'{s}_hills'), surface=s)
                groups.append((mi['label'], s, mi))
        else:
            mi = gen_maps.build(kind, opts, os.path.join(a.out, 'maps', gen_maps.map_label(kind, opts)))
            for s in mi['surfaces']:
                if s not in P['surfaces']: raise SystemExit(f'map surface "{s}" is not in the library (--list)')
            groups.append((mi['label'], mi['surfaces'][0], mi)); surfaces = mi['surfaces']
    else:
        groups = [(s, s, None) for s in surfaces]
    for label, s, mi in groups:
        speeds = [float(x) for x in a.speeds.split(',')] if a.speeds else P['experiment']['speeds'].get(s, [0.1, 0.2, 0.3])
        for sp in speeds:
            t_run = 6.0 if a.quick else (a.t_run if a.t_run else P['experiment']['t_run'].get(str(sp), 18.0 * 0.1 / sp))
            jobs.append(dict(surface=s, label=label, map=mi, speed=sp, t_run=t_run, dirs=dirs, dir=os.path.join(a.out, 'runs', f'{label}_{sp}')))
    total = sum(P['experiment']['t_start'] + len(j['dirs']) * (j['t_run'] + a.t_pause) for j in jobs)
    print(f'[robotino] engine {a.engine}, {len(jobs)} runs ({", ".join(sorted(set(j["label"] for j in jobs)))}), {len(dirs)} directions each, '
          f'{total:.0f} s of simulated time, RTF target {"max" if not a.rtf else a.rtf}, jobs {a.jobs}\n[robotino] output: {a.out}', flush=True)
    results = []
    ex = ThreadPoolExecutor(max(1, a.jobs))
    futs = [ex.submit(run_job, j, a, P, k % max(1, a.jobs)) for k, j in enumerate(jobs)]
    try:
        for f in futs:
            r = f.result(); results.append(r)
            msg = f"   -> {r['surface']} {r['speed']} m/s: {r['status']}, wall {r['wall_s']} s (RTF {r['rtf']})"
            if 'v_ratio' in r: msg += f", |v|/v_cmd {r['v_ratio']}, slip {r.get('slip_driven')}, I {r.get('I_driven')} A, Ke {r['Ke_mean']}"
            if r['status'] == 'FAILED': msg += '\n' + r.get('error', '')
            print(msg, flush=True)
    except KeyboardInterrupt:
        print('\n[robotino] Ctrl+C: stopping the worlds; finished series are written and merged', flush=True)
        STOP.set()
        for f in futs: f.cancel()
        results = [f.result() for f in futs if f.done() and not f.cancelled()]
    ex.shutdown(wait=True)
    if STOP.is_set():
        left = [f"{j['label']} {j['speed']}" for j in jobs if not any(r['surface'] == j['label'] and r['speed'] == j['speed'] and r['status'] != 'FAILED' for r in results)]
        print(f"[robotino] not finished: {', '.join(left)}. Run them into another folder and merge: scripts/robotino_missing.sh or tools/robotino_merge.py", flush=True)
    ok = [r for r in results if r['status'] != 'FAILED']
    for kind in ('clean', 'noisy'):
        files = [os.path.join(a.out, 'csv', f"{r['surface']}_{r['speed']}_{kind}.csv") for r in ok]
        files = [f for f in files if os.path.exists(f)]
        if files:
            pd.concat([pd.read_csv(f) for f in files], ignore_index=True).to_csv(os.path.join(a.out, f'dataset_{kind}.csv'), index=False, float_format='%.7g')
    if not a.no_ext:
        for kind in ('clean', 'noisy'):
            order = sorted(ok, key=lambda r: (labels_order(jobs).index(r['surface']) if r['surface'] in labels_order(jobs) else 99, r['speed']))
            files = [os.path.join(a.out, 'csv', f"{r['surface']}_{r['speed']}_ext_{kind}.csv") for r in order]
            parts = [pd.read_csv(f) for f in files if os.path.exists(f)]
            if parts: EXT.write(EXT.renumber(parts), os.path.join(a.out, f'dataset_ext_{kind}.csv'))
    surfaces_constant(P, jobs, a.out)
    T = pd.DataFrame(results)
    T.to_csv(os.path.join(a.out, 'summary.csv'), index=False)
    with open(os.path.join(a.out, 'summary.md'), 'w') as f:
        f.write(f'# Robotino, engine {a.engine}, {len(dirs)} directions\n\n')
        cols = [c for c in ('surface', 'kind', 'speed', 'status', 'runs', 'v_ratio', 'slip_driven', 'I_driven', 'Ke_mean', 'yaw_dps', 'sinkage_mm', 'Rcomp_N', 'wall_s', 'rtf') if c in T]
        try: f.write(T[cols].to_markdown(index=False))
        except ImportError: f.write(T[cols].to_string(index=False))
        f.write('\n')
    if ok and not a.no_figures:
        labels = []
        for j in jobs:
            if j['label'] not in labels and any(r['surface'] == j['label'] for r in ok): labels.append(j['label'])
        figures(a, labels, P)
        done_maps = set()
        for j in jobs:
            if j.get('map') and j['label'] not in done_maps and os.path.exists(os.path.join(j['dir'], 'log_detail.csv')):
                done_maps.add(j['label']); map_figure(j, os.path.join(a.out, 'figures', f"map_{j['label']}_{j['speed']}.png"))
    print(f'[robotino] done: {len(ok)}/{len(results)} runs OK -> {a.out}')
    if len(ok) < len(results): sys.exit(1)


if __name__ == '__main__':
    main()
