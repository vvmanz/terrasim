"""Traversability study: every robot x every surface x slope, same task for all — drive a straight line with the
path controller (closed loop on the true pose), and see who gets through.

    python3 tools/traverse.py                                   # all robots, all library surfaces, slopes 0/10/20/30 deg
    python3 tools/traverse.py --robots burger,waffle --angles 0,5,10,15,20,25 --jobs 4
    python3 tools/traverse.py --engine harmonic --robots burger --surfaces gray,sand_lete    # same in Gazebo

Each run: straight line on a uniform floor; slope runs: a ramp that starts just in front of the robot (x0 = front
wheel + 0.25 m, so the robot is spawned on level ground) and rises along the path; the line is x0 + --length m long,
i.e. --length m of it are uphill. Outputs (--out, default runs/traverse_<date>):
  runs.csv                one row per run: progress (position along the path at the end / path length, within the
                          planned time + 2 s; v7.1 and older: distance driven / path length, which counted sliding back),
                          driven-wheel contact slip, motor current / stall current, sinkage of wheels and casters,
                          energy per metre, chassis clearance, verdict
  matrix_flat.png/.csv    verdict and progress per robot x surface on the flat floor
  max_slope.png/.csv      steepest slope climbed (progress >= 0.9) per robot x surface
  REPORT.md               the tables and the reasons (traction-limited, torque-limited, sinkage, belly)
Verdicts: ok (progress >= 0.95), slow (0.5 <= progress < 0.95), stuck (progress < 0.5). The time budget is the
planned motion time (trapezoidal speed profile) + 2 s, so 'slow' means: could not keep the commanded speed.
"""
import os, sys, json, math, argparse, subprocess, datetime, glob
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robots as RB

SURFACES = ['table', 'table_wet', 'red', 'gray', 'green', 'carpet', 'carpet_shag', 'foam', 'sand_lete', 'sandy_loam', 'clay_wet', 'mars_soil']
LABEL = {'robotino': 'Robotino', 'youbot': 'youBot', 'husky': 'Husky', 'rover': 'MarsSim rover', 'burger': 'TB3 Burger',
         'waffle': 'TB3 Waffle Pi', 'jackal': 'Jackal', 'pioneer': 'Pioneer 3-DX'}


def clearance(name):
    """lowest point of the chassis visuals (casters excluded) above the floor, m"""
    import robot_preview as P
    Rb = RB.robot(name)
    if name == 'rover': return float('nan')            # procedural rover model: not measured
    pts = [P.item_tris(it)[0].reshape(-1, 3) for it in RB.visuals(Rb)['base']]
    if not pts: return float('nan')
    X = np.concatenate(pts); m = np.ones(len(X), bool)
    for w in Rb['wheels']:
        if w['type'] == 'caster': m &= np.hypot(X[:, 0] - w['x'], X[:, 1] - w['y']) > 0.03
    return float(np.percentile(X[m, 2], 0.5))


def stall_current(Rb):
    m = Rb['motor']; return m['Vmax'] / m['R']


def ramp_start(name):
    return round(max(w['x'] for w in RB.robot(name)['wheels']) + 0.25, 3)


def run_robot(name, a, out):
    x0 = ramp_start(name); L = round(x0 + a.length, 3)
    rel = ['flat'] + [f'ramp:angle={g:g},x0={x0},x1={L + 1.0}' for g in a.angles if g > 0]
    flo = [f'uniform:surface={s}' for s in a.surfaces]
    d = os.path.join(out, name)
    # through scripts/drive.sh: it sources ROS / Gazebo and sets the plugin path for Gazebo Harmonic
    cmd = ['bash', os.path.join(ROOT, 'scripts', 'drive.sh'), '--robot', name, '--engine', a.engine, '--traj', f'line:L={L}',
           '--relief', '|'.join(rel), '--floor', '|'.join(flo), '--no-figures', '--no-full', '--jobs', str(a.jobs), '--out', d,
           '--map-size', str(max(RB.robot(name)['map_size'], L + 2.5))]
    for kv in a.set: cmd += ['--set', kv]
    log = open(os.path.join(out, f'{name}.log'), 'w')
    subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
    S = pd.read_csv(os.path.join(d, 'summary.csv'))
    return S


NUM = ['driven_m', 'path_len_m', 'I_mean_A', 'slip_mean', 'sink_max_mm', 'sink_caster_max_mm', 'energy_J_per_m', 'tilt_max_deg', 'final_err_m']


def enrich(S, name):
    Rb = RB.robot(name)
    S = S.copy()
    for c in NUM:                                         # failed runs have no metrics
        if c not in S: S[c] = np.nan
    S['surface'] = S['floor'].str.replace('uniform:surface=', '', regex=False)
    S['angle'] = S['relief'].apply(lambda s: 0.0 if s == 'flat' else float(s.split('angle=')[1].split(',')[0]))
    if 'progress' not in S or S['progress'].isna().all():          # older summaries: distance driven / path
        S['progress'] = (S['driven_m'] / S['path_len_m']).clip(upper=1.2)
    S['progress'] = S['progress'].clip(upper=1.05)                  # v7.2: position along the path at the end (sliding back counts)
    S['I_over_stall'] = S['I_mean_A'] / stall_current(Rb)
    S['clearance_mm'] = clearance(name) * 1000
    v = []
    for _, row in S.iterrows():
        if row.status in ('FAILED', 'stalled'): v.append('failed')
        elif row.status == 'tipped': v.append('tipped')
        else: v.append('stuck' if row.progress < 0.5 or row.status in ('diverged', 'left_map') else ('ok' if row.progress >= 0.95 else 'slow'))
    S['verdict'] = v
    return S


def reason(row):
    if row.verdict == 'ok': return ''
    if row.verdict == 'failed': return 'the simulation did not run (see the console.log of the run)'
    if row.verdict == 'tipped': return 'tipped over (chassis > 50 deg to the ground)'
    if row.status == 'left_map': return 'slid back off the map'
    if row.I_over_stall > 0.6: return 'torque-limited (motor near stall)'
    if row.get('sink_caster_max_mm', 0) and row.sink_caster_max_mm > 0.5 * row.clearance_mm: return 'caster sinks'
    return 'traction-limited (wheels slip)'


def heat(T, title, out, fmt='{:.0f}', cmap='RdYlGn', vmin=None, vmax=None, text=None):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(1.0 + 0.85 * T.shape[1], 0.9 + 0.48 * T.shape[0]))
    im = ax.imshow(T.values.astype(float), cmap=cmap, vmin=vmin, vmax=vmax, aspect='auto')
    ax.set_xticks(range(T.shape[1])); ax.set_xticklabels(T.columns, rotation=40, ha='right', fontsize=9)
    ax.set_yticks(range(T.shape[0])); ax.set_yticklabels([LABEL.get(i, i) for i in T.index], fontsize=9)
    for i in range(T.shape[0]):
        for j in range(T.shape[1]):
            val = T.values[i, j]
            s = (text.values[i, j] if text is not None else ('' if pd.isna(val) else fmt.format(val)))
            ax.text(j, i, s, ha='center', va='center', fontsize=8)
    ax.set_title(title, fontsize=10); fig.colorbar(im, ax=ax, fraction=0.03); fig.tight_layout(); fig.savefig(out, dpi=120); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', default=','.join(RB.ROBOTS)); ap.add_argument('--surfaces', default=','.join(SURFACES))
    ap.add_argument('--angles', default='0,10,20,30'); ap.add_argument('--length', type=float, default=2.0, help='uphill part of the line, m')
    ap.add_argument('--set', action='append', default=[], help='passed to drive_run (e.g. caster.zmax_r=1.0)')
    ap.add_argument('--engine', default='emulate'); ap.add_argument('--jobs', type=int, default=4); ap.add_argument('--out', default='')
    ap.add_argument('--merge', action='store_true', help='--out is an existing study: rerun only --robots, replace their rows, rebuild the tables')
    a = ap.parse_args()
    a.surfaces = [s for s in a.surfaces.split(',') if s]; a.angles = [float(x) for x in a.angles.split(',')]
    out = a.out or os.path.join(ROOT, 'runs', 'traverse_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    allS = []
    for name in [s for s in a.robots.split(',') if s]:
        print(f'[traverse] {name}: {len(a.surfaces)} surfaces x {len(a.angles)} slopes', flush=True)
        S = enrich(run_robot(name, a, out), name); allS.append(S)
        bad = S[S.status.isin(['FAILED', 'stalled'])]
        if len(bad):
            err = str(bad['error'].iloc[0]) if 'error' in bad and bad['error'].notna().any() else ''
            print(f'[traverse] {name}: {len(bad)}/{len(S)} runs failed; first error: {err[:300]}\n'
                  f'           console logs: {os.path.join(out, name)}/<run>/console.log', flush=True)
        print(S.groupby('angle').verdict.value_counts().unstack(fill_value=0).to_string(), flush=True)
    R = pd.concat(allS, ignore_index=True)
    R['reason'] = R.apply(reason, axis=1)
    keep = ['robot', 'surface', 'angle', 'verdict', 'reason', 'progress', 'slip_mean', 'I_mean_A', 'I_over_stall', 'sink_max_mm',
            'sink_caster_max_mm', 'clearance_mm', 'energy_J_per_m', 'tilt_max_deg', 'final_err_m', 'status', 'run']
    R = R[[c for c in keep if c in R]]
    if a.merge and os.path.exists(os.path.join(out, 'runs.csv')):
        # an existing study: the rerun robots replace their rows, the others stay; order of robots and surfaces kept
        old = pd.read_csv(os.path.join(out, 'runs.csv'))
        order = list(dict.fromkeys(old.robot)) + [r for r in dict.fromkeys(R.robot) if r not in set(old.robot)]
        R = pd.concat([old[~old.robot.isin(set(R.robot))], R], ignore_index=True)
        R['_o'] = R.robot.map({r: i for i, r in enumerate(order)}); R = R.sort_values(['_o'], kind='stable').drop(columns='_o').reset_index(drop=True)
        a.surfaces = list(dict.fromkeys(list(old.surface) + a.surfaces)); a.angles = sorted(set(old.angle) | set(a.angles))
    R.to_csv(os.path.join(out, 'runs.csv'), index=False, float_format='%.4g')
    robots = list(dict.fromkeys(R.robot)); surfs = [s for s in a.surfaces if s in set(R.surface)]
    F = R[R.angle == 0]
    sr = F.pivot_table(index='robot', columns='surface', values='progress').reindex(index=robots, columns=surfs)
    vd = F.pivot_table(index='robot', columns='surface', values='verdict', aggfunc='first').reindex(index=robots, columns=surfs)
    txt = vd.fillna('') + '\n' + sr.map(lambda v: '' if pd.isna(v) else f'{v:.2f}')
    sr.to_csv(os.path.join(out, 'matrix_flat_progress.csv')); vd.to_csv(os.path.join(out, 'matrix_flat_verdict.csv'))
    heat(sr.clip(upper=1.0), 'flat floor: progress in the planned time (verdict)', os.path.join(out, 'matrix_flat.png'), vmin=0, vmax=1, text=txt)
    ms = None
    if any(g > 0 for g in a.angles):
        def mx(g):
            okv = g[(g.progress >= 0.9)].angle
            return okv.max() if len(okv) else 0.0
        ms = R.groupby(['robot', 'surface']).apply(mx, include_groups=False).unstack().reindex(index=robots, columns=surfs)
        ms.to_csv(os.path.join(out, 'max_slope.csv'))
        heat(ms, f'steepest slope climbed, deg (tested {", ".join(f"{g:g}" for g in a.angles)})', os.path.join(out, 'max_slope.png'),
             vmin=0, vmax=max(a.angles))
    with open(os.path.join(out, 'REPORT.md'), 'w') as f:
        f.write(f'# Traversability, engine {a.engine}, line {a.length} m, closed loop on the true pose\n\n')
        f.write('## Flat floor: verdict / progress (position along the path at the end / path length, in the planned time + 2 s)\n\n')
        try: f.write(txt.map(lambda s: s.replace('\n', ' ')).rename(index=LABEL).to_markdown()); f.write('\n\n')
        except ImportError: f.write(txt.to_string() + '\n\n')
        if ms is not None:
            f.write('## Steepest slope climbed (progress >= 90 %), deg\n\n')
            try: f.write(ms.rename(index=LABEL).to_markdown()); f.write('\n\n')
            except ImportError: f.write(ms.to_string() + '\n\n')
        bad = R[R.verdict != 'ok'].sort_values(['robot', 'surface', 'angle'])
        f.write('## Not ok: why\n\n')
        for _, r in bad.iterrows():
            f.write(f"- {LABEL.get(r.robot, r.robot)}, {r.surface}, {r.angle:g} deg: {r.verdict}, progress {r.progress:.2f}, "
                    f"slip {r.slip_mean:.2f}, I/I_stall {r.I_over_stall:.2f}, sinkage {r.sink_max_mm:.1f} mm -> {r.reason}\n")
    print(f'[traverse] -> {out}')


if __name__ == '__main__':
    main()
