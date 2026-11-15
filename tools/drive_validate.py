"""Self-consistency check of the two Robotino code paths (no recorded data needed). The vehicle mode: Robotino driving open-loop straight lines sideways (heading fixed,
line:angle=a) must reproduce the calibrated star experiment (omni_logic) on the same surface and speed.

    python3 tools/drive_validate.py [--surface gray] [--speed 0.2] [--dirs 0,30,60,90] [--out runs/drive_validate]
"""
import os, sys, argparse, subprocess, tempfile
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robotino_model as M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--surface', default='gray'); ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--dirs', default='0,30,60,90'); ap.add_argument('--out', default=os.path.join(ROOT, 'runs', 'drive_validate'))
    a = ap.parse_args()
    dirs = [float(d) for d in a.dirs.split(',')]
    os.makedirs(a.out, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix='drive_validate_')
    star = os.path.join(tmp, 'star')
    subprocess.run([sys.executable, os.path.join(HERE, 'robotino_run.py'), '--engine', 'emulate', '--surfaces', a.surface, '--speeds', str(a.speed),
                    '--dirs', a.dirs, '--noise', 'clean', '--out', star, '--no-figures'], check=True, capture_output=True)
    L = M.run_level(pd.read_csv(os.path.join(star, 'csv', f'{a.surface}_{a.speed}_clean.csv')))
    rows = []
    for k, d in enumerate(dirs):
        o = os.path.join(tmp, f'veh_{int(d)}')
        T = 10.0
        subprocess.run([sys.executable, os.path.join(HERE, 'drive_run.py'), '--robot', 'robotino', '--traj', f'line:L={a.speed * T},angle={d}',
                        '--heading', 'fixed', '--control', 'open', '--speed', str(a.speed), '--accel', '20', '--surface', a.surface,
                        '--imu-noise', 'none', '--no-figures', '--out', o], check=True, capture_output=True)
        run = [x for x in os.listdir(o) if x.startswith('robotino')][0]
        D = pd.read_csv(os.path.join(o, run, 'log.csv'))
        m = D[D.phase == 1]; m = m[m.t > m.t.iloc[0] + 1.5]
        s = L.iloc[k]
        rows.append(dict(dir=d, ratio_vehicle=np.hypot(m.vx, m.vy).mean() / a.speed, ratio_star=s.ratio,
                         **{f'I{i}_vehicle': m[f'w{i}_cur'].mean() for i in (1, 2, 3)}, **{f'I{i}_star': s[f'I{i}'] for i in (1, 2, 3)}))
    R = pd.DataFrame(rows).round(3)
    R.to_csv(os.path.join(a.out, f'vehicle_vs_star_{a.surface}_{a.speed}.csv'), index=False)
    dr = (R.ratio_vehicle / R.ratio_star - 1).abs().max() * 100
    dI = max((R[f'I{i}_vehicle'] - R[f'I{i}_star']).abs().max() for i in (1, 2, 3))
    txt = (f'# vehicle mode vs calibrated star ({a.surface}, {a.speed} m/s)\n\n' + R.to_markdown(index=False) +
           f'\n\nmax |ratio difference| {dr:.1f} %, max |current difference| {dI:.3f} A\n')
    open(os.path.join(a.out, f'vehicle_vs_star_{a.surface}_{a.speed}.md'), 'w').write(txt)
    print(txt)


if __name__ == '__main__':
    main()
