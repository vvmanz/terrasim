"""Checks of the body-ground contact (emulator, about 20 s):
  - on flat ground the body never touches it, and --body-contact on / off give the same run;
  - a Jackal on the 55 deg `rollover` map with --on-tip continue rolls over and comes to rest on its roof,
    carrying its weight on the body points;
  - with --on-tip stop the same run ends shortly after tipping over;
  - a Husky pushed against the foot of the 55 deg slope rests its bumper on it (body contact, no tipping).
    python3 tools/test_body.py
"""
import os, sys, json, subprocess, tempfile, glob
import pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robots as RB

fails = []


def check(ok, what):
    print(('[ OK ] ' if ok else '[FAIL] ') + what)
    if not ok: fails.append(what)


def run(out, *args):
    r = subprocess.run(['bash', os.path.join(ROOT, 'scripts', 'drive.sh'), '--engine', 'emulate', '--no-figures', '--out', out, *args],
                       capture_output=True, text=True)
    if r.returncode != 0: print(r.stdout[-2000:], r.stderr[-2000:])
    runs = {}
    for d in glob.glob(os.path.join(out, '*', 'log.csv')):
        d = os.path.dirname(d)
        runs[os.path.basename(d)] = (pd.read_csv(d + '/log.csv'), json.load(open(d + '/metrics.json')))
    return runs


T = tempfile.mkdtemp(prefix='terrasim_test_body_')
on = run(T + '/flat_on', '--robot', 'husky', '--traj', 'square:L=2,rc=0.4')
off = run(T + '/flat_off', '--robot', 'husky', '--traj', 'square:L=2,rc=0.4', '--body-contact', 'off')
(L1, M1), = on.values(); (L2, M2), = off.values()
check(L1.body_N.max() == 0 and L1.body_pts.max() == 0, 'flat ground: the body never touches it')
check(all(M1[k] == M2[k] for k in ('xtrack_rms_m', 'energy_J', 'slip_mean', 'odom_drift_m')), 'flat ground: --body-contact on / off give the same run')

roll = run(T + '/roll', '--robot', 'jackal', '--map', 'rollover@across', '--on-tip', 'continue')
(L, M), = roll.values()
w = RB.robot('jackal')['mass'] * 9.81
check(L.tilt_rel.max() > 150 and M.get('tipped_t') is not None, f'rollover: the Jackal rolls over (max tilt {L.tilt_rel.max():.0f} deg)')
end = L[L.t > L.t.iloc[-1] - 1.0]
check(end.tilt_rel.min() > 150, f'rollover: it rests upside down (final tilt {L.tilt_rel.iloc[-1]:.0f} deg)')
check(abs(end.body_N.mean() - w) < 0.15 * w, f'rollover: the body carries the weight ({end.body_N.mean():.0f} N of {w:.0f} N)')
check((end[['vx', 'vy', 'wz']].abs().max() < 0.01).all(), 'rollover: it comes to rest (no creep on the ground)')

stop = run(T + '/stop', '--robot', 'jackal', '--map', 'rollover@across')
(Ls, Ms), = stop.values()
check(Ls.t.iloc[-1] < Ms['tipped_t'] + 0.5, f"--on-tip stop: the run ends at {Ls.t.iloc[-1]:.2f} s, tipped at {Ms['tipped_t']} s")

wall = run(T + '/wall', '--robot', 'husky', '--map', 'rollover@up')
(Lw, Mw), = wall.values()
check(Lw.body_N.max() > 50 and Lw.tilt_rel.max() < 50, f'husky against the 55 deg slope: bumper on the slope ({Lw.body_N.max():.0f} N), not tipped')

print(f'test_body: {"all checks passed" if not fails else f"{len(fails)} FAILED"} ({T})')
sys.exit(1 if fails else 0)
