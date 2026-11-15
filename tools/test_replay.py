"""tests of the command replay (drive_run --replay, VehicleLogic [controller] type = replay):  python3 tools/test_replay.py"""
import os, sys, subprocess, tempfile, math, glob
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__))
ok = 0


def check(c, msg):
    global ok
    if not c: raise SystemExit('FAIL: ' + msg)
    ok += 1


def run(args):
    d = tempfile.mkdtemp()
    r = subprocess.run([sys.executable, os.path.join(HERE, 'drive_run.py'), '--engine', 'emulate', '--no-figures', '--no-full', '--out', d] + args,
                       capture_output=True, text=True)
    check(r.returncode == 0, 'drive_run failed: ' + r.stdout[-500:] + r.stderr[-500:])
    L = pd.read_csv(glob.glob(os.path.join(d, '*', 'log.csv'))[0])
    return L


tmp = tempfile.mkdtemp()
# 1) twist replay, youBot on the table: 0.2 m/s along x for 3 s, from rest
f = os.path.join(tmp, 'tw.csv'); pd.DataFrame(dict(t=[0, 3, 3.001], vx=[0.2, 0.2, 0], vy=[0, 0, 0], wz=[0, 0, 0])).to_csv(f, index=False)
L = run(['--robot', 'youbot', '--floor', 'uniform:surface=table', '--map-size', '2', '--replay', f, '--replay-mode', 'twist'])
on = L[L.phase >= 1]; dx = on.x.iloc[-1] - on.x.iloc[0]; dy = on.y.iloc[-1] - on.y.iloc[0]
check(0.45 < dx < 0.62, f'youBot twist replay: forward {dx:.3f} m (0.6 m commanded, some slip)')
check(abs(dy) < 0.05, f'youBot twist replay: sideways {dy:.3f} m')
check(abs(on.cmd_vx.iloc[len(on) // 2] - 0.2) < 1e-6, 'logged command = replayed twist')
# 2) twist replay, sideways
f = os.path.join(tmp, 'tw2.csv'); pd.DataFrame(dict(t=[0, 3, 3.001], vx=[0, 0, 0], vy=[0.2, 0.2, 0], wz=[0, 0, 0])).to_csv(f, index=False)
L = run(['--robot', 'youbot', '--floor', 'uniform:surface=table', '--map-size', '2', '--replay', f, '--replay-mode', 'twist'])
on = L[L.phase >= 1]; dy = on.y.iloc[-1] - on.y.iloc[0]
check(0.35 < dy < 0.62, f'youBot sideways replay: {dy:.3f} m')
# 3) wheel replay with initial velocity (recording starts while driving), Husky: equal wheel speeds -> straight line
f = os.path.join(tmp, 'wh.csv'); w = 3.0; pd.DataFrame(dict(t=[0, 3], g0=[w, w], g1=[w, w])).to_csv(f, index=False)
v0 = w * 0.1651
L = run(['--robot', 'husky', '--floor', 'uniform:surface=gray', '--replay', f, '--replay-mode', 'wheels', f'--v0={v0},0,0'])
check(L.phase.iloc[0] == 1, 'with --v0 the motion starts at t = 0')
check(abs(math.hypot(L.vx.iloc[2], L.vy.iloc[2]) - v0) < 0.08, f'initial speed {math.hypot(L.vx.iloc[2], L.vy.iloc[2]):.3f} vs {v0:.3f}')
d = math.hypot(L.x[L.t <= 3.0].iloc[-1] - L.x.iloc[0], L.y[L.t <= 3.0].iloc[-1] - L.y.iloc[0])
check(abs(d - 3 * v0) / (3 * v0) < 0.08, f'Husky wheel replay: {d:.3f} m in 3 s vs {3 * v0:.3f} (r*w*T)')
check(abs(L.yaw.iloc[-1] - L.yaw.iloc[0]) < math.radians(2), 'equal wheel speeds keep the heading')
# 4) wheel replay, differential: right faster -> turns left, by less than the geometric track predicts (skid steer)
f = os.path.join(tmp, 'wh2.csv'); pd.DataFrame(dict(t=[0, 4], g0=[2.0, 2.0], g1=[4.0, 4.0])).to_csv(f, index=False)
L = run(['--robot', 'husky', '--floor', 'uniform:surface=gray', '--replay', f, '--replay-mode', 'wheels'])
on = L[L.phase == 1]; wz = np.median(on.wz.iloc[len(on) // 3:])
wz_kin = 0.1651 * (4.0 - 2.0) / 0.5708
check(0.2 * wz_kin < wz < 0.95 * wz_kin, f'skid steer: yaw rate {wz:.3f} rad/s below the geometric {wz_kin:.3f}')
# 5) mecanum on a soft floor (gray rubber): straight the rollers stand still -> almost no
#    slip; sideways they roll -> slower and more current (v7.4: the rollers used to spin in every direction)
spd, cur = {}, {}
for name, vx, vy in (('fwd', 0.2, 0.0), ('side', 0.0, 0.2)):
    f = os.path.join(tmp, f'g_{name}.csv'); pd.DataFrame(dict(t=[0, 4, 4.001], vx=[vx, vx, 0], vy=[vy, vy, 0], wz=[0, 0, 0])).to_csv(f, index=False)
    L = run(['--robot', 'youbot', '--floor', 'uniform:surface=gray', '--map-size', '2', '--replay', f, '--replay-mode', 'twist', '--t-settle', '1.0'])
    on = L[L.phase == 1]; on = on[on.t - on.t.iloc[0] > 1.0]
    spd[name] = math.hypot(np.polyfit(on.t, on.x, 1)[0], np.polyfit(on.t, on.y, 1)[0]) / 0.2
    cur[name] = sum(on[f'w{i}_cur'].mean() for i in range(1, 5))
check(spd['fwd'] > 0.95, f"youBot straight on gray: |v|/v = {spd['fwd']:.3f} (rollers still, almost no slip)")
check(spd['side'] < spd['fwd'] - 0.15, f"youBot sideways on gray slower: {spd['side']:.3f} vs straight {spd['fwd']:.3f}")
check(cur['side'] > 2.0 * cur['fwd'], f"youBot sideways draws more current: {cur['side']:.2f} A vs {cur['fwd']:.2f} A")
print(f'test_replay: {ok} checks passed')
