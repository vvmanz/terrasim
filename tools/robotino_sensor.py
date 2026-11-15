"""Measurement model of the Robotino test bench, applied to simulated logs so that simulation and reality
go through the same measurement chain (scripts/robotino.sh writes dataset_noisy.csv with it).

What the recorded Robotino logs show (tools/robotino_sensor.py --dataset ... prints these statistics):
  * the logger runs at ~32 Hz (dt 31.3 ms, jitter ~1 ms), the ceiling camera at ~29.4 Hz;
    the logger takes the latest camera frame, so ~8 % of the logged frames repeat the previous camera
    frame: position unchanged, vx = vy = omega = 0 exactly, wheel slip = 1 for that frame. They come
    every ~13 frames (beat 32 - 29.4 Hz) and are compensated on average (the next frames move by a
    camera period while the logger divides by its own period), i.e. run means are unbiased, but
    per-frame and 0.5 s-window values are not: |w_enc - w_eff| gets a floor that grows with speed;
  * velocities are finite differences of camera positions -> white position/yaw noise;
  * the board current is quantised (~1/39 A) and noisy;
  * the motor speed is an encoder count difference: steps of 2.8274 rad/s at the motor
    (0.177 rad/s at the wheel); a wheel that does not turn reads exactly 0 and its slip is logged as 0.
Parameters (SENSOR) are identified from the frame-level statistics of the dataset (see __main__).
"""
import numpy as np, pandas as pd

THETA = np.radians([60.0, 180.0, 300.0])
R_KIN, R_W = 0.125, 0.04
SENSOR = dict(f_cam=29.4, dt_log=0.03129, dt_jitter=0.0010, pos_sigma=0.00025, yaw_sigma=0.0050,
              cur_sigma=0.10, cur_step=1.0 / 39.0, enc_step=2.8274, gear=16.0)


def _runs(d):
    return (d['t'].diff() < 0).cumsum().values


def apply(d, seed=1, S=SENSOR):
    """d: dataset-format frame (simulation, true signals, any log rate). Returns a new dataset-format
    frame resampled at the logger times with the camera/beat/noise model applied."""
    rng = np.random.default_rng(seed)
    run = _runs(d)
    out = []
    for r in np.unique(run):
        g = d[run == r]
        t = g['t'].values / 1000.0
        if len(t) < 4: continue
        T0, T1 = t[0], t[-1]
        # logger and camera clocks
        n_log = int((T1 - T0) / S['dt_log']) + 2
        tl = T0 + np.cumsum(np.r_[0.0, np.clip(rng.normal(S['dt_log'], S['dt_jitter'], n_log - 1), 0.5 * S['dt_log'], 2 * S['dt_log'])])
        tl = tl[tl <= T1]
        Tc = 1.0 / S['f_cam']
        phase = rng.uniform(0, Tc)
        tc = np.arange(T0 - Tc + phase, T1 + Tc, Tc)
        x = np.interp(tc, t, g['xpos'].values); y = np.interp(tc, t, g['ypos'].values)
        yaw_true = np.unwrap(np.radians(g['ang'].values))
        yw = np.interp(tc, t, yaw_true)
        x = x + rng.normal(0, S['pos_sigma'], len(tc)); y = y + rng.normal(0, S['pos_sigma'], len(tc))
        yw = yw + rng.normal(0, S['yaw_sigma'], len(tc))
        j = np.searchsorted(tc, tl, side='right') - 1          # latest camera frame at each logger tick
        X, Y, PS = x[j], y[j], yw[j]
        dtl = np.diff(tl)
        dX, dY, dP = np.diff(X) / dtl, np.diff(Y) / dtl, np.diff(PS) / dtl
        pm = 0.5 * (PS[1:] + PS[:-1])
        vx = np.r_[0.0, np.cos(pm) * dX + np.sin(pm) * dY]
        vy = np.r_[0.0, -np.sin(pm) * dX + np.cos(pm) * dY]
        om = np.r_[0.0, dP]
        h = pd.DataFrame({'t': (tl - T0 + t[0]) * 1000.0})
        for c in g.columns:
            if c in ('t',): continue
            if g[c].dtype.kind in 'fi':
                h[c] = np.interp(tl, t, g[c].values)
            else:
                h[c] = g[c].iloc[0]
        h['xpos'], h['ypos'], h['ang'] = X, Y, np.degrees(PS) % 360.0
        h['vx'], h['vy'], h['omega'] = vx, vy, om
        # encoders: counts of the motor angle between logger ticks (dithered quantisation, unbiased on average)
        dlt = S['enc_step'] * S['dt_log']
        for i in (1, 2, 3):
            wm = g[f'w{i}vel'].values * S['gear']
            ang = np.r_[0.0, np.cumsum(0.5 * (wm[1:] + wm[:-1]) * np.diff(t))] + rng.uniform(0, dlt)
            cnt = np.floor(np.interp(tl, t, ang) / dlt)
            wq = np.r_[0.0, np.diff(cnt)] * S['enc_step']
            h[f'm{i}vel'] = wq; h[f'w{i}vel'] = wq / S['gear']
        alpha = np.radians(h['movedir'].values)
        for i in (1, 2, 3):
            th = THETA[i - 1]
            we = (-np.sin(th) * vx + np.cos(th) * vy + R_KIN * om) / R_W
            wa = h[f'w{i}vel'].values
            ok = np.abs(wa) > 1e-6                              # dataset: slip = 0 when the encoder reads 0
            s = np.where(ok, 1 - we / np.where(ok, wa, 1), 0.0)
            h[f'w{i}effvel'] = we; h[f'w{i}slip'] = s; h[f'w{i}linslip'] = np.abs(np.sin(alpha - th)) * s
            c = h[f'm{i}cur'].values + rng.normal(0, S['cur_sigma'], len(h)) * (np.abs(h[f'm{i}setvel'].values) > 1e-6)
            h[f'm{i}cur'] = np.abs(np.round(c / S['cur_step']) * S['cur_step'])
        h = h.iloc[1:]                                          # first tick has no velocity
        out.append(h)
    return pd.concat(out, ignore_index=True) if out else d.iloc[:0].copy()


def frame_stats(d, t_skip_ms=1500.0):
    """frame-level statistics used to identify SENSOR (same for real and simulated logs)"""
    d = d.copy(); d['run'] = _runs(d); d = d[d['t'] > t_skip_ms]
    rows = []
    for (s, sp), g in d.groupby(['surf', 'speedamp']):
        z = ((g.vx == 0) & (g.vy == 0)).mean()
        res = []
        for i in (1, 2, 3):
            rr = g.groupby('run')[f'w{i}effvel'].transform(lambda x: x - x.rolling(15, center=True, min_periods=5).mean())
            res.append(rr.dropna().values)
        om = g.groupby('run')['omega'].transform(lambda x: x - x.rolling(15, center=True, min_periods=5).mean()).dropna()
        cur = g.groupby('run')['m1cur'].transform(lambda x: x - x.rolling(15, center=True, min_periods=5).mean()).dropna()
        rows.append(dict(surf=s, sp=sp, zero_frac=z, weff_res_std=np.concatenate(res).std(), omega_res_std=om.std(),
                         cur_res_std=cur.std(), dt_ms=g.groupby('run').t.diff().median()))
    return pd.DataFrame(rows)


if __name__ == '__main__':
    import argparse, sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description='frame-level statistics of a dataset-format CSV (your own recording), '
                                 'optionally next to simulated clean datasets passed through the measurement model')
    ap.add_argument('--dataset', required=True, help='dataset-format CSV (columns of dataset_clean.csv)')
    ap.add_argument('--sim', nargs='*', default=[], help='clean simulated datasets (e.g. .../dataset_clean.csv)')
    a = ap.parse_args()
    R = frame_stats(pd.read_csv(a.dataset)); R['src'] = 'dataset'
    frames = [R]
    if a.sim:
        Sm = frame_stats(apply(pd.concat([pd.read_csv(f) for f in a.sim], ignore_index=True))); Sm['src'] = 'sim+sensor'; frames.append(Sm)
    print(pd.concat(frames).sort_values(['surf', 'sp', 'src']).round(4).to_string(index=False))
