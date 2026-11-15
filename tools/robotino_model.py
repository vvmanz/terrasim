"""Robotino (3 omni wheels) model helpers shared by the generator, the surrogate fit and the analysis.

    ini_text(P, surface, speed, log_file, ...)   -> INI for the plugin / surrogate (same file for both)
    run_surrogate(P, surface, speed, workdir, ...) -> dataset-format DataFrame (columns of dataset_belyaev.csv)
    run_level(df)                                  -> one row per run: currents, slips, speed ratio, yaw rate
"""
import os, json, subprocess, copy
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
SURR = os.path.join(ROOT, 'core', 'build', 'robotino2d')
THETA = np.radians([60.0, 180.0, 300.0])


def load_params():
    return json.load(open(os.path.join(ROOT, 'configs', 'calibrated_params.json')))['robotino']


def _kv(d):
    return [f'{k} = {v}' for k, v in d.items() if not str(k).startswith('_') and not isinstance(v, (list, dict))]


def ini_text(P, surface, speed, log_file, t_run=None, t_pause=None, directions=None, links=None, detail_rate=None,
             label=None, map_file=None, return_to_start=None, full_rate=50.0, imu=None):
    E = P['experiment']; Rb = P['robot']
    t_run = t_run if t_run is not None else E['t_run'][str(speed)]
    t_pause = t_pause if t_pause is not None else E['t_pause']
    L = ['[general]', f'log_file = {log_file}', f'chassis_link = base_link', f'full_rate = {full_rate}']
    L += ['[robot]', 'type = omni'] + _kv(Rb)
    if 'encoder_ticks' not in Rb: L += ['encoder_ticks = 1970']   # counts per motor revolution, identified from m*pos (belyaev_kushnarev.csv)
    L += ['[imu]'] + _kv(imu if imu else P.get('imu', {'rate': 100, 'gyro_sigma': 0.002, 'gyro_bias': 0.005, 'gyro_rw': 2e-5,
                                                      'acc_sigma': 0.03, 'acc_bias': 0.1, 'acc_rw': 2e-4, 'seed': 11}))
    for k, th in enumerate(Rb['theta_deg'], 1):
        L += [f'[wheel {k}]', f'theta_deg = {th}']
        if links: L += [f'link = wheel{k}', f'joint = wheel{k}_joint']
    L += ['[omni_contact]'] + _kv(P['omni_contact'])
    L += ['[motor]'] + _kv(P['motor'])
    for name, S in P['surfaces'].items():
        L += [f'[surface {name}]'] + _kv(S)
    L += ['[experiment]', f'surface = {surface}', f'speed = {speed}', f't_run = {t_run}', f't_pause = {t_pause}',
          f"t_start = {E['t_start']}", f"camera_rate = {E['camera_rate']}"]
    if detail_rate: L += [f'detail_rate = {detail_rate}']
    if label: L += [f'label = {label}']
    if return_to_start is not None: L += [f'return_to_start = {int(bool(return_to_start))}']
    if directions is not None: L += ['directions = ' + ', '.join(str(d) for d in directions)]
    if map_file: L += ['[map]', f'dtm_file = {map_file}']
    return '\n'.join(L) + '\n'


def run_surrogate(P, surface, speed, workdir, dt=0.001, **kw):
    os.makedirs(workdir, exist_ok=True)
    ini = os.path.join(workdir, f'{surface}_{speed}.ini'); log = os.path.join(workdir, f'{surface}_{speed}.csv')
    open(ini, 'w').write(ini_text(P, surface, speed, log, **kw))
    r = subprocess.run([SURR, ini, '--dt', str(dt)], capture_output=True, text=True)
    if r.returncode != 0: raise RuntimeError(r.stderr)
    return pd.read_csv(log)


def run_level(d, t0_ms=1500.0):
    """per-run means after t0 (both for the dataset and simulation logs)"""
    d = d.copy()
    d['run'] = (d['t'].diff() < 0).cumsum()
    rows = []
    for r, g in d.groupby('run'):
        g2 = g[g.t > t0_ms]
        if len(g2) < 5: continue
        T = (g2.t.iloc[-1] - g2.t.iloc[0]) / 1000
        vcmd = float(np.hypot(g.xsetspeed.iloc[0], g.ysetspeed.iloc[0]))
        dang = (g2.ang.iloc[-1] - g2.ang.iloc[0] + 180) % 360 - 180
        row = dict(run=r, surf=g.surf.iloc[0], sp=float(g.speedamp.iloc[0]), dir=float(g.movedir.iloc[0]),
                   ratio=float(np.hypot(g2.xpos.iloc[-1] - g2.xpos.iloc[0], g2.ypos.iloc[-1] - g2.ypos.iloc[0]) / T / vcmd),
                   yawrate=float(dang / T),
                   dir_err=float((np.degrees(np.arctan2(g2.vy.mean(), g2.vx.mean())) - g.movedir.iloc[0] + 180) % 360 - 180))
        for i in (1, 2, 3):
            row[f'I{i}'] = float(g2[f'm{i}cur'].abs().mean())
            wa, we = g2[f'w{i}vel'].mean(), g2[f'w{i}effvel'].mean()
            ws = g2[f'm{i}setvel'].mean() / 16
            row[f'wa{i}'] = float(wa); row[f'we{i}'] = float(we); row[f'ws{i}'] = float(ws)
            row[f's{i}'] = float(1 - we / wa) if abs(wa) > 0.3 else np.nan
            row[f'e{i}'] = float((ws - wa) * np.sign(ws) * 16) if abs(ws) > 0.6 else np.nan
        rows.append(row)
    return pd.DataFrame(rows)
