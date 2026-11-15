"""Export simulated Robotino logs as datasets with the columns of dataset_belyaev.csv (+ the surface under each wheel).

    clean : the plugin log as is (camera-rate samples of the true motion, true encoder speeds and currents);
            the conventions applied are the dataset's (identified on belyaev_kushnarev.csv, exact on all rows):
            a wheel whose speed reads at most one encoder step (|motor speed| < 1.5 * 2.8274 rad/s) gets
            slip = linslip = 0; linslip_i = |sin(phi - th_i)| slip_i, phi = direction of the wheel-odometry
            velocity J^-1 w (not the commanded direction).
    noisy : the same run through the measurement chain of the real bench (tools/robotino_sensor.py):
            camera 29.4 Hz read by a 32 Hz logger (repeated frames), position/yaw noise, encoder counts,
            current noise and quantisation; m_vol, m_pow, rpower, Ke recomputed from the measured values
            exactly as in the dataset (U = sign(w) I R + Ce w, Ke = sum|U I| / (0.17 sum|U|)).

After the 41 dataset columns come SURF_COLUMNS: for every wheel the contact point and the surface parameters the
contact model used at that sample -- w{i}_x, w{i}_y (m, world), w{i}_mu, w{i}_K (m), w{i}_f_roll, w{i}_f_lat,
w{i}_mu_rb, w{i}_kc, w{i}_kphi (Bekker moduli, 0 on firm floors). They vary along the path where the surface is
heterogeneous (cv_res: f_roll, f_lat, mu_rb; cv_grip: K; cv_soil: kc, kphi) and at the borders of map zones; the
parameters that never vary are in surfaces_constant.csv of the dataset folder. strict=True drops them (exactly the
41 columns of the real file). In the noisy file they are the true values interpolated at the logger times.
"""
import numpy as np, pandas as pd

DATASET_COLUMNS = ['t', 'surf', 'movedir', 'speedamp', 'xsetspeed', 'ysetspeed', 'm1setvel', 'm2setvel', 'm3setvel',
                   'm1vel', 'm2vel', 'm3vel', 'm1cur', 'm2cur', 'm3cur', 'm1vol', 'm2vol', 'm3vol', 'm1pow', 'm2pow', 'm3pow',
                   'xpos', 'ypos', 'ang', 'vx', 'vy', 'omega', 'rpower', 'Ke', 'w1vel', 'w2vel', 'w3vel',
                   'w1effvel', 'w2effvel', 'w3effvel', 'w1slip', 'w2slip', 'w3slip', 'w1linslip', 'w2linslip', 'w3linslip']
SURF_KEYS = ('x', 'y', 'mu', 'K', 'f_roll', 'f_lat', 'mu_rb', 'kc', 'kphi')
SURF_COLUMNS = [f'w{i}_{k}' for i in (1, 2, 3) for k in SURF_KEYS]
THETA = np.radians([60.0, 180.0, 300.0])
R_MOT, CE, I_N, ENC_STEP = 5.3351, 0.0501, 0.17, 2.8274


def energy_columns(d):
    """m{i}vol, m{i}pow, rpower, Ke from currents and motor speeds, as in the dataset"""
    U = []
    for i in (1, 2, 3):
        w = d[f'm{i}vel'].values; I = d[f'm{i}cur'].abs().values
        sgn = np.sign(w); sgn = np.where(sgn == 0, np.sign(d[f'm{i}setvel'].values), sgn)
        u = sgn * I * R_MOT + CE * w
        d[f'm{i}vol'] = u; d[f'm{i}pow'] = np.abs(u * I); U.append(np.abs(u))
    Us = np.sum(U, axis=0)
    d['rpower'] = d['m1pow'] + d['m2pow'] + d['m3pow']
    d['Ke'] = np.where(Us > 1e-9, d['rpower'] / (I_N * np.where(Us > 1e-9, Us, 1)), 0.0)
    return d


def slip_columns(d, dead=1.5 * ENC_STEP):
    W = d[['w1vel', 'w2vel', 'w3vel']].values
    Jm = np.c_[-np.sin(THETA), np.cos(THETA), 0.125 * np.ones(3)] / 0.04
    tw = W @ np.linalg.inv(Jm).T
    alpha = np.arctan2(tw[:, 1], tw[:, 0])
    for i in (1, 2, 3):
        wa = d[f'w{i}vel'].values; we = d[f'w{i}effvel'].values
        ok = np.abs(d[f'm{i}vel'].values) >= dead
        s = np.where(ok, 1 - we / np.where(ok, wa, 1), 0.0)
        d[f'w{i}slip'] = s; d[f'w{i}linslip'] = np.abs(np.sin(alpha - THETA[i - 1])) * s
    return d


def columns(d, strict=False):
    return DATASET_COLUMNS + ([] if strict else [c for c in SURF_COLUMNS if c in d])


def clean(log, strict=False):
    d = log.copy()
    if 'rpower' not in d or 'Ke' not in d: d = energy_columns(d)
    d = slip_columns(d)
    return d[columns(d, strict)]


def noisy(log, seed=1, strict=False):
    import robotino_sensor as SNS
    d = SNS.apply(log, seed=seed)
    d = energy_columns(d)
    return d[columns(d, strict)]


def write(df, path):
    df.to_csv(path, index=False, float_format='%.7g')
