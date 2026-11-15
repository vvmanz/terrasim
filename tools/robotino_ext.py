"""Extended Robotino dataset: the 90 columns of belyaev_kushnarev.csv from the 41 columns of dataset_belyaev.csv.

    import robotino_ext as EXT
    ext = EXT.extend(d)                 # d: one run in the dataset format (clean or noisy), rays separated by t resets
    EXT.renumber([ext_a, ext_b, ...])   # several runs -> global exp_idx, run_idx per surface (as in the real table)
    python3 tools/robotino_ext.py --check /path/belyaev_kushnarev.csv    # rebuild the real table from its 41 base
                                                                          # columns and compare all 90 columns

Columns added (all identified on belyaev_kushnarev.csv, see docs/en/DATASETS.md):
  exp_idx, run_idx     ray number in the whole table / ray number within one surface (speeds in increasing order)
  setvel               always 0 in the real table (kept for the format)
  m{i}pos              encoder counts since the start of the ray, 1970 counts per motor revolution
  xcur, ycur, rotcur   M^-1 * signed currents (sign of m{i}vol), M rows (-sin th_i, cos th_i, 1)
  w{i}ddvel            w{i}vel - w{i}effvel (slip speed of the wheel, rad/s)
  ddvx, ddvy, ddomega  J^-1 * w_ddvel (chassis twist of the slip speeds; J rows (-sin th, cos th, R)/r)
  ddspeedvec           |v_odometry| - |v_camera|, v_odometry = J^-1 w_vel (planar part)
  *_kalman             per motor a linear 2-state filter x = [motor speed, load torque] with inputs (m_setvel, m_vol)
                       and measurement m_vel. It is deterministic (the same for every ray): x_k = F_k x_{k-1} + G_k
                       (m_vel, m_setvel, m_vol), F_k, G_k change during the first ~40 samples of a ray (gain of a Kalman
                       filter converging from its initial covariance) and are constant afterwards. The schedule was
                       identified exactly on the real table (--identify; configs/robotino_ext_filter.json).
                       m{i}cur_kalman: regression on the filter outputs (relative rms ~4e-4 on the real table);
                       m{i}vol/pow, rpower, Ke, w{i}vel, slips, dd-columns from the filtered values with the dataset formulas
  m{i}esttor           the filter's load-torque state, N m (motor shaft)
The base columns follow the dataset conventions found in the same file: slip = 0 when the motor speed reads at most
one encoder step (|m_vel| < 1.5 * 2.8274 rad/s for continuous speeds), linslip = |sin(phi_odo - th_i)| * slip with
phi_odo the direction of the wheel-odometry velocity; slip_kalman = 0 when |m_vel_kalman| < 3.2 rad/s.
"""
import os, sys, json, argparse
import numpy as np, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)

EXT_COLUMNS = ['t', 'exp_idx', 'run_idx', 'surf', 'movedir', 'speedamp', 'xsetspeed', 'ysetspeed', 'setvel', 'm1setvel', 'm2setvel', 'm3setvel',
               'm1pos', 'm2pos', 'm3pos', 'm1vel', 'm2vel', 'm3vel', 'm1cur', 'm2cur', 'm3cur', 'm1vol', 'm2vol', 'm3vol', 'm1pow', 'm2pow', 'm3pow',
               'xcur', 'ycur', 'rotcur', 'xpos', 'ypos', 'ang', 'vx', 'vy', 'omega', 'rpower', 'Ke', 'w1vel', 'w2vel', 'w3vel',
               'w1effvel', 'w2effvel', 'w3effvel', 'w1slip', 'w2slip', 'w3slip', 'w1ddvel', 'w2ddvel', 'w3ddvel', 'ddvx', 'ddvy', 'ddomega',
               'ddspeedvec', 'w1linslip', 'w2linslip', 'w3linslip', 'm1vel_kalman', 'm2vel_kalman', 'm3vel_kalman', 'm1cur_kalman',
               'm2cur_kalman', 'm3cur_kalman', 'm1vol_kalman', 'm2vol_kalman', 'm3vol_kalman', 'm1pow_kalman', 'm2pow_kalman', 'm3pow_kalman',
               'm3esttor', 'm1esttor', 'm2esttor', 'rpower_kalman', 'Ke_kalman', 'w1vel_kalman', 'w2vel_kalman', 'w3vel_kalman',
               'w1slip_kalman', 'w2slip_kalman', 'w3slip_kalman', 'w1ddvel_kalman', 'w2ddvel_kalman', 'w3ddvel_kalman', 'ddvx_kalman',
               'ddvy_kalman', 'ddomega_kalman', 'ddspeedvec_kalman', 'w1linslip_kalman', 'w2linslip_kalman', 'w3linslip_kalman']
THETA = np.radians([60.0, 180.0, 300.0])
R_ROB, R_WHEEL, GEAR = 0.125, 0.04, 16.0
R_MOT, CE, I_N, ENC_STEP = 5.3351, 0.0501, 0.17, 2.8274
J = np.c_[-np.sin(THETA), np.cos(THETA), R_ROB * np.ones(3)] / R_WHEEL           # wheel speeds = J (vx, vy, wz)
J_INV = np.linalg.inv(J)
M_CUR_INV = np.array([[-0.577, 0.0, 0.577], [0.333, -0.667, 0.333], [1 / 3, 1 / 3, 1 / 3]])   # M^-1 with the table's rounding
N_SCHED = 60                       # samples with their own filter coefficients; afterwards the steady-state ones
_F = None


def filt_params():
    global _F
    if _F is None: _F = json.load(open(os.path.join(ROOT, 'configs', 'robotino_ext_filter.json')))
    return _F


def rays(d):
    """ray number of every row: t restarts at every ray"""
    t = d['t'].values
    return np.r_[0, np.cumsum(np.diff(t) < 0)] if len(t) else np.zeros(0, int)


def _cur_regressors(Ik1, tau, tau1, U, U1, wk, wk1, z, sv, I, I1):
    return np.array([Ik1, abs(tau), abs(tau1), abs(U), abs(U1), abs(wk), abs(wk1), abs(z), abs(sv), I, I1, 1.0])


def kalman_motor(i, sv, U, z, I, ray):
    """the identified per-motor filter over all rows; returns w_kalman, esttor, cur_kalman"""
    P = filt_params()[f'm{i}']
    Wc = np.array(P['w']); Tc = np.array(P['tau']); Cc = np.array(P['cur'])
    n = len(z); W = np.zeros(n); T = np.zeros(n); C = np.zeros(n)
    wk = tau = Ik = 0.0; k = 0
    for j in range(n):
        if j == 0 or ray[j] != ray[j - 1]: k = 0
        q = min(k, len(Wc) - 1)
        x = np.array([wk, tau, z[j], sv[j], U[j]])
        w_new, t_new = float(Wc[q] @ x), float(Tc[q] @ x)
        if k == 0: r = _cur_regressors(0, t_new, 0, U[j], 0, w_new, 0, z[j], sv[j], I[j], 0)
        else: r = _cur_regressors(Ik, t_new, tau, U[j], U[j - 1], w_new, wk, z[j], sv[j], I[j], I[j - 1])
        Ik = max(float(Cc[q] @ r), 0.0)
        wk, tau = w_new, t_new
        W[j], T[j], C[j] = wk, tau, Ik; k += 1
    return W, T, C


def identify(path, out=None):
    """least-squares identification of the filter schedule from the real extended table"""
    R = pd.read_csv(path)
    R['k'] = R.groupby('exp_idx').cumcount(); G = R.groupby('exp_idx'); P = G.shift(1)
    res = {'_source': f'identified from {os.path.basename(path)} ({len(R)} rows, {R.exp_idx.nunique()} rays) by tools/robotino_ext.py --identify',
           '_model': 'per motor and sample index k of the ray (k >= N-1: steady state): [w_k, tau_k] = coef . [w_{k-1}, tau_{k-1}, m_vel, '
                     'm_setvel, m_vol] (w_-1 = tau_-1 = 0); cur_k = coef . [cur_{k-1}, |tau_k|, |tau_{k-1}|, |U_k|, |U_{k-1}|, |w_k|, |w_{k-1}|, '
                     '|m_vel|, |m_setvel|, m_cur, m_cur_{k-1}, 1]',
           'ticks_per_rev': 1970, 'slip_kalman_min_motor_speed': 3.2, 'slip_min_motor_speed': 1.5 * ENC_STEP, 'fit_rms': {}}
    for i in (1, 2, 3):
        Wc, Tc, Cc, rms = [], [], [], []
        for k in range(N_SCHED):
            m = (R.k == k).values if k < N_SCHED - 1 else (R.k >= k).values
            w1 = np.zeros(m.sum()) if k == 0 else P[f'm{i}vel_kalman'].values[m]
            t1 = np.zeros(m.sum()) if k == 0 else P[f'm{i}esttor'].values[m]
            X = np.c_[w1, t1, R[f'm{i}vel'].values[m], R[f'm{i}setvel'].values[m], R[f'm{i}vol'].values[m]]
            if k == 0: X[:, :2] = 0
            cw = np.linalg.lstsq(X, R[f'm{i}vel_kalman'].values[m], rcond=None)[0]
            ct = np.linalg.lstsq(X, R[f'm{i}esttor'].values[m], rcond=None)[0]
            if k == 0: cw[:2] = 0; ct[:2] = 0
            g = lambda c, n=1: (G[c].shift(n).values[m] if k > 0 else np.zeros(m.sum()))
            Xc = np.c_[g(f'm{i}cur_kalman'), R[f'm{i}esttor'].abs().values[m], np.abs(g(f'm{i}esttor')), R[f'm{i}vol'].abs().values[m],
                       np.abs(g(f'm{i}vol')), R[f'm{i}vel_kalman'].abs().values[m], np.abs(g(f'm{i}vel_kalman')), R[f'm{i}vel'].abs().values[m],
                       R[f'm{i}setvel'].abs().values[m], R[f'm{i}cur'].values[m], g(f'm{i}cur'), np.ones(m.sum())]
            yc = R[f'm{i}cur_kalman'].values[m]
            cc = np.linalg.lstsq(Xc, yc, rcond=None)[0]
            Wc.append(cw.tolist()); Tc.append(ct.tolist()); Cc.append(cc.tolist())
            rms.append([float(np.sqrt(np.mean((R[f'm{i}vel_kalman'].values[m] - X @ cw) ** 2))),
                        float(np.sqrt(np.mean((R[f'm{i}esttor'].values[m] - X @ ct) ** 2))), float(np.sqrt(np.mean((yc - Xc @ cc) ** 2)))])
        res[f'm{i}'] = dict(w=Wc, tau=Tc, cur=Cc); res['fit_rms'][f'm{i}'] = dict(first=rms[0], k1=rms[1], k10=rms[10], steady=rms[-1])
    json.dump(res, open(out or os.path.join(ROOT, 'configs', 'robotino_ext_filter.json'), 'w'), indent=1)
    global _F; _F = None
    return res


def _slips(d, suffix, wvel, mvel, thr):
    """slip and linslip with the dataset rules (see the module docstring)"""
    tw = wvel @ J_INV.T; phi = np.arctan2(tw[:, 1], tw[:, 0])
    for i in (1, 2, 3):
        wa = wvel[:, i - 1]; we = d[f'w{i}effvel'].values; ok = np.abs(mvel[:, i - 1]) >= thr
        s = np.where(ok, 1 - we / np.where(ok & (np.abs(wa) > 1e-12), wa, 1), 0.0)
        d[f'w{i}slip{suffix}'] = s
        d[f'w{i}linslip{suffix}'] = np.abs(np.sin(phi - THETA[i - 1])) * s
    return tw


def _dd(d, suffix, wvel, tw):
    wdd = wvel - d[['w1effvel', 'w2effvel', 'w3effvel']].values
    for i in (1, 2, 3): d[f'w{i}ddvel{suffix}'] = wdd[:, i - 1]
    tdd = wdd @ J_INV.T
    d['ddvx' + suffix] = tdd[:, 0]; d['ddvy' + suffix] = tdd[:, 1]; d['ddomega' + suffix] = tdd[:, 2]
    d['ddspeedvec' + suffix] = np.hypot(tw[:, 0], tw[:, 1]) - np.hypot(d['vx'].values, d['vy'].values)


def _energy(d, suffix, mvel, cur):
    U = []
    for i in (1, 2, 3):
        w = mvel[:, i - 1]; I = cur[:, i - 1]
        sg = np.sign(w); sg = np.where(sg == 0, np.sign(d[f'm{i}setvel'].values), sg)
        u = sg * I * R_MOT + CE * w
        d[f'm{i}vol{suffix}'] = u; d[f'm{i}pow{suffix}'] = np.abs(u * I); U.append(np.abs(u))
    Us = np.sum(U, axis=0)
    d['rpower' + suffix] = d[f'm1pow{suffix}'] + d[f'm2pow{suffix}'] + d[f'm3pow{suffix}']
    d['Ke' + suffix] = np.where(Us > 1e-9, d['rpower' + suffix] / (I_N * np.where(Us > 1e-9, Us, 1)), 0.0)


def extend(d, pos=None, base_rules=True):
    """d: dataset-format DataFrame (41 columns). pos: optional (n, 3) encoder counts; else integrated from m{i}vel.
    base_rules: recompute w{i}slip / linslip of the base columns with the dataset rules."""
    F = filt_params()
    d = d.reset_index(drop=True).copy()
    ray = rays(d)
    d['exp_idx'] = ray; d['run_idx'] = ray; d['setvel'] = 0
    mv = d[['m1vel', 'm2vel', 'm3vel']].values
    if pos is None:
        tk = F['ticks_per_rev'] / (2 * np.pi); pos = np.zeros((len(d), 3))
        t = d['t'].values / 1000.0
        for r in np.unique(ray):
            m = ray == r; tt = t[m]
            dt = np.diff(tt, prepend=tt[0] - (np.median(np.diff(tt)) if m.sum() > 1 else 0.031))
            v = mv[m]; vm = np.vstack([v[:1], 0.5 * (v[1:] + v[:-1])])
            pos[m] = np.cumsum(vm * dt[:, None], axis=0) * tk
    for i in (1, 2, 3): d[f'm{i}pos'] = np.round(pos[:, i - 1]).astype(np.int64)
    Isg = d[['m1cur', 'm2cur', 'm3cur']].abs().values * np.sign(d[['m1vol', 'm2vol', 'm3vol']].values)
    xc = Isg @ M_CUR_INV.T
    d['xcur'], d['ycur'], d['rotcur'] = xc[:, 0], xc[:, 1], xc[:, 2]
    wv = d[['w1vel', 'w2vel', 'w3vel']].values
    if base_rules: tw = _slips(d, '', wv, mv, F['slip_min_motor_speed'])
    else: tw = wv @ J_INV.T
    _dd(d, '', wv, tw)
    MK, CK = np.zeros_like(mv), np.zeros_like(mv)
    for i in (1, 2, 3):
        W, T, C = kalman_motor(i, d[f'm{i}setvel'].values, d[f'm{i}vol'].values, d[f'm{i}vel'].values, d[f'm{i}cur'].abs().values, ray)
        MK[:, i - 1] = W; CK[:, i - 1] = C; d[f'm{i}esttor'] = T
        d[f'm{i}vel_kalman'] = W; d[f'm{i}cur_kalman'] = C
    _energy(d, '_kalman', MK, CK)
    wk = MK / GEAR
    for i in (1, 2, 3): d[f'w{i}vel_kalman'] = wk[:, i - 1]
    twk = _slips(d, '_kalman', wk, MK, F['slip_kalman_min_motor_speed'])
    _dd(d, '_kalman', wk, twk)
    from robotino_dataset import SURF_COLUMNS
    return d[EXT_COLUMNS + [c for c in SURF_COLUMNS if c in d]]          # surface under each wheel (robotino_dataset.py)


def pos_like(ref, d):
    """encoder counts of ref (extended clean frame) interpolated at the rows of d (e.g. the noisy frame of the same run)"""
    rr, rd = rays(ref), rays(d); P = np.zeros((len(d), 3))
    for r in np.unique(rd):
        m = rd == r; mr = rr == r
        if not mr.any(): continue
        for i in (1, 2, 3): P[m, i - 1] = np.interp(d['t'].values[m], ref['t'].values[mr], ref[f'm{i}pos'].values[mr].astype(float))
    return P


def renumber(parts):
    """global exp_idx over all parts in the given order; run_idx counts rays per surface over the speeds"""
    out, e0, per_surf = [], 0, {}
    for p in parts:
        p = p.copy(); s = str(p['surf'].iloc[0]) if len(p) else ''
        r0 = per_surf.get(s, 0); n = int(p['exp_idx'].max()) + 1 if len(p) else 0
        p['exp_idx'] = p['exp_idx'] + e0; p['run_idx'] = p['run_idx'] + r0
        e0 += n; per_surf[s] = r0 + n; out.append(p)
    return pd.concat(out, ignore_index=True) if out else pd.DataFrame(columns=EXT_COLUMNS)


def write(df, path):
    df.to_csv(path, index=False, float_format='%.7g')


def check(path, rays_max=None):
    """rebuild the real extended table from its 41 base columns and compare every added column"""
    import robotino_dataset as DS
    R = pd.read_csv(path)
    if rays_max: R = R[R.exp_idx < rays_max].reset_index(drop=True)
    parts = []
    for e, g in R.groupby('exp_idx', sort=True):
        base = g[DS.DATASET_COLUMNS].reset_index(drop=True)
        x = extend(base, pos=g[['m1pos', 'm2pos', 'm3pos']].values, base_rules=True)
        x['exp_idx'] = e; x['run_idx'] = g['run_idx'].values
        parts.append(x)
    X = pd.concat(parts, ignore_index=True); R = R.reset_index(drop=True)
    rows = []
    for c in EXT_COLUMNS:
        if c in ('surf',): continue
        a = pd.to_numeric(X[c], errors='coerce').values; b = pd.to_numeric(R[c], errors='coerce').values
        e = np.abs(a - b); sc = np.nanstd(b)
        rows.append(dict(column=c, exact_share=float(np.mean(e < 1e-6 * max(1.0, sc))), rms=float(np.sqrt(np.nanmean(e ** 2))),
                         rel_rms=float(np.sqrt(np.nanmean(e ** 2)) / sc) if sc > 0 else 0.0, median_abs=float(np.nanmedian(e))))
    T = pd.DataFrame(rows)
    # m{i}pos is taken from the table here; the integrated version is checked separately
    for i in (1, 2, 3):
        x = extend(R[DS.DATASET_COLUMNS].iloc[:20000].reset_index(drop=True))
        P = R[f'm{i}pos'].values[:20000]; Q = x[f'm{i}pos'].values
        ray = rays(R.iloc[:20000]); last = np.r_[np.diff(ray) != 0, True]
        T.loc[len(T)] = dict(column=f'm{i}pos (integrated from m{i}vel, end of ray)', exact_share=np.nan,
                             rms=float(np.sqrt(np.mean((P[last] - Q[last]) ** 2))), rel_rms=float(np.median(np.abs(P[last] - Q[last]) / np.maximum(np.abs(P[last]), 1))),
                             median_abs=float(np.median(np.abs(P[last] - Q[last]))))
    return T


if __name__ == '__main__':
    sys.path.insert(0, HERE)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', default='', help='path to belyaev_kushnarev.csv')
    ap.add_argument('--identify', default='', help='path to belyaev_kushnarev.csv: re-identify configs/robotino_ext_filter.json')
    ap.add_argument('--rays', type=int, default=0, help='only the first N rays (faster)')
    ap.add_argument('--out', default='', help='write the comparison table (csv)')
    a = ap.parse_args()
    if a.identify:
        r = identify(a.identify); print(json.dumps(r['fit_rms'], indent=1))
    if a.check:
        T = check(a.check, a.rays or None)
        pd.set_option('display.width', 200); pd.set_option('display.max_rows', 200)
        print(T.to_string(index=False, float_format=lambda v: f'{v:.3g}'))
        if a.out: T.to_csv(a.out, index=False)
