"""Slip-versus-current figures for a Robotino dataset (scripts/robotino.sh output or any file in the same format).

Definitions (wheel i, theta = 60, 180, 300 deg for wheels 1, 2, 3):
  w_act_i  = motor speed / gear                      (encoder wheel speed)                   -> column w{i}vel
  w_eff_i  = (1/r) M v_body                          (from the body velocity, inverse kinematics) -> column w{i}effvel
  dw_i     = |w_act_i - w_eff_i|                     absolute slip, rad/s
  s_i      = 1 - w_eff_i / w_act_i                   relative slip                           -> column w{i}slip
  s_ic     = |sin(alpha - theta_i)| * s_i            slip weighted by how much the wheel drives ~ column w{i}linslip
             (alpha = direction of motion)
Values are averaged over 0.5 s windows inside every run; the first 1 s (acceleration) is skipped.

    python3 tools/robotino_figures.py datasets/<run>/dataset_clean.csv [--out datasets/<run>/figures]
"""
import os, sys, argparse
import numpy as np, pandas as pd

THETA = np.radians([60.0, 180.0, 300.0])
SURF_COLORS = {'gray': '#d62728', 'green': '#1f3fd6', 'table': '#111111', 'red': '#ff9900',
               'table_wet': '#8c6d31', 'carpet': '#6a3d9a', 'carpet_shag': '#e377c2', 'foam': '#bcbd22',
               'sand_lete': '#c49c94', 'sandy_loam': '#8c564b', 'clay_wet': '#7f7f7f', 'mars_soil': '#17becf',
               'tile': '#9edae5', 'linoleum': '#aec7e8', 'concrete': '#98df8a', 'gray_wet': '#ff9896', 'green_wet': '#c5b0d5', 'ice': '#4fa3d1'}
SURF_NAMES = {'gray': 'gray rubber mat', 'green': 'green plastic turf', 'table': 'table (chipboard)', 'red': 'red EVA mat',
              'table_wet': 'wet table', 'carpet': 'carpet 6 mm', 'carpet_shag': 'shag carpet 15 mm', 'foam': 'foam mat 20 mm',
              'sand_lete': 'dense sand (LETE)', 'sandy_loam': 'sandy loam', 'clay_wet': 'wet clay', 'mars_soil': 'Mars soil simulant',
              'tile': 'ceramic tile', 'linoleum': 'linoleum', 'concrete': 'concrete', 'gray_wet': 'wet gray mat',
              'green_wet': 'wet green turf', 'ice': 'ice'}
_EXTRA = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d', '#666666']
SPEED_SHADE = {0.1: 0.35, 0.2: 0.65, 0.3: 1.0}


def surf_color(s):
    if s not in SURF_COLORS: SURF_COLORS[s] = _EXTRA[len(SURF_COLORS) % len(_EXTRA)]
    return SURF_COLORS[s]


def surf_name(s):
    return SURF_NAMES.get(s, s)


def windows(d, win_ms=500.0, t_skip_ms=1000.0):
    """0.5 s window means per run and motor: current, relative / weighted / absolute slip"""
    d = d.copy()
    d['run'] = (d['t'].diff() < 0).cumsum()
    d = d[d['t'] >= t_skip_ms]
    d['win'] = (d['t'] // win_ms).astype(int)
    alpha = np.radians(d['movedir'].astype(float))
    rows = []
    for i in (1, 2, 3):
        p = pd.DataFrame({'run': d['run'], 'win': d['win'], 'surf': d['surf'], 'speed': d['speedamp'], 'dir': d['movedir'], 'motor': i,
                          'cur': d[f'm{i}cur'].abs(), 'w_act': d[f'w{i}vel'], 'w_eff': d[f'w{i}effvel'],
                          's': d[f'w{i}slip'], 'linslip_col': d[f'w{i}linslip']})
        p['dw'] = (p['w_act'] - p['w_eff']).abs()
        p['wnorm'] = np.abs(np.sin(alpha - THETA[i - 1]))
        p['s_c'] = p['wnorm'] * p['s']
        rows.append(p)
    L = pd.concat(rows, ignore_index=True)
    agg = L.groupby(['run', 'win', 'motor']).agg(surf=('surf', 'first'), speed=('speed', 'first'), dir=('dir', 'first'),
                                                 cur=('cur', 'mean'), s=('s', 'mean'), s_c=('s_c', 'mean'), dw=('dw', 'mean'),
                                                 linslip_col=('linslip_col', 'mean'), wnorm=('wnorm', 'mean'), n=('cur', 'size')).reset_index()
    return agg[agg['n'] >= 5]


def scatter_fig(A, ycol, ylabel, path, title, by_speed=False, ylim=(0, 1), surfaces=None):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    surfaces = list(surfaces) if surfaces is not None else sorted(A.surf.unique())
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    for a, m in zip(ax, (1, 3, 2)):
        for s in surfaces:
            g = A[(A.motor == m) & (A.surf == s)]
            if g.empty: continue
            if by_speed:
                for sp, sh in SPEED_SHADE.items():
                    gg = g[np.isclose(g.speed, sp)]
                    if gg.empty: continue
                    c = np.array(to_rgb(surf_color(s))); c = 1 - sh * (1 - c) if s != 'table' else np.array([0.6, 0.6, 0.6]) * (1.3 - sh)
                    a.scatter(gg.cur, gg[ycol], s=3, color=np.clip(c, 0, 1), alpha=.7, label=f'{surf_name(s)} {sp} m/s')
            else:
                a.scatter(g.cur, g[ycol], s=3, color=surf_color(s), alpha=.6, label=surf_name(s))
        a.set_xlim(0, 1.8); a.set_ylim(*ylim); a.grid(alpha=.3); a.set_xlabel('motor current, A'); a.set_ylabel(ylabel)
        a.set_title(f'motor {m}')
    ax[0].legend(fontsize=7, markerscale=3, loc='upper left')
    fig.suptitle(title)
    plt.tight_layout(); plt.savefig(path, dpi=95); plt.close(fig)


def linear_models(A, ycol):
    """least-squares line y = a*I + b per motor and surface"""
    out = []
    for (s, m), g in A.groupby(['surf', 'motor']):
        g = g[np.isfinite(g[ycol]) & (g[ycol].between(-0.5, 1.5))]
        if len(g) < 20: continue
        a, b = np.polyfit(g.cur, g[ycol], 1)
        out.append(dict(surf=s, motor=m, a=a, b=b, n=len(g), r=float(np.corrcoef(g.cur, g[ycol])[0, 1])))
    return pd.DataFrame(out)


def all_figures(A, out, tag='', surfaces=None):
    """the three slip-vs-current figures and the linear fits of the weighted slip"""
    os.makedirs(out, exist_ok=True)
    sfx = f'_{tag}' if tag else ''
    scatter_fig(A, 's', 'relative slip s', os.path.join(out, f'slip_vs_current{sfx}.png'),
                f'Relative slip vs motor current {tag}'.strip(), surfaces=surfaces)
    scatter_fig(A, 'dw', '|w_enc - w_eff|, rad/s', os.path.join(out, f'abs_slip_vs_current{sfx}.png'),
                f'Absolute slip vs motor current, by speed {tag}'.strip(), by_speed=True, ylim=(0, 6), surfaces=surfaces)
    scatter_fig(A, 's_c', 'weighted slip s_c', os.path.join(out, f'weighted_slip_vs_current{sfx}.png'),
                f'Weighted slip vs motor current {tag}'.strip(), surfaces=surfaces)
    M = linear_models(A, 's_c'); M.to_csv(os.path.join(out, f'linear_models_s_c{sfx}.csv'), index=False)
    return M


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv'); ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.csv)), 'figures')
    A = windows(pd.read_csv(a.csv))
    A.to_csv(os.path.join(out if os.path.isdir(out) else (os.makedirs(out) or out), 'windows_0p5s.csv.gz'), index=False)
    M = all_figures(A, out)
    print(M.round(3).to_string(index=False))
    print(f'figures -> {out}')
