"""DTM (digital terrain map) writer + terrain profiles used by all experiments.

DTM text format is parsed by terramech::DTM::load (core/include/terramech/terramech.hpp).
"""
import math, os
import numpy as np

TABLE2 = dict(kc=13.6e3, kphi=2259.1e3, n0=0.92, n1=0.5, c=462.3, phi_deg=35.0, K=0.015, Ky=0.015, soil=1)
TABLE1 = dict(kc=15.6e3, kphi=2407.6e3, n0=0.895, n1=0.995, c=250.0, phi_deg=31.9, K=0.01, Ky=0.01, soil=1)
RIGID = dict(kc=0, kphi=0, n0=1, n1=0, c=0, phi_deg=0, K=0.01, Ky=0.01, soil=0)


def write_dtm(path, x0, y0, res, H, T, types):
    """H, T: arrays [ny, nx]; types: list of (name, dict)"""
    H = np.asarray(H, float); T = np.asarray(T, int)
    ny, nx = H.shape
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write('terramech_dtm 1\n')
        f.write(f'{nx} {ny} {x0:.6f} {y0:.6f} {res:.6f}\n{len(types)}\n')
        for name, p in types:
            f.write(f"{name} {p['kc']:.6g} {p['kphi']:.6g} {p['n0']:.6g} {p['n1']:.6g} {p['c']:.6g} {p['phi_deg']:.6g} {p['K']:.6g} {p['Ky']:.6g} {p.get('soil', 1):.6g}\n")
        for j in range(ny):
            f.write(' '.join(f'{v:.5f}' for v in H[j]) + '\n')
        for j in range(ny):
            f.write(' '.join(str(int(v)) for v in T[j]) + '\n')


def grid(xmin, xmax, ymin, ymax, res):
    xs = np.arange(xmin, xmax + 1e-9, res)
    ys = np.arange(ymin, ymax + 1e-9, res)
    return xs, ys


def soil_bedrock(path, soil=TABLE2, x_rock0=1.20, rock_len=1.00, rock_dh=0.0, res=0.02,
                 xmin=-3.0, xmax=6.0, ymin=-1.2, ymax=1.2, rock_ymin=-1.2, rock_ymax=1.2):
    """Flat soft soil with a rigid bedrock slab [x_rock0, x_rock0+rock_len] whose top is rock_dh above the soil."""
    xs, ys = grid(xmin, xmax, ymin, ymax, res)
    X, Y = np.meshgrid(xs, ys)
    H = np.zeros_like(X)
    T = np.zeros(X.shape, int)
    rock = (X >= x_rock0) & (X <= x_rock0 + rock_len) & (Y >= rock_ymin) & (Y <= rock_ymax)
    H[rock] = rock_dh
    T[rock] = 1
    write_dtm(path, xmin, ymin, res, H, T, [('soil', soil), ('bedrock', RIGID)])
    return dict(rigid_boxes=[dict(x0=x_rock0, x1=x_rock0 + rock_len, y0=rock_ymin, y1=rock_ymax, ztop=rock_dh, zbot=rock_dh - 0.05)])


def mound_profile(x, x_start, L_up, L_top, L_down, height, smooth=0.15):
    """sandy slope: smooth trapezoid (cosine-blended corners)."""
    x = np.asarray(x, float)
    xa, xb, xc, xd = x_start, x_start + L_up, x_start + L_up + L_top, x_start + L_up + L_top + L_down
    h = np.zeros_like(x)
    up = (x > xa) & (x < xb); h[up] = height * (x[up] - xa) / L_up
    top = (x >= xb) & (x <= xc); h[top] = height
    dn = (x > xc) & (x < xd); h[dn] = height * (xd - x[dn]) / L_down
    if smooth > 0:  # gaussian blur in x
        dx = x[1] - x[0] if len(x) > 1 else 0.01
        k = max(1, int(smooth / dx))
        ker = np.exp(-0.5 * (np.arange(-3 * k, 3 * k + 1) / k) ** 2); ker /= ker.sum()
        hp = np.pad(h, 3 * k, mode='edge'); h = np.convolve(hp, ker, mode='valid')
    return h


def sandy_slope(path, soil=TABLE2, x_start=1.0, L_up=0.8, L_top=0.3, L_down=0.8, height=0.14, smooth=0.1,
                res=0.02, xmin=-3.0, xmax=7.0, ymin=-1.2, ymax=1.2):
    xs, ys = grid(xmin, xmax, ymin, ymax, res)
    prof = mound_profile(xs, x_start, L_up, L_top, L_down, height, smooth)
    H = np.tile(prof, (len(ys), 1))
    T = np.zeros(H.shape, int)
    write_dtm(path, xmin, ymin, res, H, T, [('soil', soil)])
    return dict(rigid_boxes=[], profile=(xs, prof))


def flat(path, soil=TABLE2, res=0.05, xmin=-5, xmax=15, ymin=-5, ymax=5, name='soil'):
    xs, ys = grid(xmin, xmax, ymin, ymax, res)
    H = np.zeros((len(ys), len(xs)))
    T = np.zeros(H.shape, int)
    write_dtm(path, xmin, ymin, res, H, T, [(name, soil)])
    return dict(rigid_boxes=[])


def write_obj(path, dtm_path, z_offset=0.0, step=1):
    """Visual mesh of the DTM surface (for Gazebo visuals)."""
    with open(dtm_path) as f:
        toks = f.read().split()
    nx, ny = int(toks[2]), int(toks[3]); x0, y0, res = float(toks[4]), float(toks[5]), float(toks[6])
    nt = int(toks[7]); p = 8 + nt * 10
    H = np.array(toks[p:p + nx * ny], float).reshape(ny, nx)
    ii = list(range(0, nx, step)); jj = list(range(0, ny, step))
    if ii[-1] != nx - 1: ii.append(nx - 1)
    if jj[-1] != ny - 1: jj.append(ny - 1)
    with open(path, 'w') as f:
        f.write('# terramech DTM surface\n')
        for j in jj:
            for i in ii:
                f.write(f'v {x0 + i * res:.4f} {y0 + j * res:.4f} {H[j, i] + z_offset:.4f}\n')
        for j in jj:
            for i in ii:
                f.write(f'vt {(i / (nx - 1)) * (nx * res / 1.0):.4f} {(j / (ny - 1)) * (ny * res / 1.0):.4f}\n')
        f.write('vn 0 0 1\n')
        W = len(ii)
        for a in range(len(jj) - 1):
            for b in range(W - 1):
                v1 = a * W + b + 1; v2 = v1 + 1; v3 = v1 + W; v4 = v3 + 1
                f.write(f'f {v1}/{v1}/1 {v2}/{v2}/1 {v4}/{v4}/1\nf {v1}/{v1}/1 {v4}/{v4}/1 {v3}/{v3}/1\n')
