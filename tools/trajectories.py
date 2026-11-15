"""Reference paths for the vehicle mode: s, x, y, yaw, kappa sampled every ~2 mm in the START frame
(origin = spawn point, +x = the robot's forward direction at spawn). The plugin rotates/translates the path
into the world with the pose the robot has when it starts moving.

Specs (as for scripts/drive.sh --traj):
    line:L=3,angle=0              straight line of length L (angle: direction in the start frame, deg — with
                                  --heading fixed a holonomic robot drives sideways at this angle, like a star ray)
    circle:R=1,loops=1            circle of radius R, turning left (R<0: right), starting tangentially
    square:L=2,rc=0.3,loops=1     square of side L (counter-clockwise), corners rounded with radius rc
                                  (rc=0: sharp corners — only sensible for holonomic robots)
    rect:Lx=3,Ly=1.5,rc=0.3       rectangle
    figure8:A=1.5,loops=1         figure eight (Gerono lemniscate) of half-width A
    spline:pts=1,0.5;2,0;3,-0.5   Catmull–Rom spline from the start through the points (start frame, metres)
    slalom:L=4,A=0.4,n=2          slalom along +x of length L, n periods, amplitude up to A in the middle
    spiral:R0=0.3,R1=1.5,turns=2  Archimedean spiral outwards
Every path starts at (0, 0) with the tangent along +x (curves are rotated accordingly).
"""
import math, os, sys
import numpy as np


def parse(spec):
    kind, _, rest = spec.partition(':')
    opts = {}
    for kv in filter(None, rest.split(',') if kind != 'spline' else [rest]):
        if kind == 'spline' and kv.startswith('pts='):
            opts['pts'] = [tuple(float(v) for v in p.split()) for p in kv[4:].replace(',', ' ').split(';') if p.strip()]
            continue
        k, v = kv.split('=', 1); opts[k] = float(v)
    return kind, opts


def _arc(cx, cy, r, a0, a1, n):
    a = np.linspace(a0, a1, n)
    return cx + r * np.cos(a), cy + r * np.sin(a)


def _rounded_rect(Lx, Ly, rc):
    rc = min(rc, 0.5 * Lx, 0.5 * Ly)
    pts = []
    # start at the middle of the bottom edge, heading +x, counter-clockwise
    segs = [((0, 0), (Lx / 2 - rc, 0)), ('arc', (Lx / 2 - rc, rc), -math.pi / 2, 0),
            ((Lx / 2, rc), (Lx / 2, Ly - rc)), ('arc', (Lx / 2 - rc, Ly - rc), 0, math.pi / 2),
            ((Lx / 2 - rc, Ly), (-Lx / 2 + rc, Ly)), ('arc', (-Lx / 2 + rc, Ly - rc), math.pi / 2, math.pi),
            ((-Lx / 2, Ly - rc), (-Lx / 2, rc)), ('arc', (-Lx / 2 + rc, rc), math.pi, 1.5 * math.pi),
            ((-Lx / 2 + rc, 0), (0, 0))]
    for sg in segs:
        if sg[0] == 'arc':
            if rc <= 0: continue
            (cx, cy), a0, a1 = sg[1], sg[2], sg[3]
            x, y = _arc(cx, cy, rc, a0, a1, max(8, int(abs(a1 - a0) * rc / 0.002)))
        else:
            (x0, y0), (x1, y1) = sg
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.002))
            x, y = np.linspace(x0, x1, n), np.linspace(y0, y1, n)
        pts.append(np.c_[x, y])
    return np.vstack(pts)


def _catmull_rom(P, n_per=400):
    P = np.asarray(P, float)
    P = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        t = np.linspace(0, 1, n_per, endpoint=False)[:, None]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    out.append(P[-2][None, :])
    return np.vstack(out)


def raw_path(kind, o):
    loops = int(o.get('loops', 1))
    if kind == 'line':
        L = o.get('L', 3.0); x = np.linspace(0, L, max(2, int(L / 0.002))); a = math.radians(o.get('angle', 0.0))
        return np.c_[x * math.cos(a), x * math.sin(a)]
    if kind == 'circle':
        R = o.get('R', 1.0); a = np.linspace(-math.pi / 2, -math.pi / 2 + 2 * math.pi * loops, int(2 * math.pi * abs(R) * loops / 0.002))
        xy = np.c_[abs(R) * np.cos(a), abs(R) * np.sin(a) + abs(R)]
        if R < 0: xy[:, 1] *= -1
        return xy
    if kind in ('square', 'rect'):
        Lx = o.get('L', o.get('Lx', 2.0)); Ly = o.get('L', o.get('Ly', Lx))
        one = _rounded_rect(Lx, Ly, o.get('rc', 0.3))
        return np.vstack([one] * loops)
    if kind == 'figure8':
        A = o.get('A', 1.5); t = np.linspace(0, 2 * math.pi * loops, int(6 * A * loops / 0.002))
        return np.c_[A * np.sin(t), A * np.sin(t) * np.cos(t)]
    if kind == 'spline':
        pts = [(0.0, 0.0)] + list(o.get('pts', [(1, 0.5), (2, 0), (3, -0.5)]))
        return _catmull_rom(pts)
    if kind == 'slalom':
        L, A, n = o.get('L', 4.0), o.get('A', 0.4), o.get('n', 2)
        x = np.linspace(0, L, int(L / 0.002))       # envelope sin(pi x/L): starts and ends along +x
        return np.c_[x, A * np.sin(2 * math.pi * n * x / L) * np.sin(math.pi * x / L)]
    if kind == 'spiral':
        R0, R1, turns = o.get('R0', 0.3), o.get('R1', 1.5), o.get('turns', 2)
        a = np.linspace(0, 2 * math.pi * turns, int(2 * math.pi * turns * R1 / 0.002))
        r = R0 + (R1 - R0) * a / a[-1]
        return np.c_[r * np.cos(a) - R0, r * np.sin(a)]
    raise SystemExit(f'unknown trajectory "{kind}" (line, circle, square, rect, figure8, spline, slalom, spiral)')


def build(spec, rotate=True):
    kind, o = parse(spec)
    xy = raw_path(kind, o)
    keep = np.r_[True, np.hypot(*np.diff(xy, axis=0).T) > 1e-6]     # drop duplicates
    xy = xy[keep]
    xy = xy - xy[0]
    d = np.diff(xy, axis=0)
    if rotate and kind != 'spline' and not (kind == 'line' and o.get('angle', 0.0) != 0.0):
        k0 = min(len(d) - 1, 5)
        a0 = math.atan2(d[:k0 + 1, 1].sum(), d[:k0 + 1, 0].sum())
        c, s = math.cos(-a0), math.sin(-a0)
        xy = xy @ np.array([[c, s], [-s, c]])
        d = np.diff(xy, axis=0)
    ds = np.hypot(d[:, 0], d[:, 1])
    s = np.r_[0, np.cumsum(ds)]
    yaw = np.unwrap(np.r_[np.arctan2(d[:, 1], d[:, 0]), math.atan2(d[-1, 1], d[-1, 0])])
    # curvature from the smoothed heading
    w = max(1, int(0.02 / max(np.median(ds), 1e-6)))
    ys = np.convolve(np.pad(yaw, w, mode='edge'), np.ones(2 * w + 1) / (2 * w + 1), mode='valid')
    kappa = np.gradient(ys, s)
    return np.c_[s, xy[:, 0], xy[:, 1], yaw, kappa]


def write(spec, path):
    A = build(spec)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savetxt(path, A, delimiter=',', header='s,x,y,yaw,kappa', comments='', fmt='%.6f')
    return A


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument('spec'); ap.add_argument('--out', default='/tmp/path.csv'); ap.add_argument('--plot', action='store_true')
    a = ap.parse_args()
    A = write(a.spec, a.out)
    print(f'{a.spec}: length {A[-1, 0]:.2f} m, {len(A)} points -> {a.out}')
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        plt.figure(figsize=(5, 5)); plt.plot(A[:, 1], A[:, 2]); plt.plot(0, 0, 'ko'); plt.axis('equal'); plt.grid(alpha=.3)
        plt.savefig(os.path.splitext(a.out)[0] + '.png', dpi=80)
