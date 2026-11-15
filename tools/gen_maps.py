"""Terrain maps for the omni robot (heights + surface type per node) -> DTM for the plugin + meshes for Gazebo.

The star experiment starts at the map origin (0, 0); with a map the robot drives back to the centre after
every ray (return_to_start), so that every ray crosses the same terrain from the same point.

Map specs (as for scripts/robotino.sh --map):
    hills:A=0.01,L=0.4,r0=0.3,r1=2.5        concentric smooth ring bumps, height A, wavelength L, from radius
                                            r0 to r1 (one surface: the run's surface)
    rings:surfaces=gray+carpet,w=0.5,r0=0.3 flat, concentric rings alternating between surfaces (transitions)
    sectors:surfaces=gray+green+table        flat, angular sectors of different surfaces (direction <-> surface)
    patches:surfaces=gray+carpet+foam,cell=0.4,seed=3   flat, random Voronoi patches of the surfaces
    hills and rings/sectors/patches combine:  rings:surfaces=gray+carpet,A=0.01,L=0.4
    file:/abs/path/map.dtm                   own map (format: core/include/terramech/terramech.hpp, DTM::load);
                                            type names must be surfaces of the library
    slope_10 | map:slope_10 | my_map.yaml    map file (configs/maps, tools/maps.py, docs/en/MAPS.md); the star starts at
                                            the map origin (0, 0)
Common options: res (node spacing, 0.01 m), size (half-width of the square map, 3.0 m).
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_dtm

DUMMY_SOIL = dict(kc=0, kphi=0, n0=1, n1=0, c=0, phi_deg=30, K=0.01, Ky=0.01, soil=0)   # unused by the omni contact


def _opts(rest):
    o = {}
    for kv in filter(None, rest.split(',')):
        k, v = kv.split('=', 1)
        o[k] = v.split('+') if k == 'surfaces' else (v if k in ('surface',) else float(v))
    return o


KINDS = ('hills', 'rings', 'sectors', 'patches', 'checkerboard', 'stripes', 'voronoi', 'half', 'file')


def parse_spec(spec):
    """spec of a generated map, or a map file: map:NAME / NAME (configs/maps/NAME.yaml, tools/maps.py) / path.yaml"""
    kind, _, rest = spec.partition(':')
    if kind == 'file': return 'file', {'path': rest}
    if kind == 'map': return 'mapfile', {'name': rest}
    if kind not in KINDS:
        import maps as MP
        MP.find(spec.partition('@')[0])            # exits with a message if there is no such map
        return 'mapfile', {'name': spec}
    return kind, _opts(rest)


# ------------------------------------------------------------------------------------------ relief
def relief(spec, X, Y):
    """heights [m] on the grid; several reliefs can be added with ';' (e.g. craters:n=6;bumps:n=30)"""
    H = np.zeros_like(X)
    if not spec or spec == 'flat': return H
    rho = np.hypot(X, Y)
    for part in spec.split(';'):
        kind, _, rest = part.partition(':'); o = _opts(rest)
        rng = np.random.default_rng(int(o.get('seed', 1)))
        size = float(np.abs(X).max())
        keep = o.get('flat0', 0.0)                      # flat disc around the spawn point
        if kind == 'flat': continue
        elif kind == 'hills':                           # concentric smooth rings
            A, L, r0, r1 = o.get('A', 0.01), o.get('L', 0.4), o.get('r0', 0.3), o.get('r1', 0.8 * size)
            m = (rho >= r0) & (rho <= r1); h = np.zeros_like(X)
            h[m] = A * 0.5 * (1 - np.cos(2 * np.pi * (rho[m] - r0) / L)); H += h
        elif kind == 'bumps':                           # random Gaussian bumps (positive or both signs)
            n, h0, w = int(o.get('n', 30)), o.get('h', 0.03), o.get('w', 0.2)
            for _ in range(n):
                cx, cy = rng.uniform(-size, size, 2)
                if math.hypot(cx, cy) < keep + 2 * w: continue
                H += h0 * rng.uniform(0.5, 1.0) * (1 if o.get('signed', 0) == 0 else rng.choice([-1, 1])) * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * w * w))
        elif kind == 'craters':                         # bowl + raised rim (Mars-like)
            n, R, D, rim = int(o.get('n', 6)), o.get('R', 0.8), o.get('depth', 0.12), o.get('rim', 0.3)
            for _ in range(n):
                cx, cy = rng.uniform(-0.85 * size, 0.85 * size, 2); Rk = R * rng.uniform(0.6, 1.4)
                if math.hypot(cx, cy) < keep + 1.3 * Rk: continue
                r = np.hypot(X - cx, Y - cy) / Rk
                bowl = np.where(r < 1, -D * Rk / R * (1 - r ** 2), 0.0)
                ring = rim * D * Rk / R * np.exp(-((r - 1.0) / 0.25) ** 2)
                H += bowl + ring
        elif kind == 'waves':                           # parallel sine waves
            A, L, ang = o.get('A', 0.03), o.get('L', 1.0), math.radians(o.get('angle', 0))
            H += A * np.sin(2 * np.pi * (np.cos(ang) * X + np.sin(ang) * Y) / L)
        elif kind == 'ramp':                            # slope along +x from x0 to x1, plateau after
            ang, x0, x1 = math.radians(o.get('angle', 10)), o.get('x0', 0.5), o.get('x1', 0.5 + 0.4 * size)
            H += np.tan(ang) * (np.clip(X, x0, x1) - x0)
        elif kind == 'rough':                           # fractal (sum of random Fourier modes, spectrum ~ 1/k^beta)
            A, L, beta = o.get('A', 0.03), o.get('L', 2.0), o.get('beta', 2.0)
            h = np.zeros_like(X)
            for k in range(1, int(o.get('octaves', 6)) + 1):
                kk = 2 ** (k - 1) / L
                for _ in range(4):
                    a = rng.uniform(0, 2 * np.pi); ph = rng.uniform(0, 2 * np.pi)
                    h += kk ** (-beta / 2) * np.cos(2 * np.pi * kk * (np.cos(a) * X + np.sin(a) * Y) + ph)
            H += A * h / max(h.std(), 1e-9)
        else:
            raise SystemExit(f'unknown relief "{kind}" (flat, hills, bumps, craters, waves, ramp, rough)')
        if keep > 0:
            H *= np.clip((rho - keep) / max(0.5 * keep, 1e-6), 0, 1) ** 2
    return H


# ------------------------------------------------------------------------------------------ floor pattern
def floor(spec, X, Y, default_surface):
    """type index per node + list of surface names"""
    kind, _, rest = (spec or 'uniform').partition(':'); o = _opts(rest)
    S = list(o.get('surfaces', [o.get('surface', default_surface)]))
    rho = np.hypot(X, Y); ang = np.degrees(np.arctan2(Y, X)) % 360.0
    if kind == 'uniform': T = np.zeros_like(X, dtype=int)
    elif kind == 'checkerboard':
        c = o.get('cell', 1.0); T = ((np.floor(X / c) + np.floor(Y / c)).astype(int)) % len(S)
    elif kind in ('voronoi', 'patches', 'voronoi_mosaic'):
        rng = np.random.default_rng(int(o.get('seed', 3))); c = o.get('cell', 1.0); size = float(np.abs(X).max())
        n = max(4, int((2 * size / c) ** 2))
        P = rng.uniform(-size, size, (n, 2)); lab = rng.integers(0, len(S), n)
        from scipy.spatial import cKDTree
        _, idx = cKDTree(P).query(np.c_[X.ravel(), Y.ravel()]); T = lab[idx].reshape(X.shape)
    elif kind == 'stripes':
        w, a = o.get('w', 1.0), math.radians(o.get('angle', 90))
        T = (np.floor((np.cos(a) * X + np.sin(a) * Y) / w).astype(int)) % len(S)
    elif kind == 'rings':
        w, r0 = o.get('w', 0.5), o.get('r0', 0.3); T = np.where(rho < r0, 0, ((rho - r0) // w + 1).astype(int) % len(S))
    elif kind == 'sectors': T = (ang // (360.0 / len(S))).astype(int) % len(S)
    elif kind == 'half':
        a = math.radians(o.get('angle', 0)); T = ((np.cos(a) * X + np.sin(a) * Y) > o.get('offset', 0.0)).astype(int) % len(S)
    else:
        raise SystemExit(f'unknown floor "{kind}" (uniform, checkerboard, voronoi, stripes, rings, sectors, half)')
    if o.get('r0start', 0) > 0: T[rho < o['r0start']] = 0
    return T.astype(int), S


def build_world(relief_spec, floor_spec, size, res, out_dir, default_surface='gray', label=None, mesh_step=2):
    os.makedirs(out_dir, exist_ok=True)
    xs = np.arange(-size, size + 1e-9, res); X, Y = np.meshgrid(xs, xs)
    H = relief(relief_spec, X, Y)
    T, S = floor(floor_spec, X, Y, default_surface)
    dtm = os.path.join(out_dir, 'map.dtm')
    gen_dtm.write_dtm(dtm, xs[0], xs[0], res, H, T, [(s, DUMMY_SOIL) for s in S])
    meshes = []
    for k, s in enumerate(S):
        if not (T == k).any(): continue
        obj = os.path.join(out_dir, f'map_{s}.obj'); write_obj_types(obj, xs, H, T, k, step=mesh_step); meshes.append((obj, s))
    lab = label or '_'.join(x for x in (floor_spec.split(':')[0] if floor_spec else 'uniform', '+'.join(S), (relief_spec or 'flat').split(':')[0]) if x)
    return dict(dtm=dtm, label=lab, surfaces=S, meshes=meshes, size=size, res=res, H=H, T=T, xs=xs)


def height_at(mi, x, y):
    xs = mi['xs']; H = mi['H']; r = mi['res']
    fx, fy = (x - xs[0]) / r, (y - xs[0]) / r
    i, j = int(np.clip(math.floor(fx), 0, len(xs) - 2)), int(np.clip(math.floor(fy), 0, len(xs) - 2))
    u, v = fx - i, fy - j
    return float((1 - u) * (1 - v) * H[j, i] + u * (1 - v) * H[j, i + 1] + (1 - u) * v * H[j + 1, i] + u * v * H[j + 1, i + 1])


# ---- compatibility with the Robotino star runs (scripts/robotino.sh --map)
def map_label(kind, opts, surface=None):
    if kind == 'file': return os.path.splitext(os.path.basename(opts['path']))[0]
    if kind == 'mapfile': return os.path.splitext(os.path.basename(opts['name'].partition('@')[0]))[0]
    if kind == 'hills': return f'{surface}_hills'
    lab = f"{kind}_{'+'.join(opts['surfaces'])}"
    return lab + ('_hills' if opts.get('A', 0) > 0 else '')


def surfaces_of(kind, opts, surface=None):
    if kind == 'file':
        with open(opts['path']) as f: toks = f.read().split()
        nt = int(toks[7]); return [toks[8 + 10 * k] for k in range(nt)]
    if kind == 'hills': return [surface]
    if kind == 'mapfile':
        import maps as MP
        M, _ = MP.load(opts['name']); return MP.compute(M, max(M['resolution'], 0.05))['S']
    return list(opts['surfaces'])


def build(kind, opts, out_dir, surface=None):
    """returns dict(dtm=path, label=str, surfaces=[...], meshes=[(obj, surface)]) — star-experiment maps"""
    os.makedirs(out_dir, exist_ok=True)
    if kind == 'mapfile':                          # configs/maps file (the star starts at the map origin)
        import maps as MP
        M, _ = MP.load(opts['name'])
        mi = MP.build(M, out_dir, res=opts.get('res'))
        return dict(dtm=mi['dtm'], label=mi['label'], surfaces=mi['surfaces'], meshes=mi['meshes'])
    if kind == 'file':
        S = surfaces_of(kind, opts)
        dtm = os.path.abspath(opts['path'])
        obj = os.path.join(out_dir, 'map.obj'); gen_dtm.write_obj(obj, dtm, step=2)
        return dict(dtm=dtm, label=map_label(kind, opts), surfaces=S, meshes=[(obj, S[0])])
    res = opts.get('res', 0.01); size = opts.get('size', 3.0)
    hill = ''
    A = opts.get('A', 0.01 if kind == 'hills' else 0.0)
    if A > 0: hill = f"hills:A={A},L={opts.get('L', 0.4)},r0={opts.get('r0', 0.3)},r1={opts.get('r1', 2.5)}"
    if kind == 'hills': fl = f'uniform:surface={surface}'
    elif kind == 'rings': fl = f"rings:surfaces={'+'.join(opts['surfaces'])},w={opts.get('w', 0.5)},r0={opts.get('r0', 0.3)}"
    elif kind == 'sectors': fl = f"sectors:surfaces={'+'.join(opts['surfaces'])}"
    elif kind == 'patches': fl = f"voronoi:surfaces={'+'.join(opts['surfaces'])},cell={opts.get('cell', 0.4)},seed={int(opts.get('seed', 3))},r0start={opts.get('r0', 0.2)}"
    elif kind in ('checkerboard', 'stripes', 'voronoi', 'half'): fl = kind + ':' + ','.join(f"{k}={'+'.join(v) if isinstance(v, list) else v}" for k, v in opts.items())
    else: raise SystemExit(f'unknown map kind {kind}')
    mi = build_world(hill, fl, size, res, out_dir, default_surface=surface or 'gray', label=map_label(kind, opts, surface))
    return dict(dtm=mi['dtm'], label=mi['label'], surfaces=mi['surfaces'], meshes=mi['meshes'])


def write_obj_types(path, xs, H, T, k, step=2):
    """mesh of the cells whose (lower-left node) type is k"""
    ii = np.arange(0, len(xs), step)
    W = len(ii)
    with open(path, 'w') as f:
        f.write('# terrasim map surface\n')
        for j in ii:
            for i in ii:
                f.write(f'v {xs[i]:.4f} {xs[j]:.4f} {H[j, i]:.4f}\n')
        f.write('vn 0 0 1\n')
        for a in range(W - 1):
            for b in range(W - 1):
                if T[ii[a], ii[b]] != k: continue
                v1 = a * W + b + 1; v2 = v1 + 1; v3 = v1 + W; v4 = v3 + 1
                f.write(f'f {v1}//1 {v2}//1 {v4}//1\nf {v1}//1 {v4}//1 {v3}//1\n')


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument('spec'); ap.add_argument('--out', required=True); ap.add_argument('--surface', default='gray')
    ap.add_argument('--plot', action='store_true')
    a = ap.parse_args()
    k, o = parse_spec(a.spec); r = build(k, o, a.out, a.surface)
    print(r)
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        with open(r['dtm']) as f: toks = f.read().split()
        nx, ny, x0, y0, res, nt = int(toks[2]), int(toks[3]), float(toks[4]), float(toks[5]), float(toks[6]), int(toks[7])
        p = 8 + nt * 10
        H = np.array(toks[p:p + nx * ny], float).reshape(ny, nx); T = np.array(toks[p + nx * ny:p + 2 * nx * ny], int).reshape(ny, nx)
        fig, ax = plt.subplots(1, 2, figsize=(11, 5))
        ext = [x0, x0 + (nx - 1) * res, y0, y0 + (ny - 1) * res]
        im = ax[0].imshow(H * 1000, origin='lower', extent=ext); plt.colorbar(im, ax=ax[0], label='height, mm')
        im = ax[1].imshow(T, origin='lower', extent=ext, cmap='tab10', vmin=0, vmax=9); ax[1].set_title(' / '.join(f'{k}: {s}' for k, s in enumerate(r['surfaces'])))
        for t in np.radians(np.arange(0, 360, 30)): ax[0].plot([0, 2 * np.cos(t)], [0, 2 * np.sin(t)], 'w-', lw=0.5)
        plt.tight_layout(); plt.savefig(os.path.join(a.out, 'map.png'), dpi=80)
