"""Map files: configs/maps/<name>.yaml — bounds, surface zones, relief and ready-made runs (scenarios).

    python3 tools/maps.py list                          # all maps: size, surfaces, heights, max slope, scenarios
    python3 tools/maps.py show slope_15                 # one map: features, scenarios, contact-model check per robot
    python3 tools/maps.py preview all                   # docs/maps/<name>.png + docs/maps/catalog.png
    python3 tools/maps.py preview my_map --out /tmp     # one map
    python3 tools/maps.py check my_map --robot burger,husky
    python3 tools/maps.py new my_map                    # configs/maps/my_map.yaml from the commented template
    python3 tools/maps.py build my_map --out /tmp/m     # map.dtm + Gazebo meshes, without running anything
    python3 tools/maps.py catalog                       # rewrite configs/maps/catalog.yaml from the files

Runs on a map:  bash scripts/drive.sh --map slope_15            (default scenario, the map's robots)
                bash scripts/drive.sh --map slope_15@across --robot husky --engine harmonic --gui
                bash scripts/drive.sh --map "slope_15@*"        (every scenario of the map)
                bash scripts/drive.sh --map slope_15 --map-set relief.0.slope_deg=12   (change a value)

The format (docs/en/MAPS.md has the full reference):

    world:
      name: my_map
      description: ...
      default_surface: gray            # everywhere not covered by a zone
      bounds: {xmin: -4, xmax: 4, ymin: -3, ymax: 3}
      resolution: 0.02                 # node spacing of the height/surface grid, m
      zones:                           # surfaces; later zones overwrite earlier ones
        - {surface: carpet, type: rect, xmin: 0, xmax: 4, ymin: -3, ymax: 3}
        - {surface: sand_lete, type: circle, center: [1, 1], radius: 0.6}
      relief:                          # height features, applied in order (most of them add up)
        - {type: plane, slope_deg: 10, dir_deg: 0}
        - {type: bump_rows, height: 0.03, radius: 0.15, pitch: 0.6, area: [1, 4, -3, 3]}
      surfaces:                        # optional new surfaces: a library surface + changed parameters
        my_sand: {base: sand_lete, mu: 0.51, _color: "0.9 0.8 0.5 1"}
      robots: [husky, jackal]          # robots that drive.sh --map takes when --robot is not given
      scenarios:                       # ready-made runs: spawn [x, y, yaw_deg] + trajectory (tools/trajectories.py)
        across: {spawn: [-3, 0, 0], traj: "line:L=6"}
        tour:   {route: [[-3, -2], [0, 0], [3, 2]]}     # or a route through map points (spline)

What the contact model cannot do (and the check reports): the wheel touches the ground at ONE point under its
centre, on the local tangent plane. Relief must therefore be smooth on the scale of the wheel: bumps much
narrower than the wheel, steps and walls are not seen correctly (a real wheel bridges a narrow hollow and hits
the edge of a step with its rim). `check` estimates the error 0.5*kappa*r^2 of that approximation for every
robot (ok: error <= 5 % of r; approx: <= 25 %, i.e. the terrain is curved less than twice the wheel; invalid above). Rigid obstacles
(walls, boxes) are not supported for the same reason. The chassis touches the ground at the points of its collision shapes
(drive.sh --body-contact): a robot scrapes its belly on a bump, falls and rolls over.
"""
import os, sys, math, json, copy, re, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import gen_dtm

MAP_DIR = os.path.join(ROOT, 'configs', 'maps')
DUMMY_SOIL = dict(kc=0, kphi=0, n0=1, n1=0, c=0, phi_deg=30, K=0.01, Ky=0.01, soil=0)   # the plugin reads [surface X] sections
COLORS = {'gray': '0.55 0.55 0.57 1', 'green': '0.25 0.55 0.25 1', 'table': '0.80 0.68 0.50 1', 'red': '0.75 0.18 0.15 1',
          'table_wet': '0.62 0.55 0.45 1', 'carpet': '0.35 0.35 0.55 1', 'carpet_shag': '0.55 0.45 0.65 1', 'foam': '0.85 0.80 0.35 1',
          'sand_lete': '0.85 0.75 0.55 1', 'sandy_loam': '0.55 0.42 0.30 1', 'clay_wet': '0.45 0.33 0.25 1', 'mars_soil': '0.75 0.45 0.30 1',
          'tile': '0.88 0.90 0.92 1', 'linoleum': '0.70 0.62 0.48 1', 'concrete': '0.62 0.62 0.60 1', 'gray_wet': '0.38 0.40 0.44 1', 'green_wet': '0.15 0.40 0.18 1', 'ice': '0.80 0.90 0.97 1'}
WORLD_KEYS = {'name', 'description', 'tags', 'default_surface', 'bounds', 'resolution', 'collision_resolution', 'zones', 'relief',
              'surfaces', 'surfaces_file', 'robots', 'scenarios', 'notes'}
ZONE_TYPES = ('rect', 'polygon', 'circle', 'ring', 'band', 'halfplane', 'stripes', 'checkerboard', 'voronoi', 'grid')
RELIEF_TYPES = ('plane', 'ramp', 'slopes', 'mound', 'ridges', 'bump', 'hill', 'bump_rows', 'bumps', 'crater', 'craters', 'waves',
                'rough', 'trench', 'berm', 'image', 'flatten', 'grid')


# ------------------------------------------------------------------------------------------------ files
def _yaml():
    try:
        import yaml
        return yaml
    except ImportError:
        raise SystemExit('maps: PyYAML is needed for .yaml maps (sudo apt install python3-yaml  or  pip install pyyaml); '
                         'a map can also be written as .json with the same structure')


def find(name):
    """map name (configs/maps/<name>.yaml) or a path"""
    if os.path.isfile(name): return os.path.abspath(name)
    for ext in ('', '.yaml', '.yml', '.json'):
        p = os.path.join(MAP_DIR, name + ext)
        if os.path.isfile(p): return p
    raise SystemExit(f'map "{name}" not found (maps: python3 tools/maps.py list; own file: give its path)')


def all_names():
    out = []
    for p in sorted(glob.glob(os.path.join(MAP_DIR, '*.yaml')) + glob.glob(os.path.join(MAP_DIR, '*.json'))):
        b = os.path.splitext(os.path.basename(p))[0]
        if b == 'catalog' or b.startswith('_'): continue
        out.append(b)
    return out


def read(path):
    with open(path) as f:
        txt = f.read()
    D = json.loads(txt) if path.endswith('.json') else _yaml().safe_load(txt)
    if not isinstance(D, dict): raise SystemExit(f'{path}: not a map (top level must be a mapping)')
    return D


def _parse_value(v):
    if isinstance(v, str):
        try: return json.loads(v)
        except ValueError:
            try: return float(v)
            except ValueError: return v
    return v


def set_path(W, path, value):
    """--map-set relief.0.slope_deg=12 | scenarios.up.traj=line:L=3 | bounds.xmax=8 | default_surface=carpet"""
    keys = path.split('.'); d = W
    for k in keys[:-1]:
        if isinstance(d, list): d = d[int(k)]
        else:
            if k not in d: raise SystemExit(f'--map-set {path}: no "{k}" in the map')
            d = d[k]
    k = keys[-1]
    if isinstance(d, list): d[int(k)] = _parse_value(value)
    else: d[k] = _parse_value(value)


def load(spec, overrides=()):
    """'name' | 'name@scenario' | 'path.yaml@scenario' -> (map dict, scenario name or None or '*')"""
    name, _, scen = spec.partition('@')
    path = find(name)
    raw = read(path)
    W = copy.deepcopy(raw.get('world', raw))
    for ov in overrides:
        k, _, v = ov.partition('=')
        if not _: raise SystemExit(f'--map-set {ov}: expected key=value')
        set_path(W, k.strip(), v.strip())
    if 'obstacles' in W:
        raise SystemExit(f'{path}: "obstacles" (walls, boxes) are not supported: the wheel contact is a point under the wheel centre '
                         'on the local tangent plane, it cannot hit a vertical edge (docs/en/MAPS.md, "Limitations"). '
                         'Use smooth relief (bump, berm, ridges, mound) or a surface zone instead.')
    unknown = set(W) - WORLD_KEYS
    if unknown: raise SystemExit(f'{path}: unknown keys {sorted(unknown)} (allowed: {sorted(WORLD_KEYS)})')
    b = W.get('bounds', {})
    try: xmin, xmax, ymin, ymax = (float(b[k]) for k in ('xmin', 'xmax', 'ymin', 'ymax'))
    except (KeyError, TypeError, ValueError): raise SystemExit(f'{path}: bounds needs xmin, xmax, ymin, ymax')
    if not (xmin < xmax and ymin < ymax): raise SystemExit(f'{path}: empty bounds')
    M = dict(name=W.get('name') or os.path.splitext(os.path.basename(path))[0], file=path, dir=os.path.dirname(path),
             description=W.get('description', ''), tags=list(W.get('tags', [])), default_surface=W.get('default_surface', 'gray'),
             bounds=(xmin, xmax, ymin, ymax), resolution=float(W.get('resolution', W.get('collision_resolution', 0.02))),
             zones=list(W.get('zones') or []), relief=list(W.get('relief') or []), surfaces=dict(W.get('surfaces') or {}),
             surfaces_file=W.get('surfaces_file'), robots=list(W.get('robots') or []), scenarios=dict(W.get('scenarios') or {}),
             notes=W.get('notes', ''))
    if M['resolution'] <= 0: raise SystemExit(f'{path}: resolution must be > 0')
    for z in M['zones']:
        if z.get('type', 'rect') not in ZONE_TYPES: raise SystemExit(f"{path}: zone type {z.get('type')} (allowed: {', '.join(ZONE_TYPES)})")
    for f in M['relief']:
        if f.get('type') not in RELIEF_TYPES: raise SystemExit(f"{path}: relief type {f.get('type')} (allowed: {', '.join(RELIEF_TYPES)})")
    if scen and scen != '*' and scen not in M['scenarios']:
        raise SystemExit(f"map {M['name']}: no scenario '{scen}' (scenarios: {', '.join(M['scenarios']) or 'none'})")
    if not scen: scen = next(iter(M['scenarios']), None)
    return M, scen


def custom_surfaces(M):
    """{name: params} defined by the map (surfaces:, surfaces_file:) — same format as drive.sh --surface-file"""
    S = {}
    if M.get('surfaces_file'):
        p = M['surfaces_file'] if os.path.isabs(M['surfaces_file']) else os.path.join(M['dir'], M['surfaces_file'])
        X = json.load(open(p)) if p.endswith('.json') else _yaml().safe_load(open(p))
        S.update(X.get('surfaces', X))
    S.update(M['surfaces'])
    return S


def resolve_surfaces(M, lib):
    """library dict + the map's own surfaces (base + overrides); returns a new dict"""
    S = copy.deepcopy(lib)
    for name, par in custom_surfaces(M).items():
        if name.startswith('_'): continue
        if 'base' in par and par['base'] not in S: raise SystemExit(f"map {M['name']}: surface {name}: base '{par['base']}' is not in the library")
        base = copy.deepcopy(S[par['base']]) if 'base' in par else {}
        base.update({k: v for k, v in par.items() if k != 'base'}); base.setdefault('_kind', 'map'); S[name] = base
    return S


def colors(M):
    C = dict(COLORS)
    for name, par in custom_surfaces(M).items():
        C[name] = par.get('_color', C.get(par.get('base', ''), '0.5 0.5 0.5 1'))
    return C


# ------------------------------------------------------------------------------------------------ scenarios
def scenario(M, name):
    """-> dict(spawn=(x, y, yaw_deg), traj=spec, speed, robots, note); a 'route' of map points becomes a spline"""
    if name is None: return dict(spawn=(0.0, 0.0, 0.0), traj='', speed=0.0, robots=[], note='')
    S = dict(M['scenarios'][name])
    if 'route' in S:
        P = np.array(S['route'], float)
        if len(P) < 2: raise SystemExit(f"map {M['name']}@{name}: route needs at least 2 points")
        x0, y0 = P[0]
        yaw = S['spawn'][2] if 'spawn' in S and len(S['spawn']) > 2 else math.degrees(math.atan2(P[1, 1] - y0, P[1, 0] - x0))
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        loc = [((px - x0) * c + (py - y0) * s, -(px - x0) * s + (py - y0) * c) for px, py in P[1:]]
        traj = 'spline:pts=' + ';'.join(f'{u:.3f},{v:.3f}' for u, v in loc) if len(loc) > 1 else f'line:L={math.hypot(*loc[0]):.3f}'
        spawn = (float(x0), float(y0), float(yaw))
    else:
        sp = list(S.get('spawn', [0, 0, 0])) + [0, 0, 0]
        spawn = (float(sp[0]), float(sp[1]), float(sp[2])); traj = S.get('traj', '')
    return dict(spawn=spawn, traj=traj, speed=float(S.get('speed', 0) or 0), robots=list(S.get('robots', [])), note=S.get('note', ''))


def route_world(sc):
    """reference path of a scenario in map coordinates (N x 2)"""
    import trajectories as TJ
    if not sc['traj']: return np.zeros((0, 2))
    A = TJ.build(sc['traj'])
    x0, y0, yaw = sc['spawn']; c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return np.c_[x0 + c * A[:, 1] - s * A[:, 2], y0 + s * A[:, 1] + c * A[:, 2]]


# ------------------------------------------------------------------------------------------------ geometry helpers
def _uv(f, X, Y):
    d = math.radians(float(f.get('dir_deg', 0.0))); x0, y0 = (float(v) for v in f.get('origin', [0.0, 0.0]))
    return (X - x0) * math.cos(d) + (Y - y0) * math.sin(d), -(X - x0) * math.sin(d) + (Y - y0) * math.cos(d)


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0); return t * t * (3 - 2 * t)


def _soft_int(x, b):
    """integral of a slope that switches on smoothly (cubic smoothstep over length b) at x = 0: 0 before, x after"""
    if b <= 0: return np.maximum(x, 0.0)
    t = np.clip(x / b + 0.5, 0.0, 1.0)
    return np.where(x >= 0.5 * b, x, b * (t ** 3 - 0.5 * t ** 4))


def _bell(r, R):
    """cosine bell: 1 at r = 0, 0 (with zero slope) at r >= R"""
    return np.where(r < R, 0.5 * (1 + np.cos(np.pi * np.minimum(r, R) / R)), 0.0)


def _area_mask(area, X, Y):
    if area is None: return np.ones(X.shape, bool)
    if isinstance(area, dict): x0, x1, y0, y1 = area['xmin'], area['xmax'], area['ymin'], area['ymax']
    else: x0, x1, y0, y1 = area
    e = 1e-9
    return (X >= x0 - e) & (X <= x1 + e) & (Y >= y0 - e) & (Y <= y1 + e)


def _area_box(area, bounds):
    if area is None: return bounds
    if isinstance(area, dict): return area['xmin'], area['xmax'], area['ymin'], area['ymax']
    return tuple(area)


def _polyline_dist(P, X, Y):
    P = np.asarray(P, float); d = np.full(X.shape, np.inf)
    for (ax, ay), (bx, by) in zip(P[:-1], P[1:]):
        vx, vy = bx - ax, by - ay; L2 = vx * vx + vy * vy
        t = np.clip(((X - ax) * vx + (Y - ay) * vy) / max(L2, 1e-12), 0, 1)
        d = np.minimum(d, np.hypot(X - ax - t * vx, Y - ay - t * vy))
    return d


def _in_polygon(P, X, Y):
    from matplotlib.path import Path
    return Path(np.asarray(P, float)).contains_points(np.c_[X.ravel(), Y.ravel()], radius=1e-9).reshape(X.shape)


def _avoid_ok(cx, cy, R, avoid):
    return all(math.hypot(cx - a[0], cy - a[1]) >= a[2] + R for a in (avoid or []))


# ------------------------------------------------------------------------------------------------ relief
def relief(M, X, Y):
    H = np.zeros_like(X)
    for k, f in enumerate(M['relief']):
        t = f['type']; g = lambda key, d=None: f.get(key, d)
        if t == 'plane':
            u, _ = _uv(f, X, Y); H += math.tan(math.radians(g('slope_deg', 10))) * u
        elif t in ('ramp', 'slopes'):
            u, _ = _uv(f, X, Y)
            if t == 'ramp': segs, start = [dict(slope_deg=g('slope_deg', 10), length=g('to', 3.0) - g('from', 0.0))], g('from', 0.0)
            else: segs, start = g('segments'), g('start', 0.0)
            b = float(g('blend', 0.3)); prev = 0.0; uk = float(start)
            for sg in segs:
                s = math.tan(math.radians(float(sg['slope_deg'])))
                H += (s - prev) * _soft_int(u - uk, b); prev = s; uk += float(sg['length'])
            if g('then', 'flat') == 'flat': H -= prev * _soft_int(u - uk, b)
        elif t == 'mound':                            # MarsSim Fig. 15 sandy mound (smooth trapezoid), across the map
            u, _ = _uv(f, X, Y)
            uu = np.arange(u.min() - 1.0, u.max() + 1.0, min(M['resolution'], 0.01))
            prof = gen_dtm.mound_profile(uu, g('x_start', 1.5), g('L_up', 0.8), g('L_top', 0.3), g('L_down', 0.8), g('height', 0.14), g('smooth', 0.1))
            H += np.interp(u, uu, prof)
        elif t == 'ridges':                           # parallel transverse humps ("rows of bumps" across the path)
            u, _ = _uv(f, X, Y)
            h, w, p = float(g('height', 0.03)), float(g('width', 0.25)), float(g('pitch', 0.6))
            u0, u1 = float(g('from', 0.0)), float(g('to', 1e9)); n = int(g('count', 10 ** 6))
            c = u0 + 0.5 * w; i = 0
            while c + 0.5 * w <= u1 + 1e-9 and i < n:
                H += h * _bell(np.abs(u - c), 0.5 * w); c += p; i += 1
        elif t in ('bump', 'hill'):
            cx, cy = g('center', [0, 0]); H += float(g('height', 0.05)) * _bell(np.hypot(X - cx, Y - cy), float(g('radius', 0.3)))
        elif t == 'bump_rows':                        # lattice of round bumps (or pits), optional staggered rows
            u, v = _uv(f, X, Y)
            h, R = float(g('height', 0.03)), float(g('radius', 0.15))
            pu, pv = (g('pitch', 0.5),) * 2 if not isinstance(g('pitch', 0.5), (list, tuple)) else g('pitch')
            u0, u1, v0, v1 = (float(a) for a in g('area', [-2, 2, -2, 2]))      # in the rotated frame (u along dir_deg)
            rng = np.random.default_rng(int(g('seed', 1))); jit = float(g('jitter', 0.0)); sign = g('sign', 'up')
            row = 0; uc = u0 + R
            while uc <= u1 - R + 1e-9:
                vc = v0 + R + (0.5 * pv if (g('stagger', True) and row % 2) else 0.0)
                while vc <= v1 - R + 1e-9:
                    hk = h * (1 - jit * rng.uniform()) * (-1 if sign == 'down' else (rng.choice([-1, 1]) if sign == 'mixed' else 1))
                    m = (np.abs(u - uc) < R) & (np.abs(v - vc) < R)
                    if m.any(): H[m] += hk * _bell(np.hypot(u[m] - uc, v[m] - vc), R)
                    vc += pv
                uc += pu; row += 1
        elif t in ('bumps', 'craters'):               # random field (seeded)
            rng = np.random.default_rng(int(g('seed', 1)))
            xa, xb, ya, yb = _area_box(g('area'), M['bounds']); n = int(g('n', 20)); tries = 0; placed = 0
            rr = g('radius', 0.3 if t == 'bumps' else [0.5, 1.2]); rlo, rhi = (rr, rr) if not isinstance(rr, (list, tuple)) else rr
            while placed < n and tries < 50 * n:
                tries += 1; R = rng.uniform(rlo, rhi); cx, cy = rng.uniform(xa + R, xb - R), rng.uniform(ya + R, yb - R)
                if not _avoid_ok(cx, cy, R, g('avoid')): continue
                r = np.hypot(X - cx, Y - cy); placed += 1
                if t == 'bumps':
                    hk = float(g('height', 0.05)) * rng.uniform(0.5, 1.0) * (rng.choice([-1, 1]) if g('signed', False) else 1)
                    H += hk * _bell(r, R)
                else:
                    D = float(g('depth_ratio', 0.15)) * R; rim = float(g('rim', 0.25)); rw = float(g('rim_width', 0.3)) * R
                    H += -D * _bell(r, R) + rim * D * np.exp(-((r - R) / rw) ** 2)
        elif t == 'crater':
            cx, cy = g('center', [0, 0]); R = float(g('radius', 1.5)); D = float(g('depth', 0.3))
            r = np.hypot(X - cx, Y - cy); rw = float(g('rim_width', 0.3)) * R
            H += -D * _bell(r, R) + float(g('rim', 0.25)) * D * np.exp(-((r - R) / rw) ** 2)
        elif t == 'waves':
            u, _ = _uv(f, X, Y); A, L = float(g('amplitude', 0.03)), float(g('wavelength', 1.0))
            w = np.ones_like(u)
            if g('from') is not None: w = w * _smoothstep((u - float(g('from'))) / L)
            if g('to') is not None: w = w * (1 - _smoothstep((u - float(g('to'))) / L + 1))
            H += A * np.sin(2 * np.pi * u / L) * w
        elif t == 'rough':                            # fractal: random Fourier modes, spectrum ~ 1/k^beta, rms = amplitude
            rng = np.random.default_rng(int(g('seed', 1))); A, L, beta = float(g('amplitude', 0.03)), float(g('wavelength', 2.0)), float(g('beta', 2.0))
            h = np.zeros_like(X)
            for o in range(1, int(g('octaves', 5)) + 1):
                kk = 2 ** (o - 1) / L
                for _ in range(4):
                    a = rng.uniform(0, 2 * np.pi); ph = rng.uniform(0, 2 * np.pi)
                    h += kk ** (-beta / 2) * np.cos(2 * np.pi * kk * (np.cos(a) * X + np.sin(a) * Y) + ph)
            h *= A / max(h.std(), 1e-12)
            if g('area') is not None: h *= _area_mask(g('area'), X, Y)
            H += h
        elif t in ('trench', 'berm'):                 # along a polyline: cosine cross-section, optional flat bottom/top
            d = _polyline_dist(g('points'), X, Y); w = float(g('width', 0.5)); fl = float(g('flat', 0.0))
            h = float(g('height', -0.2 if t == 'trench' else 0.2))
            H += h * np.where(d <= fl, 1.0, _bell(d - fl, w))
        elif t == 'image':                            # grayscale picture: black = 0, white = height
            import matplotlib.image as mpimg
            from scipy.ndimage import map_coordinates, gaussian_filter
            p = g('file'); p = p if os.path.isabs(p) else os.path.join(M['dir'], p)
            I = mpimg.imread(p).astype(float)
            if I.ndim == 3: I = I[..., :3].mean(axis=2)
            if I.max() > 1.0: I /= 255.0
            if g('invert', False): I = 1 - I
            if g('blur', 0): I = gaussian_filter(I, float(g('blur')))
            xa, xb, ya, yb = _area_box(g('area'), M['bounds']); ny_, nx_ = I.shape
            ci = (X - xa) / (xb - xa) * (nx_ - 1); cj = (yb - Y) / (yb - ya) * (ny_ - 1)       # image row 0 = top = ymax
            v = map_coordinates(I, [cj.ravel(), ci.ravel()], order=1, mode='nearest').reshape(X.shape)
            H += float(g('height', 0.2)) * v * _area_mask((xa, xb, ya, yb), X, Y)
        elif t == 'grid':                             # height field from a file: .npz with H (ny x nx), x0, y0, res (dataset terrain)
            from scipy.ndimage import map_coordinates
            p = g('file'); p = p if os.path.isabs(p) else os.path.join(M['dir'], p)
            Z = np.load(p); Hg = Z['H'].astype(float); x0g, y0g, rg = float(Z['x0']), float(Z['y0']), float(Z['res'])
            ci = (X - x0g) / rg; cj = (Y - y0g) / rg
            H += float(g('scale', 1.0)) * map_coordinates(Hg, [cj.ravel(), ci.ravel()], order=1, mode='nearest').reshape(X.shape)
        elif t == 'flatten':                          # level pad (start area) blended into the surroundings
            cx, cy = g('center', [0, 0]); R = float(g('radius', 0.5)); bl = float(g('blend', 0.5 * R))
            j = int(np.clip(round((cy - Y[0, 0]) / M['_res']), 0, X.shape[0] - 1)); i = int(np.clip(round((cx - X[0, 0]) / M['_res']), 0, X.shape[1] - 1))
            lev = float(g('level', H[j, i])) if g('level', 'center') != 'center' else float(H[j, i])
            w = _smoothstep((np.hypot(X - cx, Y - cy) - R) / max(bl, 1e-6))
            H = lev + (H - lev) * w
    return H


# ------------------------------------------------------------------------------------------------ zones
def zones(M, X, Y):
    S = [M['default_surface']]; T = np.zeros(X.shape, int)

    def idx(s):
        if s not in S: S.append(s)
        return S.index(s)
    for z in M['zones']:
        t = z.get('type', 'rect'); name = z.get('surface', z.get('name'))
        lim = _area_mask(z.get('area'), X, Y)
        if t == 'grid':                       # class raster from a file: .npz with L (ny x nx int), x0, y0, res; classes -> surfaces
            p = z['file']; p = p if os.path.isabs(p) else os.path.join(M['dir'], p)
            Z = np.load(p); Lg = Z['L']; x0g, y0g, rg = float(Z['x0']), float(Z['y0']), float(Z['res'])
            ci = np.clip(np.round((X - x0g) / rg).astype(int), 0, Lg.shape[1] - 1); cj = np.clip(np.round((Y - y0g) / rg).astype(int), 0, Lg.shape[0] - 1)
            lab = Lg[cj, ci]
            for cls, surf in z['surfaces'].items():
                T = np.where(lim & (lab == int(cls)), idx(surf), T)
            continue
        if t in ('stripes', 'checkerboard', 'voronoi'):
            surf = z['surfaces']; ids = np.array([idx(s) for s in surf])
            if t == 'stripes':
                u, _ = _uv(z, X, Y); lab = np.floor(u / float(z.get('width', 1.0))).astype(int) % len(surf)
            elif t == 'checkerboard':
                x0, y0 = z.get('origin', [0, 0]); c = float(z.get('cell', 1.0))
                lab = (np.floor((X - x0) / c) + np.floor((Y - y0) / c)).astype(int) % len(surf)
            else:
                from scipy.spatial import cKDTree
                rng = np.random.default_rng(int(z.get('seed', 3))); xa, xb, ya, yb = _area_box(z.get('area'), M['bounds'])
                c = float(z.get('cell', 1.0)); n = max(4, int((xb - xa) * (yb - ya) / c ** 2))
                P = np.c_[rng.uniform(xa, xb, n), rng.uniform(ya, yb, n)]; L = rng.integers(0, len(surf), n)
                _, k = cKDTree(P).query(np.c_[X.ravel(), Y.ravel()]); lab = L[k].reshape(X.shape)
            T = np.where(lim, ids[lab], T); continue
        if name is None: raise SystemExit(f"map {M['name']}: zone {z} has no surface")
        e = 1e-9
        if t == 'rect': m = (X >= z['xmin'] - e) & (X <= z['xmax'] + e) & (Y >= z['ymin'] - e) & (Y <= z['ymax'] + e)
        elif t == 'polygon': m = _in_polygon(z['points'], X, Y)
        elif t == 'circle': m = np.hypot(X - z['center'][0], Y - z['center'][1]) <= z['radius']
        elif t == 'ring':
            r = np.hypot(X - z['center'][0], Y - z['center'][1]); m = (r >= z['r_in']) & (r <= z['r_out'])
        elif t == 'band': m = _polyline_dist(z['points'], X, Y) <= 0.5 * float(z['width'])
        elif t == 'halfplane':
            u, _ = _uv(z, X, Y); m = u >= float(z.get('offset', 0.0))
        T = np.where(m & lim, idx(name), T)
    return T, S


# ------------------------------------------------------------------------------------------------ build
def compute(M, res=None):
    res = float(res or M['resolution']); M['_res'] = res
    xmin, xmax, ymin, ymax = M['bounds']
    xs = xmin + res * np.arange(int(math.floor((xmax - xmin) / res + 1e-6)) + 1)
    ys = ymin + res * np.arange(int(math.floor((ymax - ymin) / res + 1e-6)) + 1)
    if len(xs) * len(ys) > 6_000_000:
        raise SystemExit(f"map {M['name']}: {len(xs)} x {len(ys)} nodes at {res} m — too many; use a coarser resolution (--res)")
    X, Y = np.meshgrid(xs, ys)
    H = relief(M, X, Y); T, S = zones(M, X, Y)
    used = sorted(set(np.unique(T)))           # drop surfaces that no node uses (fully overwritten zones)
    remap = {o: n for n, o in enumerate(used)}; T = np.vectorize(remap.get)(T) if len(used) < len(S) else T
    S = [S[o] for o in used]
    return dict(xs=xs, ys=ys, H=H, T=T.astype(int), S=S, res=res)


def write_meshes(out_dir, G, step, names, max_tris=600_000):
    """one OBJ per surface (cells by their lower-left node), smooth vertex normals; only the used vertices"""
    xs, ys, H, T = G['xs'], G['ys'], G['H'], G['T']
    while ((len(xs) - 1) // step) * ((len(ys) - 1) // step) * 2 > max_tris: step += 1
    ii = np.arange(0, len(xs), step); jj = np.arange(0, len(ys), step)
    if ii[-1] != len(xs) - 1: ii = np.r_[ii, len(xs) - 1]
    if jj[-1] != len(ys) - 1: jj = np.r_[jj, len(ys) - 1]
    Hs = H[np.ix_(jj, ii)]; Ts = T[np.ix_(jj, ii)]
    gy, gx = np.gradient(Hs, ys[jj], xs[ii])
    N = np.dstack([-gx, -gy, np.ones_like(Hs)]); N /= np.linalg.norm(N, axis=2, keepdims=True)
    Xs, Ys = np.meshgrid(xs[ii], ys[jj]); nyi, nxi = Hs.shape
    out = []
    for k, s in enumerate(names):
        cell = Ts[:-1, :-1] == k
        if not cell.any(): continue
        used = np.zeros(Hs.shape, bool)
        used[:-1, :-1] |= cell; used[1:, :-1] |= cell; used[:-1, 1:] |= cell; used[1:, 1:] |= cell
        vid = np.full(Hs.shape, -1, int); vid[used] = np.arange(1, used.sum() + 1)
        jc, ic = np.nonzero(cell)
        v1, v2, v3, v4 = vid[jc, ic], vid[jc, ic + 1], vid[jc + 1, ic], vid[jc + 1, ic + 1]
        p = os.path.join(out_dir, f'map_{s}.obj')
        with open(p, 'w') as f:
            f.write(f'# terrasim map surface {s}\n')
            V = np.c_[Xs[used], Ys[used], Hs[used]]; NN = N[used]
            f.write('\n'.join(f'v {a:.4f} {b:.4f} {c:.4f}' for a, b, c in V) + '\n')
            f.write('\n'.join(f'vn {a:.4f} {b:.4f} {c:.4f}' for a, b, c in NN) + '\n')
            F = np.r_[np.c_[v1, v2, v4], np.c_[v1, v4, v3]]
            f.write('\n'.join(f'f {a}//{a} {b}//{b} {c}//{c}' for a, b, c in F) + '\n')
        out.append((p, s))
    return out


def build(M, out_dir, res=None, mesh_step=None, meshes=True):
    """map.dtm + meshes; returns the map-info dict used by drive_run / robotino_run"""
    os.makedirs(out_dir, exist_ok=True)
    G = compute(M, res)
    dtm = os.path.join(out_dir, 'map.dtm')
    gen_dtm.write_dtm(dtm, G['xs'][0], G['ys'][0], G['res'], G['H'], G['T'], [(s, DUMMY_SOIL) for s in G['S']])
    ms = write_meshes(out_dir, G, mesh_step or max(1, int(round(0.04 / G['res']))), G['S']) if meshes else []
    xmin, xmax, ymin, ymax = M['bounds']
    return dict(dtm=dtm, label=M['name'], surfaces=G['S'], meshes=ms, size=max(abs(xmin), abs(xmax), abs(ymin), abs(ymax)), res=G['res'],
                H=G['H'], T=G['T'], xs=G['xs'], ys=G['ys'], bounds=M['bounds'], colors=colors(M), name=M['name'], file=M['file'])


# ------------------------------------------------------------------------------------------------ analysis helpers
def slope_deg(G):
    gy, gx = np.gradient(G['H'], G['ys'], G['xs'])
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def height_at(G, x, y):
    xs, ys, H = G['xs'], G['ys'], G['H']
    fx = np.clip((np.asarray(x) - xs[0]) / (xs[1] - xs[0]), 0, len(xs) - 1.000001)
    fy = np.clip((np.asarray(y) - ys[0]) / (ys[1] - ys[0]), 0, len(ys) - 1.000001)
    i, j = np.floor(fx).astype(int), np.floor(fy).astype(int); u, v = fx - i, fy - j
    return (1 - u) * (1 - v) * H[j, i] + u * (1 - v) * H[j, i + 1] + (1 - u) * v * H[j + 1, i] + u * v * H[j + 1, i + 1]


def contact_error(G, r):
    """height error of the single-point contact, 0.5*kappa*r^2 (kappa: largest principal curvature)"""
    xs, ys, H = G['xs'], G['ys'], G['H']
    gy, gx = np.gradient(H, ys, xs); hyy, hyx = np.gradient(gy, ys, xs); hxy, hxx = np.gradient(gx, ys, xs)
    m = 0.5 * (hxx + hyy); d = np.sqrt((0.5 * (hxx - hyy)) ** 2 + (0.5 * (hxy + hyx)) ** 2)
    k = np.maximum(np.abs(m + d), np.abs(m - d)) / (1 + gx ** 2 + gy ** 2) ** 1.5
    return 0.5 * k * r * r


def check(M, G, robots):
    """per robot: wheel radius, max single-point contact error and verdict; scenario routes inside the map"""
    import robots as RB
    rows = []
    for rn in robots:
        R = RB.robot(rn); r = max(w['r'] for w in R['wheels'] if w['type'] != 'caster')
        e = contact_error(G, r); emax = float(np.percentile(e, 99.9)); bad = float((e > 0.05 * r).mean())
        verdict = 'ok' if emax <= 0.05 * r else ('approx' if emax <= 0.25 * r else 'invalid')   # 0.5*kappa*r/r: kappa*r <= 0.1 | <= 0.5
        rows.append(dict(robot=rn, wheel_r_mm=round(r * 1000, 1), err_max_mm=round(emax * 1000, 2), err_over_r=round(emax / r, 3),
                         area_over_5pct=round(bad, 4), verdict=verdict))
    out = []
    xmin, xmax, ymin, ymax = M['bounds']
    for name in M['scenarios']:
        sc = scenario(M, name); P = route_world(sc)
        if len(P):
            inside = (P[:, 0] >= xmin) & (P[:, 0] <= xmax) & (P[:, 1] >= ymin) & (P[:, 1] <= ymax)
            margin = min(P[:, 0].min() - xmin, xmax - P[:, 0].max(), P[:, 1].min() - ymin, ymax - P[:, 1].max())
            out.append(dict(scenario=name, length_m=round(float(np.hypot(*np.diff(P, axis=0).T).sum()), 2), inside=bool(inside.all()),
                            margin_m=round(float(margin), 2)))
    return rows, out


def summary(M, G):
    sl = slope_deg(G)
    return dict(name=M['name'], size=f"{M['bounds'][1] - M['bounds'][0]:g} x {M['bounds'][3] - M['bounds'][2]:g} m",
                surfaces='+'.join(G['S']), h_mm=f"{G['H'].min() * 1000:.0f}..{G['H'].max() * 1000:.0f}", slope_max_deg=round(float(sl.max()), 1),
                scenarios=','.join(M['scenarios']), robots=','.join(M['robots']), description=M['description'])


# ------------------------------------------------------------------------------------------------ pictures
def _rgb(G, C):
    return np.array([[float(v) for v in C.get(s, '0.6 0.6 0.6 1').split()[:3]] for s in G['S']])[G['T']]


def _draw_map(ax, M, G, C, scen_names=None, small=False):
    from matplotlib.colors import LightSource
    xs, ys, H = G['xs'], G['ys'], G['H']
    rgb = _rgb(G, C); rel = H.max() - H.min()
    if rel > 1e-4:
        ls = LightSource(azdeg=315, altdeg=45)
        ve = min(10.0, max(1.0, 0.15 * max(xs[-1] - xs[0], ys[-1] - ys[0]) / rel))
        rgb = ls.shade_rgb(rgb, H, vert_exag=ve, dx=xs[1] - xs[0], dy=ys[1] - ys[0], blend_mode='soft')
    ax.imshow(rgb, origin='lower', extent=[xs[0], xs[-1], ys[0], ys[-1]], interpolation='bilinear')
    if rel > 1e-4:
        lev = np.linspace(H.min(), H.max(), 12)[1:-1]
        cs = ax.contour(xs, ys, H * 1000, levels=lev * 1000, colors='k', linewidths=0.35, alpha=0.55)
        if not small: ax.clabel(cs, fontsize=6, fmt='%.0f')
    cols = ['#d62728', '#1f77b4', '#ff7f0e', '#9467bd', '#2ca02c', '#e377c2', '#17becf', '#8c564b']
    for k, name in enumerate(scen_names if scen_names is not None else M['scenarios']):
        sc = scenario(M, name); P = route_world(sc)
        if not len(P): continue
        c = cols[k % len(cols)]
        ax.plot(P[:, 0], P[:, 1], '-', color=c, lw=1.6 if not small else 1.2, label=None if small else f"{name}: {sc['traj'][:38]}")
        x0, y0, yaw = sc['spawn']; L = 0.04 * max(xs[-1] - xs[0], ys[-1] - ys[0])
        ax.annotate('', (x0 + L * math.cos(math.radians(yaw)), y0 + L * math.sin(math.radians(yaw))), (x0, y0),
                    arrowprops=dict(arrowstyle='->', color=c, lw=1.4))
        ax.plot(x0, y0, 'o', color=c, ms=4 if not small else 3)
    ax.set_xlim(xs[0], xs[-1]); ax.set_ylim(ys[0], ys[-1]); ax.set_aspect('equal')


def preview(M, G, path):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    C = colors(M); sl = slope_deg(G)
    fig = plt.figure(figsize=(15, 6.6)); gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], height_ratios=[1, 1])
    ax = fig.add_subplot(gs[:, 0]); _draw_map(ax, M, G, C)
    h, l = ax.get_legend_handles_labels()
    h += [Patch(color=_rgb(dict(S=[s], T=np.zeros((1, 1), int)), C)[0, 0]) for s in G['S']]; l += G['S']
    ax.legend(h, l, fontsize=7, loc='upper left', framealpha=0.85); ax.set_xlabel('x, m'); ax.set_ylabel('y, m'); ax.grid(alpha=0.2)
    ax.set_title(f"{M['name']}: {M['description']}\n{M['bounds'][1] - M['bounds'][0]:g} × {M['bounds'][3] - M['bounds'][2]:g} m, "
                 f"heights {G['H'].min() * 1000:.0f}…{G['H'].max() * 1000:.0f} mm, max slope {sl.max():.1f}° (contours: height, mm)", fontsize=9)
    a2 = fig.add_subplot(gs[0, 1]); a3 = fig.add_subplot(gs[1, 1])
    name = next(iter(M['scenarios']), None)
    if name:
        sc = scenario(M, name); P = route_world(sc)
        s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
        z = height_at(G, P[:, 0], P[:, 1])
        fx = np.clip(np.round((P[:, 0] - G['xs'][0]) / G['res']).astype(int), 0, len(G['xs']) - 1)
        fy = np.clip(np.round((P[:, 1] - G['ys'][0]) / G['res']).astype(int), 0, len(G['ys']) - 1)
        surf = G['T'][fy, fx]; rgb = _rgb(G, C)[fy, fx]
        a2.scatter(s, z * 1000, c=rgb, s=6); a2.plot(s, z * 1000, 'k-', lw=0.6)
        a2.set_title(f"profile along scenario '{name}' ({sc['traj'][:50]}), colour = surface", fontsize=9)
        a2.set_ylabel('height, mm'); a2.grid(alpha=0.3)
        # slope along the path (pitch for the robot) and across it (roll)
        tx, ty = np.gradient(P[:, 0]), np.gradient(P[:, 1]); tn = np.hypot(tx, ty) + 1e-12; tx, ty = tx / tn, ty / tn
        gy, gx = np.gradient(G['H'], G['ys'], G['xs'])
        gxp, gyp = gx[fy, fx], gy[fy, fx]
        a3.plot(s, np.degrees(np.arctan(gxp * tx + gyp * ty)), 'r-', lw=1, label='along the path (pitch; + uphill)')
        a3.plot(s, np.degrees(np.arctan(-gxp * ty + gyp * tx)), 'b-', lw=1, label='across the path (roll; + left side higher)')
        a3.set_ylabel('slope, °'); a3.set_xlabel('path, m'); a3.legend(fontsize=7); a3.grid(alpha=0.3)
    else:
        a2.axis('off'); a3.axis('off')
    plt.tight_layout(); plt.savefig(path, dpi=80); plt.close(fig)


def sheet(items, path, ncol=6):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    n = len(items); nrow = int(math.ceil(n / ncol))
    fig, axs = plt.subplots(nrow, ncol, figsize=(3.1 * ncol, 3.25 * nrow))
    for k, ax in enumerate(np.ravel(axs)):
        if k >= n: ax.axis('off'); continue
        M, G = items[k]; _draw_map(ax, M, G, colors(M), small=True)
        ax.set_title(f"{M['name']}\n{'+'.join(G['S'])[:42]}", fontsize=8); ax.set_xticks([]); ax.set_yticks([])
    plt.tight_layout(); plt.savefig(path, dpi=70); plt.close(fig)


# ------------------------------------------------------------------------------------------------ CLI
TEMPLATE = os.path.join(MAP_DIR, '_template.yaml')


def results_figure(summary_csv, out):
    """progress along the path per scenario (map@scenario) x robot of a drive.sh map study; T = tipped, L = left the map"""
    import pandas as pd, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    S = pd.read_csv(summary_csv)
    order = [r for r in ('burger', 'waffle', 'robotino', 'youbot', 'pioneer', 'jackal', 'husky', 'rover') if r in set(S.robot)]
    order += [r for r in dict.fromkeys(S.robot) if r not in order]
    P = S.pivot_table(index='scenario', columns='robot', values='progress', aggfunc='first').reindex(columns=order).sort_index()
    St = S.pivot_table(index='scenario', columns='robot', values='status', aggfunc='first').reindex(index=P.index, columns=order)
    fig, ax = plt.subplots(figsize=(8.6, 0.25 * len(P) + 1.0))
    ax.imshow(np.ma.masked_invalid(P.values.astype(float)), cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
    for i in range(P.shape[0]):
        for j in range(P.shape[1]):
            v = P.values[i, j]
            if np.isnan(v): continue
            st = St.values[i, j]; tag = ' T' if st == 'tipped' else (' L' if st == 'left_map' else '')
            ax.text(j, i, f'{v:.2f}{tag}', ha='center', va='center', fontsize=6, color='white' if (v < 0.15 or v > 0.85) else 'black')
    ax.set_xticks(range(len(order))); ax.set_xticklabels(order, fontsize=7); ax.xaxis.tick_top()
    ax.set_yticks(range(len(P))); ax.set_yticklabels(P.index, fontsize=6)
    ax.set_title('progress along the path (emulator); T = tipped over, L = slid off the map', fontsize=7)
    plt.tight_layout(); plt.savefig(out, dpi=110); plt.close(fig); print(out)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['list', 'show', 'preview', 'check', 'new', 'build', 'catalog', 'results'])
    ap.add_argument('name', nargs='?', default='all')
    ap.add_argument('--out', default=None); ap.add_argument('--res', type=float, default=None)
    ap.add_argument('--robot', default='robotino,youbot,burger,waffle,pioneer,jackal,husky,rover')
    ap.add_argument('--set', action='append', default=[], help='key.path=value (as drive.sh --map-set)')
    ap.add_argument('--from', dest='src', default=TEMPLATE)
    a = ap.parse_args()
    if a.cmd == 'results':
        # heat map of a map study: maps.py results <summary.csv> --out docs/maps/results.png
        results_figure(a.name, a.out or os.path.join(os.path.dirname(os.path.abspath(a.name)), 'results.png')); return
    names = all_names() if a.name == 'all' else [a.name]
    if a.cmd == 'new':
        if a.name == 'all': raise SystemExit('maps.py new NAME')
        dst = os.path.join(MAP_DIR, a.name + '.yaml')
        if os.path.exists(dst): raise SystemExit(f'{dst} exists')
        src = find(a.src)
        txt = open(src).read()
        txt = re.sub(r'^(\s*name:\s*)\S+', rf'\g<1>{a.name}', txt, count=1, flags=re.M)
        open(dst, 'w').write(txt); print(f'{dst} (from {os.path.basename(src)}); preview: python3 tools/maps.py preview {a.name}'); return
    if a.cmd == 'catalog':
        rows = []
        for n in all_names():
            M, _ = load(n); rows.append(dict(name=M['name'], file=os.path.basename(M['file']), description=M['description'], tags=M['tags'],
                                             robots=M['robots'], scenarios=list(M['scenarios'])))
        p = os.path.join(MAP_DIR, 'catalog.yaml')
        with open(p, 'w') as f:
            f.write('# generated by: python3 tools/maps.py catalog   (the map files are the source; this is only the index)\n')
            f.write(_yaml().safe_dump(dict(maps=rows), allow_unicode=True, sort_keys=False, width=200))
        print(p, len(rows), 'maps'); return
    import pandas as pd
    if a.cmd == 'list':
        R = []
        for n in names:
            M, _ = load(n, a.set); G = compute(M, max(M['resolution'], a.res or 0.05)); R.append(summary(M, G))
        print(pd.DataFrame(R).to_string(index=False)); return
    items = []
    for n in names:
        M, _ = load(n, a.set)
        G = compute(M, a.res or (M['resolution'] if a.cmd != 'preview' else max(M['resolution'], 0.02)))
        if a.cmd in ('show', 'check'):
            print(f"== {M['name']} ({M['file']})\n{M['description']}")
            print(pd.DataFrame([summary(M, G)]).drop(columns=['description']).to_string(index=False))
            if a.cmd == 'show':
                print('relief:', *(f'  {f}' for f in M['relief']), sep='\n')
                print('zones:', *(f'  {z}' for z in M['zones']), sep='\n')
                for s in M['scenarios']: print(f'scenario {s}:', scenario(M, s))
            rb = M['robots'] if (a.cmd == 'show' and M['robots']) else [r.strip() for r in a.robot.split(',') if r.strip()]
            rows, routes = check(M, G, rb)
            print('\nsingle-point wheel contact (error 0.5·κ·r², 99.9 % of the map): ok ≤ 5 % r, approx ≤ 25 % r (κ·r ≤ 0.5), invalid above')
            print(pd.DataFrame(rows).to_string(index=False))
            if routes: print('\nscenario routes:'); print(pd.DataFrame(routes).to_string(index=False))
        elif a.cmd == 'preview':
            out = a.out or os.path.join(ROOT, 'docs', 'maps'); os.makedirs(out, exist_ok=True)
            p = os.path.join(out, f"{M['name']}.png"); preview(M, G, p); print(p)
            items.append((M, compute(M, max(M['resolution'], 0.04))))
        elif a.cmd == 'build':
            mi = build(M, a.out or os.path.join('/tmp', 'map_' + M['name']), a.res)
            print(mi['dtm'], [os.path.basename(m) for m, _ in mi['meshes']])
    if a.cmd == 'preview' and len(items) > 1:
        p = os.path.join(a.out or os.path.join(ROOT, 'docs', 'maps'), 'catalog.png'); sheet(items, p); print(p)


if __name__ == '__main__':
    main()
