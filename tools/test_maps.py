"""tests of the map files and tools/maps.py:  python3 tools/test_maps.py"""
import os, sys, math, json, tempfile, copy
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import maps as MP, gen_dtm

ok = 0


def check(cond, msg):
    global ok
    if not cond: raise SystemExit('FAIL: ' + msg)
    ok += 1


def mk(world):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'm.json'); json.dump(dict(world=world), open(p, 'w')); return p


base = dict(name='t', default_surface='gray', bounds=dict(xmin=-2, xmax=4, ymin=-1, ymax=1), resolution=0.01, zones=[], relief=[])

# plane: exact height and slope
M, _ = MP.load(mk(dict(base, relief=[dict(type='plane', slope_deg=10, dir_deg=0)])))
G = MP.compute(M)
check(abs(float(MP.height_at(G, 1.0, 0.3)) - math.tan(math.radians(10))) < 1e-9, 'plane height')
check(abs(MP.slope_deg(G).max() - 10) < 1e-6, 'plane slope')
# plane along y
M, _ = MP.load(mk(dict(base, relief=[dict(type='plane', slope_deg=20, dir_deg=90)])))
G = MP.compute(M); check(abs(float(MP.height_at(G, 2.0, 0.5)) - 0.5 * math.tan(math.radians(20))) < 1e-9, 'plane dir 90')

# ramp: flat before, slope in the middle, plateau after, continuous, smooth knee
M, _ = MP.load(mk(dict(base, relief=[dict(type='ramp', slope_deg=15, **{'from': 0.0, 'to': 2.0}, blend=0.4)])))
G = MP.compute(M); xs = G['xs']; h = G['H'][100]
t = math.tan(math.radians(15))
check(abs(h[xs < -0.3]).max() < 1e-12, 'ramp flat before')
check(abs(float(MP.height_at(G, 1.0, 0)) - t * 1.0) < 1e-6, 'ramp mid height')
check(abs(float(MP.height_at(G, 3.5, 0)) - t * 2.0) < 1e-6, 'ramp plateau = slope * length')
check(np.abs(np.diff(h)).max() < t * 0.0101 + 1e-9, 'ramp continuous')
check(np.abs(np.diff(np.diff(h))).max() < 1e-3, 'ramp knee smooth (blend)')

# slopes: total height = sum tan * length
segs = [dict(slope_deg=5, length=1), dict(slope_deg=10, length=1), dict(slope_deg=20, length=1)]
M, _ = MP.load(mk(dict(base, relief=[dict(type='slopes', start=0.0, blend=0.2, segments=segs)])))
G = MP.compute(M)
check(abs(float(MP.height_at(G, 3.8, 0)) - sum(math.tan(math.radians(s['slope_deg'])) for s in segs)) < 1e-6, 'slopes total height')

# mound = gen_dtm.mound_profile (Fig. 15)
M, _ = MP.load(mk(dict(base, relief=[dict(type='mound', x_start=0.5, L_up=0.8, L_top=0.3, L_down=0.8, height=0.14, smooth=0.1)])))
G = MP.compute(M); ref = gen_dtm.mound_profile(np.arange(-3, 5, 0.01), 0.5, 0.8, 0.3, 0.8, 0.14, 0.1)
check(abs(G['H'].max() - ref.max()) < 2e-3 and abs(G['H'].max() - 0.14) < 5e-3, 'mound height')

# ridges: count and height
M, _ = MP.load(mk(dict(base, relief=[dict(type='ridges', height=0.02, width=0.3, pitch=0.6, **{'from': 0.0, 'to': 3.0})])))
G = MP.compute(M); h = G['H'][100]; pk = ((h[1:-1] > h[:-2]) & (h[1:-1] >= h[2:]) & (h[1:-1] > 0.019)).sum()
check(pk == 5 and abs(h.max() - 0.02) < 1e-4, f'ridges: 5 crests of 2 cm (got {pk}, {h.max():.4f})')

# bump: volume of the cosine bell = pi R^2 h (1/2 - 2/pi^2)
M, _ = MP.load(mk(dict(base, relief=[dict(type='bump', center=[1, 0], height=0.05, radius=0.4)])))
G = MP.compute(M); vol = G['H'].sum() * 0.01 ** 2
check(abs(vol - math.pi * 0.16 * 0.05 * (0.5 - 2 / math.pi ** 2)) < 2e-5, 'bump volume')

# crater: depth at the centre, rim above zero
M, _ = MP.load(mk(dict(base, bounds=dict(xmin=-3, xmax=3, ymin=-3, ymax=3), relief=[dict(type='crater', center=[0, 0], radius=1.5, depth=0.3, rim=0.25)])))
G = MP.compute(M); check(abs(float(MP.height_at(G, 0, 0)) + 0.3) < 0.01 and G['H'].max() > 0.05, 'crater depth and rim')

# flatten: level pad
M, _ = MP.load(mk(dict(base, relief=[dict(type='plane', slope_deg=10), dict(type='flatten', center=[2, 0], radius=0.3, blend=0.2)])))
G = MP.compute(M); check(abs(float(MP.height_at(G, 2.2, 0.1)) - float(MP.height_at(G, 2.0, 0))) < 1e-9, 'flatten pad level')

# zones: later overwrite earlier; circle area
Z = [dict(surface='carpet', type='rect', xmin=-2, xmax=4, ymin=-1, ymax=1), dict(surface='foam', type='circle', center=[1, 0], radius=0.5)]
M, _ = MP.load(mk(dict(base, zones=Z)))
G = MP.compute(M); S = G['S']
check(S == ['carpet', 'foam'], f'unused default surface dropped, order kept ({S})')
frac = (G['T'] == 1).sum() * 0.01 ** 2
check(abs(frac - math.pi * 0.25) < 0.01, 'circle area')
M, _ = MP.load(mk(dict(base, zones=Z[::-1])))
G = MP.compute(M); check(G['S'] == ['carpet'], 'rect over circle hides it')
M, _ = MP.load(mk(dict(base, zones=[dict(type='checkerboard', surfaces=['red', 'green'], cell=1.0)])))
G = MP.compute(M); check(set(G['S']) == {'red', 'green'} and abs((G['T'] == G['S'].index('red')).mean() - 0.5) < 0.05, 'checkerboard')
M, _ = MP.load(mk(dict(base, zones=[dict(surface='table', type='band', points=[[-2, 0], [4, 0]], width=0.4)])))
G = MP.compute(M); check(abs((G['T'] == G['S'].index('table')).mean() - 0.4 / 2.0) < 0.02, 'band width')

# DTM round trip
d = tempfile.mkdtemp(); M, _ = MP.load(mk(dict(base, zones=Z, relief=[dict(type='plane', slope_deg=5)])))
mi = MP.build(M, d, meshes=True)
toks = open(mi['dtm']).read().split(); nx, ny = int(toks[2]), int(toks[3]); x0, y0, res = (float(v) for v in toks[4:7]); nt = int(toks[7])
check((nx, ny) == (len(mi['xs']), len(mi['ys'])) and abs(x0 + 2) < 1e-9 and abs(y0 + 1) < 1e-9 and abs(res - 0.01) < 1e-12, 'dtm header')
check([toks[8 + 10 * k] for k in range(nt)] == mi['surfaces'], 'dtm type names')
H = np.array(toks[8 + nt * 10:8 + nt * 10 + nx * ny], float).reshape(ny, nx); check(np.abs(H - mi['H']).max() < 1e-5, 'dtm heights')
check(len(mi['meshes']) == 2 and all(os.path.getsize(o) > 1000 for o, _ in mi['meshes']), 'meshes')

# scenarios: route -> spline from the first point, spawn yaw towards the second
W = dict(base, scenarios=dict(r=dict(route=[[0, 0], [1, 1], [2, 0]]), l=dict(spawn=[-1, 0, 90], traj='line:L=0.5')))
M, sc = MP.load(mk(W)); s = MP.scenario(M, 'r')
check(sc == 'r' and abs(s['spawn'][2] - 45) < 1e-9 and s['traj'].startswith('spline:pts='), 'route scenario')
P = MP.route_world(s); check(np.hypot(*(P[-1] - [2, 0])) < 0.02, 'route ends at the last point')
P = MP.route_world(MP.scenario(M, 'l')); check(np.hypot(*(P[-1] - [-1, 0.5])) < 1e-3, 'line scenario in map frame')

# overrides and errors
M, _ = MP.load(mk(dict(base, relief=[dict(type='plane', slope_deg=10)])), ['relief.0.slope_deg=12', 'default_surface=carpet'])
check(M['relief'][0]['slope_deg'] == 12 and M['default_surface'] == 'carpet', '--map-set')
for bad, what in ((dict(base, obstacles=[dict(type='box')]), 'obstacles'), (dict(base, relief=[dict(type='nope')]), 'relief type'),
                  (dict(base, foo=1), 'unknown key')):
    try: MP.load(mk(bad)); check(False, what + ' accepted')
    except SystemExit as e: check('FAIL' not in str(e), what)

# custom surfaces
M, _ = MP.load(mk(dict(base, surfaces=dict(my=dict(base='carpet', phi_deg=20, _color='0.1 0.2 0.3 1')))))
S = MP.resolve_surfaces(M, dict(carpet=dict(phi_deg=30, K=0.01)))
check(S['my']['phi_deg'] == 20 and S['my']['K'] == 0.01 and MP.colors(M)['my'] == '0.1 0.2 0.3 1', 'custom surface')

# contact check: a plane is exact, a 2 cm ridge of 30 cm is approximate for a 4 cm wheel
G = dict(xs=np.arange(0, 2, 0.01), ys=np.arange(0, 1, 0.01)); X, Y = np.meshgrid(G['xs'], G['ys'])
G['H'] = 0.1 * X; check(MP.contact_error(G, 0.04).max() < 1e-9, 'plane contact error 0')
G['H'] = 0.02 * MP._bell(np.abs(X - 1), 0.15); e = MP.contact_error(G, 0.04).max()
check(abs(e - 0.5 * 0.02 * math.pi ** 2 / (2 * 0.15 ** 2) * 0.04 ** 2) < 2e-4, f'ridge contact error ({e})')

# every shipped map loads, builds and its routes stay inside
import robots as RB
for n in MP.all_names():
    M, sc = MP.load(n); G = MP.compute(M, max(M['resolution'], 0.05))
    check(len(M['scenarios']) > 0 and sc in M['scenarios'], f'{n}: scenarios')
    for r in M['robots']: RB.robot(r)
    rows, routes = MP.check(M, G, [])
    check(all(r['inside'] for r in routes), f'{n}: routes inside the map')
print(f'test_maps: {ok} checks passed')
