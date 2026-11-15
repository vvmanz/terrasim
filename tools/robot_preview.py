"""Preview images of the robot models exactly as they are placed in the Gazebo SDF (tools/robots.py visuals):
CAD meshes + procedural parts, wheels at their physical positions, the wheel / caster contact circles drawn
in red (radius used by the contact model) so that visual and physical geometry can be compared.

    python3 tools/robot_preview.py                       # all robots -> docs/robots/<robot>.png and robots_all.png
    python3 tools/robot_preview.py --robots burger,jackal --out /tmp/prev
"""
import os, sys, math, argparse
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import robots as RB
import meshes as MS

_CACHE = {}


def item_tris(it):
    g = it['geom']
    if g[0] == 'mesh':
        key = g[1]
        if key not in _CACHE: _CACHE[key] = MS.load_mesh(os.path.join(RB.MESH, g[1]), 1.0, with_color=True)
        T, c = _CACHE[key]
        T = T * np.asarray(g[2], float)
    elif g[0] == 'box': T, c = MS.box_tris(*g[1:4]), None
    elif g[0] == 'cyl': T, c = MS.cylinder_tris(g[1], g[2]), None
    else: T, c = MS.sphere_tris(g[1], 8), None
    col = it.get('color')
    if col is None: col = c if c is not None else (0.6, 0.6, 0.6)
    return MS.place(T, it['pose']), col


def robot_parts(name, contact=True):
    Rb = RB.robot(name); V = RB.visuals(Rb); parts = []
    if name == 'rover':
        return rover_parts(Rb)
    for it in V['base']: parts.append(item_tris(it))
    for w in Rb['wheels']:
        r = w['r']; yaw = math.radians(w.get('heading_deg', 0)) - (math.pi / 2 if name == 'robotino' else 0.0)
        if name == 'robotino': yaw = math.radians(w['heading_deg'])
        wp = (w['x'], w['y'], r, 0, 0, yaw if name == 'robotino' else 0.0)
        items = V.get(w['label'], [])
        if not items:
            items = [dict(geom=('cyl', r, w.get('b', 0.04)), pose=(0, 0, 0, math.pi / 2, 0, 0), color=(0.15, 0.15, 0.15))]
        for it in items:
            T, col = item_tris(it); parts.append((MS.place(T, wp), col))
        for it in V.get(w['label'] + '_swivel', []):       # caster swivel fork (rotates about z with the caster)
            sp = (w['x'] + w.get('trail', 0.0), w['y'], r + w.get('swivel_h', 0.0), 0, 0, 0)
            T, col = item_tris(it); parts.append((MS.place(T, sp), col))
        if contact:      # contact circle (physics radius) in the wheel plane
            a = np.linspace(0, 2 * np.pi, 41); b = w.get('b', 0.02) / 2 + 0.002
            ring = np.c_[r * np.cos(a), np.full(41, b), r * np.sin(a)]
            T = np.array([[ring[k], ring[k + 1], ring[k + 1] + [0, 0.0005, 0]] for k in range(40)])
            parts.append((MS.place(T, wp), (0.9, 0.1, 0.1)))
    return parts


def rover_parts(Rb):
    """the rover keeps its procedural model (gz_sdf.rover_model): boxes and cylinders"""
    R = Rb['rover']; parts = []
    parts.append((MS.place(MS.box_tris(1.0, 0.6, 0.25), (R['pc'][0], 0, Rb['base_h'] + R['pc'][1], 0, 0, 0)), (0.85, 0.85, 0.80)))
    for w in Rb['wheels']:
        T = MS.cylinder_tris(w['r'], w['b']); parts.append((MS.place(T, (w['x'], w['y'], w['r'], math.pi / 2, 0, 0)), (0.3, 0.3, 0.3)))
    return parts


def render(name, out, view=(28, -58), body=False):
    Rb = RB.robot(name)
    title = f"{Rb['name']}: {Rb['mass']} kg, {sum(w['type'] != 'caster' for w in Rb['wheels'])} driven wheels" + \
            (f" + {sum(w['type'] == 'caster' for w in Rb['wheels'])} caster(s)" if any(w['type'] == 'caster' for w in Rb['wheels']) else '')
    parts = robot_parts(name)
    if body:         # body-ground contact points (blue cubes)
        P = RB.body_points(Rb); e = 0.012 * max(max(abs(c) for p in P for c in p), 0.2)
        for p in P: parts.append((MS.place(MS.box_tris(e, e, e), (p[0], p[1], p[2], 0, 0, 0)), (0.1, 0.3, 0.95)))
        title += f', {len(P)} body contact points (blue)'
    MS.render_png(parts, out, view=view, size=(900, 700), title=title)


def sheet(names, out_dir):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    n = len(names); cols = 4; rows = math.ceil(n / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.4 * rows))
    for ax, nm in zip(np.ravel(axs), names):
        ax.imshow(plt.imread(os.path.join(out_dir, f'{nm}.png'))); ax.set_axis_off()
    for ax in np.ravel(axs)[n:]: ax.set_axis_off()
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'robots_all.png'), dpi=110); plt.close(fig)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', default=','.join(r for r in RB.ROBOTS if r != 'rover'), help='the rover is procedural (rocker-bogie); not previewed'); ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'robots'))
    ap.add_argument('--body', action='store_true', help='also draw the body-ground contact points (blue)')
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    names = [s for s in a.robots.split(',') if s]
    stem = lambda nm: os.path.splitext(os.path.basename(nm))[0]          # robot file paths -> file name
    for nm in names:
        f = os.path.join(a.out, f'{stem(nm)}.png'); render(nm, f, body=a.body); print('->', f)
    if len(names) > 1: sheet([stem(nm) for nm in names], a.out)
