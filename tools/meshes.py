"""Minimal STL / COLLADA (.dae) readers and a software renderer for robot previews (no external packages).

    tris = load_mesh('husky/wheel.dae')             # (n, 3, 3) triangles in metres (DAE unit + node transforms applied)
    render_png(parts, 'preview.png', view=(35, -60)) # parts: list of (tris, rgb) already placed in the model frame

STL: binary or ASCII. DAE: <asset><unit meter>, <library_geometries> triangles / polylist / polygons (positions
only), <library_visual_scenes> node matrices / translate / rotate / scale and instance_geometry, nested
instance_node; materials: the diffuse colour of the first effect used by a geometry (for the preview only).
"""
import os, re, struct, math
import numpy as np
import xml.etree.ElementTree as ET


def load_stl(path):
    data = open(path, 'rb').read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            a = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
            return a['v'].astype(float)
    txt = data.decode(errors='ignore')
    v = np.array(re.findall(r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', txt), float)
    return v.reshape(-1, 3, 3)


def _ns(tag):
    return tag.split('}', 1)[1] if '}' in tag else tag


def _node_matrix(node):
    M = np.eye(4)
    for ch in node:
        t = _ns(ch.tag); vals = [float(x) for x in (ch.text or '').split()]
        if t == 'matrix' and len(vals) == 16: M = M @ np.array(vals).reshape(4, 4)
        elif t == 'translate': T = np.eye(4); T[:3, 3] = vals[:3]; M = M @ T
        elif t == 'rotate':
            ax = np.array(vals[:3]); ax = ax / (np.linalg.norm(ax) or 1); a = math.radians(vals[3])
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(4); R[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; M = M @ R
        elif t == 'scale': S = np.eye(4); S[0, 0], S[1, 1], S[2, 2] = vals[:3]; M = M @ S
    return M


def load_dae(path, with_color=False):
    root = ET.parse(path).getroot()
    unit = 1.0; up = 'Y_UP'
    for e in root.iter():
        if _ns(e.tag) == 'unit' and e.get('meter'): unit = float(e.get('meter'))
        if _ns(e.tag) == 'up_axis' and e.text: up = e.text.strip()
    # colours: effect -> diffuse, material -> effect
    eff_col = {}
    for eff in root.iter():
        if _ns(eff.tag) != 'effect': continue
        for d in eff.iter():
            if _ns(d.tag) == 'diffuse':
                for c in d:
                    if _ns(c.tag) == 'color': eff_col[eff.get('id')] = [float(x) for x in c.text.split()[:3]]
    mat_col = {}
    for m in root.iter():
        if _ns(m.tag) == 'material':
            for ie in m:
                if _ns(ie.tag) == 'instance_effect': mat_col[m.get('id')] = eff_col.get(ie.get('url', '').lstrip('#'))
    geoms = {}
    for g in root.iter():
        if _ns(g.tag) != 'geometry': continue
        mesh = next((c for c in g if _ns(c.tag) == 'mesh'), None)
        if mesh is None: continue
        srcs = {}
        for s in mesh:
            if _ns(s.tag) == 'source':
                fa = next((c for c in s.iter() if _ns(c.tag) == 'float_array'), None)
                acc = next((c for c in s.iter() if _ns(c.tag) == 'accessor'), None)
                if fa is not None and fa.text:
                    stride = int(acc.get('stride', 3)) if acc is not None else 3
                    srcs[s.get('id')] = np.array(fa.text.split(), float).reshape(-1, stride)
        verts = {}
        for v in mesh:
            if _ns(v.tag) == 'vertices':
                for inp in v:
                    if inp.get('semantic') == 'POSITION': verts[v.get('id')] = srcs.get(inp.get('source', '').lstrip('#'))
        out, cols = [], []
        for prim in mesh:
            pt = _ns(prim.tag)
            if pt not in ('triangles', 'polylist', 'polygons'): continue
            inputs = [c for c in prim if _ns(c.tag) == 'input']
            nin = max(int(i.get('offset', 0)) for i in inputs) + 1
            vin = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            P = verts.get(vin.get('source', '').lstrip('#'))
            if P is None: continue
            off = int(vin.get('offset', 0))
            mat = prim.get('material')
            if pt == 'triangles':
                p = next((c for c in prim if _ns(c.tag) == 'p'), None)
                if p is None or not p.text: continue
                idx = np.array(p.text.split(), int).reshape(-1, nin)[:, off]
                tri = P[idx[:len(idx) // 3 * 3]][:, :3].reshape(-1, 3, 3)
            else:
                ps = [c for c in prim if _ns(c.tag) == 'p']
                if pt == 'polylist':
                    vc = np.array(next(c for c in prim if _ns(c.tag) == 'vcount').text.split(), int)
                    idx = np.array(ps[0].text.split(), int).reshape(-1, nin)[:, off]; polys, k = [], 0
                    for c in vc: polys.append(idx[k:k + c]); k += c
                else:
                    polys = [np.array(p.text.split(), int).reshape(-1, nin)[:, off] for p in ps]
                tl = [[q[0], q[j], q[j + 1]] for q in polys for j in range(1, len(q) - 1)]
                if not tl: continue
                tri = P[np.array(tl)][:, :, :3]
            out.append(tri); cols.append((mat, len(tri)))
        if out: geoms[g.get('id')] = (np.concatenate(out), cols)
    nodes_lib = {n.get('id'): n for n in root.iter() if _ns(n.tag) == 'node' and n.get('id')}
    tris, colors = [], []

    def walk(node, M):
        M = M @ _node_matrix(node)
        for ch in node:
            t = _ns(ch.tag)
            if t == 'instance_geometry':
                g = geoms.get(ch.get('url', '').lstrip('#'))
                if g is None: continue
                T, cols = g
                binds = {im.get('symbol'): im.get('target', '').lstrip('#') for im in ch.iter() if _ns(im.tag) == 'instance_material'}
                h = np.c_[T.reshape(-1, 3), np.ones(T.shape[0] * 3)] @ M.T
                tris.append(h[:, :3].reshape(-1, 3, 3))
                for mat, n in cols: colors.extend([mat_col.get(binds.get(mat, mat))] * n)
            elif t == 'node': walk(ch, M)
            elif t == 'instance_node':
                ref = nodes_lib.get(ch.get('url', '').lstrip('#'))
                if ref is not None: walk(ref, M)
    scenes = [s for s in root.iter() if _ns(s.tag) == 'visual_scene']
    if scenes:
        for n in scenes[0]:
            if _ns(n.tag) == 'node': walk(n, np.eye(4))
    if not tris:                                     # no scene: all geometries as they are
        for T, cols in geoms.values():
            tris.append(T)
            for mat, n in cols: colors.extend([mat_col.get(mat)] * n)
    T = np.concatenate(tris) * unit if tris else np.zeros((0, 3, 3))
    if up == 'Y_UP': T = np.stack([T[..., 0], -T[..., 2], T[..., 1]], -1)       # COLLADA Y-up -> Z-up (as gz-common)
    elif up == 'X_UP': T = np.stack([-T[..., 1], T[..., 0], T[..., 2]], -1)
    return (T, colors) if with_color else T


def load_mesh(path, scale=1.0, with_color=False):
    if path.lower().endswith('.dae'):
        T, c = load_dae(path, with_color=True)
    else:
        T, c = load_stl(path), None
    T = T * np.asarray(scale, float)
    return (T, c) if with_color else T


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def place(T, pose):
    """pose = (x, y, z, roll, pitch, yaw) -> triangles moved by the pose"""
    R = rpy_matrix(*pose[3:6]); return T @ R.T + np.array(pose[:3])


def box_tris(sx, sy, sz):
    c = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2) for z in (-sz / 2, sz / 2)])
    F = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return c[np.array(F)]


def cylinder_tris(r, L, n=24):
    """axis along z"""
    a = np.linspace(0, 2 * np.pi, n + 1)
    top = np.c_[r * np.cos(a), r * np.sin(a), np.full(n + 1, L / 2)]; bot = top * np.array([1, 1, -1])
    T = []
    for k in range(n):
        T += [[bot[k], bot[k + 1], top[k + 1]], [bot[k], top[k + 1], top[k]], [[0, 0, L / 2], top[k], top[k + 1]], [[0, 0, -L / 2], bot[k + 1], bot[k]]]
    return np.array(T)


def sphere_tris(r, n=12):
    T = []
    th = np.linspace(0, np.pi, n + 1); ph = np.linspace(0, 2 * np.pi, 2 * n + 1)
    P = lambda t, p: [r * math.sin(t) * math.cos(p), r * math.sin(t) * math.sin(p), r * math.cos(t)]
    for i in range(n):
        for j in range(2 * n):
            a, b, c, d = P(th[i], ph[j]), P(th[i + 1], ph[j]), P(th[i + 1], ph[j + 1]), P(th[i], ph[j + 1])
            T += [[a, b, c], [a, c, d]]
    return np.array(T)


def render_png(parts, out, view=(30, -55), size=(900, 700), title=None, ground=True, max_tris=250000):
    """painter's-algorithm render with Lambert shading; parts = [(tris, rgb or [rgb per tri])]"""
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    allT, allC = [], []
    for T, col in parts:
        if len(T) == 0: continue
        if len(T) > max_tris:
            idx = np.random.default_rng(0).choice(len(T), max_tris, replace=False); T = T[idx]
            if isinstance(col, list): col = [col[i] for i in idx]
        allT.append(T)
        if isinstance(col, list) and len(col) == len(T): allC += [c if c is not None else [0.6, 0.6, 0.62] for c in col]
        else: allC += [col] * len(T)
    T = np.concatenate(allT); C = np.array(allC, float)
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0); ctr = (lo + hi) / 2; span = max(hi - lo)
    if ground:
        g = 0.75 * span; z0 = lo[2] - 1e-4
        G = np.array([[[ctr[0] - g, ctr[1] - g, z0], [ctr[0] + g, ctr[1] - g, z0], [ctr[0] + g, ctr[1] + g, z0]],
                      [[ctr[0] - g, ctr[1] - g, z0], [ctr[0] + g, ctr[1] + g, z0], [ctr[0] - g, ctr[1] + g, z0]]])
        T = np.concatenate([G, T]); C = np.concatenate([[[0.86, 0.84, 0.80]] * 2, C])
    el, az = math.radians(view[0]), math.radians(view[1])
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])     # towards the viewer
    right = np.cross([0, 0, 1], d); right /= np.linalg.norm(right); upv = np.cross(d, right)
    P = T - ctr
    X = P @ right; Y = P @ upv; Z = P @ d
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nn = np.linalg.norm(N, axis=1); ok = nn > 1e-14
    N[ok] /= nn[ok, None]
    light = d * 0.6 + np.array([0.3, 0.2, 0.75]); light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.abs(N @ light)
    order = np.argsort(Z.mean(1))
    fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1])
    polys = np.stack([X, Y], -1)[order]
    cols = np.clip(C[order] * shade[order, None], 0, 1)
    ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors=cols, linewidths=0.2))
    ng = 2 if ground else 0                             # frame the robot (not the ground plane), 8 % margin, room for the title
    x0, x1, y0, y1 = X[ng:].min(), X[ng:].max(), Y[ng:].min(), Y[ng:].max()
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2 + 0.04 * (y1 - y0)
    hw = 0.54 * max(x1 - x0, (y1 - y0) * 1.12 * size[0] / size[1])
    ax.set_xlim(cx - hw, cx + hw); ax.set_ylim(cy - hw * size[1] / size[0], cy + hw * size[1] / size[0]); ax.set_aspect('equal'); ax.axis('off')
    if title: ax.text(0.02, 0.97, title, transform=ax.transAxes, fontsize=13, va='top')
    fig.savefig(out, dpi=100, facecolor='white'); plt.close(fig)


def save_stl(path, tris):
    """binary STL (float32), normals from the vertex order"""
    t = np.asarray(tris, float)
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
    a = np.zeros(len(t), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    a['n'] = nrm; a['v'] = t
    with open(path, 'wb') as f:
        f.write(b'terrasim decimated mesh'.ljust(80, b' ')); f.write(struct.pack('<I', len(t))); f.write(a.tobytes())


def decimate(tris, cell):
    """vertex-clustering decimation: vertices snapped to the mean of their grid cell (size cell, mesh units),
    collapsed and duplicate triangles dropped; the shape is kept to about one cell"""
    t = np.asarray(tris, float); V = t.reshape(-1, 3)
    key = np.floor((V - V.min(0)) / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    cnt = np.bincount(inv); P = np.zeros((cnt.size, 3))
    for k in range(3): P[:, k] = np.bincount(inv, V[:, k]) / cnt
    F = inv.reshape(-1, 3)
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    _, first = np.unique(np.sort(F, axis=1), axis=0, return_index=True)
    return P[F[np.sort(first)]]


if __name__ == '__main__':          # python3 tools/meshes.py decimate in.stl out.stl cell_in_mesh_units
    import sys
    if len(sys.argv) == 5 and sys.argv[1] == 'decimate':
        T = load_stl(sys.argv[2]); D = decimate(T, float(sys.argv[4])); save_stl(sys.argv[3], D)
        print(f'{sys.argv[2]}: {len(T)} -> {len(D)} triangles')
