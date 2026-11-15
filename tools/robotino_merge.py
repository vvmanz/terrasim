"""Merge several Robotino dataset folders (outputs of scripts/robotino.sh) into one, without re-simulating.

    python3 tools/robotino_merge.py datasets/<old> datasets/<new> --out datasets/<merged>

Takes csv/<surface>_<speed>_{clean,noisy}.csv (and _ext_*) from every folder. If a surface and speed occur in several
folders, the LAST folder wins (so a re-run replaces the old one). Writes dataset_clean.csv / dataset_noisy.csv,
dataset_ext_*.csv, summary.csv / summary.md and the figures of the merged set. Large per-run files (csv/*_full.csv.gz,
runs/) are linked, not copied.
"""
import os, sys, glob, json, argparse, shutil
import pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='+'); ap.add_argument('--out', required=True); ap.add_argument('--no-figures', action='store_true')
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    for d in ('csv', 'runs', 'figures'): os.makedirs(os.path.join(out, d), exist_ok=True)
    pick, summ, params, const = {}, [], {}, []
    for f in a.folders:
        f = os.path.abspath(f)
        if not os.path.isdir(os.path.join(f, 'csv')): raise SystemExit(f'{f}: no csv/ folder (not an output of scripts/robotino.sh)')
        for p in glob.glob(os.path.join(f, 'csv', '*.csv')) + glob.glob(os.path.join(f, 'csv', '*_full.csv.gz')):
            pick[os.path.basename(p)] = p
        for r in glob.glob(os.path.join(f, 'runs', '*')):
            pick['runs/' + os.path.basename(r)] = r
        if os.path.exists(os.path.join(f, 'summary.csv')): summ.append((f, pd.read_csv(os.path.join(f, 'summary.csv'))))
        if os.path.exists(os.path.join(f, 'params_used.json')): params.update(json.load(open(os.path.join(f, 'params_used.json'))).get('surfaces', {}))
        if os.path.exists(os.path.join(f, 'surfaces_constant.csv')): const.append(pd.read_csv(os.path.join(f, 'surfaces_constant.csv')))
    if const: pd.concat(const, ignore_index=True).drop_duplicates('surface', keep='last').to_csv(os.path.join(out, 'surfaces_constant.csv'), index=False)
    for name, src in pick.items():
        dst = os.path.join(out, name) if name.startswith('runs/') else os.path.join(out, 'csv', name)
        if os.path.lexists(dst): os.remove(dst) if not os.path.isdir(dst) or os.path.islink(dst) else shutil.rmtree(dst)
        os.symlink(src, dst)
    if params: json.dump(dict(surfaces=params, merged_from=[os.path.abspath(x) for x in a.folders]), open(os.path.join(out, 'params_used.json'), 'w'), indent=1, ensure_ascii=False)
    keys = sorted({n.rsplit('_', 1)[0] for n in pick if (n.endswith('_clean.csv') or n.endswith('_noisy.csv')) and '_ext_' not in n})   # <surface>_<speed>
    import robotino_run as RUN, robotino_model as M
    P = M.load_params()
    S_all = dict(P['surfaces']); S_all.update(params)
    # summary: last folder wins for a surface/speed; series without a row (a run interrupted before its summary was
    # written) get one recomputed from csv/<series>_clean.csv and runs/<series>/log_detail.csv
    T = pd.concat([s for _, s in summ], ignore_index=True) if summ else pd.DataFrame(columns=['surface', 'speed'])
    T = T.drop_duplicates(['surface', 'speed'], keep='last')
    have = {(r.surface, round(float(r.speed), 3)) for r in T.itertuples()}
    extra = []
    for k in keys:
        surf, sp = k.rsplit('_', 1)
        if (surf, round(float(sp), 3)) in have: continue
        f = os.path.join(out, 'csv', f'{k}_clean.csv')
        if not os.path.exists(f): continue
        row = dict(surface=surf, speed=float(sp), status='from csv')
        try: row.update(RUN.run_summary(pd.read_csv(f), os.path.join(out, 'runs', k, 'log_detail.csv'), S_all.get(surf, {})))
        except Exception as e: print(f'{k}: summary could not be recomputed ({e})')
        extra.append(row)
    if extra: T = pd.concat([T, pd.DataFrame(extra)], ignore_index=True)
    T = T[[(f'{r.surface}_{float(r.speed):g}' in keys) or (f'{r.surface}_{r.speed}' in keys) for r in T.itertuples()]].reset_index(drop=True)
    if not T.empty:
        T.to_csv(os.path.join(out, 'summary.csv'), index=False)
        cols = [c for c in ('surface', 'kind', 'speed', 'status', 'runs', 'v_ratio', 'slip_driven', 'I_driven', 'Ke_mean', 'yaw_dps', 'sinkage_mm', 'Rcomp_N') if c in T]
        with open(os.path.join(out, 'summary.md'), 'w') as fh:
            fh.write(f'# Robotino, merged dataset: {", ".join(os.path.basename(os.path.normpath(x)) for x in a.folders)}\n\n')
            try: fh.write(T[cols].to_markdown(index=False))
            except ImportError: fh.write(T[cols].to_string(index=False))
            fh.write('\n')
    order = list(P['surfaces'])
    keys.sort(key=lambda k: (order.index(k.rsplit('_', 1)[0]) if k.rsplit('_', 1)[0] in order else 99, float(k.rsplit('_', 1)[1])))
    surfaces = []
    for k in keys:
        s = k.rsplit('_', 1)[0]
        if s not in surfaces: surfaces.append(s)
    for kind in ('clean', 'noisy'):
        files = [os.path.join(out, 'csv', f'{k}_{kind}.csv') for k in keys if os.path.exists(os.path.join(out, 'csv', f'{k}_{kind}.csv'))]
        if files:
            pd.concat([pd.read_csv(f) for f in files], ignore_index=True).to_csv(os.path.join(out, f'dataset_{kind}.csv'), index=False, float_format='%.7g')
            print(f'dataset_{kind}.csv: {len(files)} series')
        ext = [os.path.join(out, 'csv', f'{k}_ext_{kind}.csv') for k in keys if os.path.exists(os.path.join(out, 'csv', f'{k}_ext_{kind}.csv'))]
        if ext:
            RUN.EXT.write(RUN.EXT.renumber([pd.read_csv(f) for f in ext]), os.path.join(out, f'dataset_ext_{kind}.csv'))
    if not a.no_figures and surfaces:
        RUN.figures(argparse.Namespace(out=out), surfaces, P)
    print(f'merged: {len(surfaces)} surfaces ({", ".join(surfaces)}) -> {out}')


if __name__ == '__main__':
    main()
