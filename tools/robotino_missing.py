#!/usr/bin/env python3
"""Fill the gaps of a Robotino synthetic dataset: every (surface, speed) series that is not in DATASET/csv is run and the
result is merged with the existing set.

Why gaps appear: `--surfaces all` takes the speeds of every surface from the config, and the calibrated floors that the
real dataset has only at 0.1 m/s (table, red) are then run only at 0.1 m/s. Their parameters are the same at any speed,
so the model gives them at 0.2 and 0.3 m/s as well (there is no real data to compare those with).

    bash scripts/robotino_missing.sh DATASET [--engine harmonic] [--jobs 4] [--speeds 0.1,0.2,0.3] [--surfaces all]
        -> DATASET_missing/   only the new series (same format, same options)
        -> DATASET_full/      DATASET + DATASET_missing merged (tools/robotino_merge.py), links, no copies
    --dry-run: only list what is missing.
Other options (e.g. --noise, --seed, --strict-format, --no-full) are passed to tools/robotino_run.py.
"""
import argparse, glob, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


def present(dataset):
    """(surface, speed) pairs that already have a clean CSV"""
    out = set()
    for p in glob.glob(os.path.join(dataset, 'csv', '*_clean.csv')):
        n = os.path.basename(p)[:-len('_clean.csv')]
        if n.endswith('_ext'): continue
        s, sp = n.rsplit('_', 1)
        try: out.add((s, round(float(sp), 3)))
        except ValueError: pass
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dataset'); ap.add_argument('--engine', default='harmonic'); ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--speeds', default='0.1,0.2,0.3'); ap.add_argument('--surfaces', default='all')
    ap.add_argument('--dry-run', action='store_true')
    a, rest = ap.parse_known_args()
    import robotino_model as M, robotino_run as RUN
    P = M.load_params()
    want = RUN.select_surfaces(P, a.surfaces) if hasattr(RUN, 'select_surfaces') else None
    if want is None:
        S = P['surfaces']; want = []
        for tok in a.surfaces.split(','):
            if tok == 'all': want += [k for k in S if S[k].get('_kind') != 'sweep']
            elif tok == 'real': want += [k for k in S if S[k].get('_kind') == 'calibrated']
            elif tok in ('synthetic', 'extreme', 'user', 'hard'): want += [k for k in S if S[k].get('_kind') == tok]
            elif tok in S: want.append(tok)
    speeds = [round(float(x), 3) for x in a.speeds.split(',')]
    have = present(a.dataset)
    if not have: raise SystemExit(f'{a.dataset}/csv: no *_clean.csv (is this a folder written by scripts/robotino.sh?)')
    miss = {}
    for s in want:
        for sp in speeds:
            if (s, sp) not in have: miss.setdefault(s, []).append(sp)
    if not miss: print('nothing is missing'); return
    print('missing series:'); [print(f'  {s}: {", ".join(f"{v:g}" for v in sp)} m/s') for s, sp in miss.items()]
    if a.dry_run: return
    out = os.path.normpath(a.dataset) + '_missing'
    # surfaces with the same missing speeds go into one call
    groups = {}
    for s, sp in miss.items(): groups.setdefault(tuple(sp), []).append(s)
    for k, (sp, ss) in enumerate(groups.items()):
        o = out if len(groups) == 1 else f'{out}/part{k}'
        cmd = ['bash', os.path.join(ROOT, 'scripts', 'robotino.sh'), '--engine', a.engine, '--surfaces', ','.join(ss),
               '--speeds', ','.join(f'{v:g}' for v in sp), '--jobs', str(a.jobs), '--out', o] + rest
        print('->', ' '.join(cmd), flush=True)
        r = subprocess.run(cmd)
        if r.returncode != 0: raise SystemExit(f'run failed: {" ".join(cmd)}')
    parts = [out] if len(groups) == 1 else sorted(glob.glob(f'{out}/part*'))
    full = os.path.normpath(a.dataset) + '_full'
    cmd = [sys.executable, os.path.join(HERE, 'robotino_merge.py'), a.dataset] + parts + ['--out', full]
    print('->', ' '.join(cmd), flush=True); subprocess.run(cmd, check=True)
    print(f'done: new series in {out}, everything together in {full}')


if __name__ == '__main__':
    main()
