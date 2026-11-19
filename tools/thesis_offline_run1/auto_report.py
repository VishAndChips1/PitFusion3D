#!/usr/bin/env python3
"""Rebuild every report from whatever data exists (figures, tables, images captions, data dictionary,
RESULTS_RUN1.md, thesis_numbers.json, PAPER_MAP.md, results_bundle.zip).

    python3 auto_report.py [run_dir]        (default: the newest ~/thesis_tests/*_run1)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path.home() / 'thesis_tests'


def pick():
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    runs = sorted(ROOT.glob('*_run1'))
    if not runs:
        sys.exit('no run folder found')
    return runs[-1]


if __name__ == '__main__':
    run = pick()
    import TestWithLidV1 as T
    T.S.out = run
    T.S.markers_path = str(run / '00_meta' / 'markers.jsonl')
    import run1_report
    run1_report.build_all(run)
    print('reports rebuilt in', run)
