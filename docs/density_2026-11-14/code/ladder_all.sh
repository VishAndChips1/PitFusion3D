#!/bin/bash
# Ladder over merge windows; stops early if any step tripped a safety gate.
D=~/density_deep_dive_20261114
run() { # window caps
  $D/code/run_campaign.sh $1 ladder w$1 $2 90 > $D/data/driver_w$1.log 2>&1
  if grep -q '"aborted": "' $D/data/results_w$1.jsonl 2>/dev/null; then echo "ABORTED in window $1" > $D/data/ladder_aborted.flag; exit 1; fi
}
run 0.5 20000,40000,80000,150000
run 0.25 40000,150000
run 0.75 80000,150000
run 1.0 80000,150000
echo done > $D/data/ladder_done.flag
