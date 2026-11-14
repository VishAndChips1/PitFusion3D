#!/bin/bash
# Run a command so that, if the Pi runs short of memory, only this job is killed (never the pipeline or Claude Code).
echo 1000 > /proc/self/oom_score_adj          # the kernel picks this process first
exec systemd-run --user --scope --quiet -p MemoryMax=1500M -p MemorySwapMax=0 "$@"
