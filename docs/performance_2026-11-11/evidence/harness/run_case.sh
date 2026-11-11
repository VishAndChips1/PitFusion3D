#!/bin/bash
# usage: run_case.sh <label> [seconds]   (measurement shell loads the same DDS profile as the pipeline)
cd /home/vish/thesis_tests/perf_20261111
source /opt/ros/jazzy/setup.bash; source /home/vish/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=/home/vish/ros2_ws/install/pitfusion3d_bringup/share/pitfusion3d_bringup/config/fastdds_large_buffers.xml
[ "$3" = viewer ] && python3 viewer_sim.py "${2:-120}" &
python3 pipeline_perf.py "$1" "${2:-120}" > /dev/null 2>&1
wait
python3 - "$1" <<'PY'
import csv,collections,json,sys
L=sys.argv[1]; a=collections.defaultdict(lambda:[[],[]])
for r in list(csv.reader(open(L+'_rates.csv')))[1:]:
    a[r[2]][0].append(float(r[3]))
    if r[6]: a[r[2]][1].append(float(r[6]))
print('[%s]'%L,'  '.join('%s %.2fHz/%sms'%(t.split('/')[-1] if 'image' not in t else t.split('/')[2],sum(h)/len(h),round(sum(l)/len(l)) if l else '-') for t,(h,l) in a.items()))
for t,(h,l) in a.items():
    if len(l)>=6 and ('rgbt' in t or 'merged' in t):
        k=len(l)//3; print('   latency drift %s: first third %.0fms -> last third %.0fms'%(t.split('/')[-1],sum(l[:k])/k,sum(l[-k:])/k))
r=json.load(open(L+'.json')); print('   system %.0f%%/400 pipeline %.0f%% per-core %s temp %.1fC'%(r['whole_system_cpu_pct'],r['pipeline_total_cpu_pct'],r['cores_pct_mean'],r['soc_temp_c'][1]))
print('   '+'  '.join('%s %.0f%%'%(k,v['cpu_pct_mean']) for k,v in sorted(r['processes'].items(),key=lambda x:-x[1]['cpu_pct_mean'])))
PY
