import json
M = {}
# ---- data volumes (MB/s from measured message sizes and rates)
streams = {'LiDAR scans (raw)': (432000, 10.0), 'RGB frames (raw rgb8)': (2764800, 10.0), 'thermal frames (raw mono16)': (655360, 30.0), 'fused cloud (XYZ+I+RGB+T)': (576000, 10.0), 'merged cloud (full resolution)': (121000 * 24, 1.53)}
vol = {k: dict(mb_per_s=s * r / 1e6, gb_per_hour=s * r * 3600 / 1e9) for k, (s, r) in streams.items()}
vol['all three raw sensors'] = dict(mb_per_s=sum(vol[k]['mb_per_s'] for k in ('LiDAR scans (raw)', 'RGB frames (raw rgb8)', 'thermal frames (raw mono16)'))); vol['all three raw sensors']['gb_per_hour'] = vol['all three raw sensors']['mb_per_s'] * 3.6
vol['merged cloud, quantised 11 B/pt + gzip (viewer format, full res)'] = dict(mb_per_s=121000 * 11 / 1.5 * 1.53 / 1e6, gb_per_hour=121000 * 11 / 1.5 * 1.53 * 3600 / 1e9)
M['data_volume'] = vol; M['sd'] = dict(free_gb=70, write_mb_s=53.0, read_mb_s=79.6)
M['sd']['hours_by_stream'] = {k: M['sd']['free_gb'] / v['gb_per_hour'] for k, v in vol.items()}
M['sd']['can_sustain'] = {k: v['mb_per_s'] <= M['sd']['write_mb_s'] * 0.8 for k, v in vol.items()}
# ---- power / battery runtime
pi_typ, pi_cons = 4.10, 4.10 / 0.85 + 2.5          # SoC-rail mean (measured) and the conservative input-power bound
lidar = 9.0                                         # Livox Avia typical operating power (manufacturer figure; not measured here)
modes = {'Pi + sensors, LiDAR asleep': (pi_typ, pi_cons), 'Pi + sensors + LiDAR scanning': (pi_typ + lidar, pi_cons + lidar)}
M['power_w'] = {k: dict(typical=a, conservative=b) for k, (a, b) in modes.items()}
M['runtime_h'] = {f'{wh} Wh': {k: dict(typical=wh / a, conservative=wh / b) for k, (a, b) in modes.items()} for wh in (50, 100, 150, 300)}
json.dump(M, open('data/stakeholder_numbers.json', 'w'), indent=1)
for k, v in vol.items(): print('%-62s %6.2f MB/s %7.1f GB/h  | SD free 70 GB lasts %5.1f h  %s' % (k, v['mb_per_s'], v['gb_per_hour'], M['sd']['hours_by_stream'][k], 'ok' if M['sd']['can_sustain'][k] else 'EXCEEDS card write speed'))
print(); 
for wh, d in M['runtime_h'].items(): print(wh, {k: '%.1f-%.1f h' % (v['conservative'], v['typical']) for k, v in d.items()})
