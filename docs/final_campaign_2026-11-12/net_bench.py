#!/usr/bin/env python3
"""Network measurements: WiFi link, gateway/internet latency, download speed, DNS. JSON on last line."""
import re, json, subprocess, time
def sh(c, t=90):
    try: return subprocess.run(c, shell=True, capture_output=True, text=True, timeout=t).stdout
    except subprocess.TimeoutExpired: return ''
R = {}
iw = sh('iwconfig wlan0 2>&1'); R['iwconfig'] = iw
for k, pat in dict(bitrate_mbps=r'Bit Rate=([\d.]+)', signal_dbm=r'Signal level=(-?\d+)', link_quality=r'Link Quality=(\d+/\d+)', tx_power_dbm=r'Tx-Power=(\d+)', excessive_retries=r'Tx excessive retries:(\d+)', freq_ghz=r'Frequency:([\d.]+)').items():
    m = re.search(pat, iw); R[k] = m.group(1) if m else None
R['power_save'] = 'on' if 'Power Management:on' in iw else 'off'
def ping(host, n, iv):
    o = sh(f'ping -c {n} -i {iv} -W 2 {host}', n * iv + 20); m = re.search(r'(\d+) packets transmitted, (\d+) received.*?([\d.]+)% packet loss', o, re.S); t = re.search(r'= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms', o)
    return dict(sent=int(m.group(1)), received=int(m.group(2)), loss_pct=float(m.group(3)), rtt_min=float(t.group(1)), rtt_avg=float(t.group(2)), rtt_max=float(t.group(3)), rtt_mdev=float(t.group(4))) if m and t else dict(raw=o[-200:])
gw = sh("ip route | awk '/default/ {print $3; exit}'").strip(); R['gateway'] = gw
R['ping_gateway_200'] = ping(gw, 200, 0.1) if gw else None
R['ping_1.1.1.1_50'] = ping('1.1.1.1', 50, 0.2); R['ping_8.8.8.8_50'] = ping('8.8.8.8', 50, 0.2)
R['dns_ms'] = None
t = time.time(); sh('getent hosts example.com'); R['dns_ms'] = (time.time() - t) * 1000
dl = {}
for name, url in (('cloudflare_25MB', 'https://speed.cloudflare.com/__down?bytes=25000000'), ('cachefly_10MB', 'http://cachefly.cachefly.net/10mb.test')):
    o = sh(f'curl -s -o /dev/null -L -m 30 -w "%{{speed_download}} %{{size_download}} %{{time_total}} %{{http_code}}" "{url}"', 40).split()
    if len(o) == 4: dl[name] = dict(mbit_s=float(o[0]) * 8 / 1e6, bytes=float(o[1]), secs=float(o[2]), http=o[3])
R['download'] = dl
R['upload'] = {}
o = sh('head -c 8000000 /dev/zero | curl -s -o /dev/null -m 30 -X POST --data-binary @- -w "%{speed_upload} %{size_upload} %{time_total} %{http_code}" "https://speed.cloudflare.com/__up"', 40).split()
if len(o) == 4: R['upload']['cloudflare_8MB'] = dict(mbit_s=float(o[0]) * 8 / 1e6, bytes=float(o[1]), secs=float(o[2]), http=o[3])
R['iwconfig_after'] = sh('iwconfig wlan0 2>&1 | grep -E "Signal|Retries|Bit Rate"')
print(json.dumps(R))
