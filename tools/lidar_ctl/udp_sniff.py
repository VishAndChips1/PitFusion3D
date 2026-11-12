#!/usr/bin/env python3
"""Watch what the Livox LiDAR does on the network after its host disappears.
Counts point-data datagrams on the host data port (56001) per 5-second bin and logs the first re-broadcast (port 55000).
Usage: udp_sniff.py <seconds>   -> JSON on the last line."""
import json, select, socket, sys, time
secs = float(sys.argv[1]); t0 = time.time()
def sock(port, bcast=False):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if bcast: s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(('0.0.0.0', port)); s.setblocking(False); return s
data, bc = sock(56001), sock(55000, True)
bins = {}; first_bcast = None; last_data = None; n_bcast = 0; total = 0
while time.time() - t0 < secs:
    r, _, _ = select.select([data, bc], [], [], 0.5)
    for s in r:
        try: pkt, addr = s.recvfrom(65535)
        except BlockingIOError: continue
        now = time.time() - t0
        if s is data: bins[int(now // 5)] = bins.get(int(now // 5), 0) + 1; last_data = now; total += 1
        else:
            n_bcast += 1
            if first_bcast is None: first_bcast = now
print(json.dumps(dict(seconds=secs, data_packets_total=total, data_packets_per_5s={k * 5: v for k, v in sorted(bins.items())}, last_data_packet_at_s=last_data, first_rebroadcast_at_s=first_bcast, broadcast_packets=n_bcast)))
