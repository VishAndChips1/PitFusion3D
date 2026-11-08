#!/usr/bin/env python3
"""WiFi end-to-end latency + bandwidth test, for the thesis report's
"Communication results: wifi end2end speed, bandwidth vs latency" section.

This cannot be run solo -- unlike the power/CPU/benchmark tooling in
tools/power_monitoring/, which only needs this one Pi, a real WiFi
bandwidth number needs a second device (the viewing laptop/phone) as the
other end of the link, with an operator present to run the client-side
half. Confirmed on this Pi (2026-11-08): iperf3 is NOT currently
installed (`which iperf3` found nothing) -- install it first:

    sudo apt install iperf3

Usage, once iperf3 is installed and a second device is on the same WiFi
(either this Pi's own network, or connected to its PitFusion3D-AP
hotspot -- see pitfusion3d_bringup's start_hotspot_viewing.sh):

  On the OTHER device (the one playing "viewer"):
    iperf3 -s

  On this Pi, pointed at that device's IP:
    python3 wifi_bandwidth_latency_test.py <other_device_ip> [--duration 30]

What it measures:
  1. Ping latency sweep to that device (min/avg/max/mdev, packet loss) --
     the "how fast does a round-trip actually happen over this WiFi link"
     number, independent of any application-level code.
  2. iperf3 TCP throughput, both directions (-c then -c --reverse) -- raw
     link bandwidth in each direction, again independent of this
     project's own HTTP/gzip code.
  3. If fusion_web_streamer is already running (ros2 launch ... already
     up), also times N real HTTP GETs of /merged.bin from this script
     itself as a rough proxy for "what the browser actually experiences"
     -- NOT a replacement for opening the page in a real browser on the
     other device (this request doesn't pay the JS parse/WebGL-render
     cost a real viewer does), just a quick same-script sanity check of
     the server's real response time and payload size over this same
     link while the iperf3 numbers establish the link's raw ceiling.

Writes one JSON summary + prints a plain-text report; nothing here needs
root or touches system config.
"""
import argparse
import json
import re
import shutil
import statistics
import subprocess
import sys
import time
import urllib.request


def check_iperf3():
    return shutil.which('iperf3') is not None


def ping_sweep(target_ip, count=30):
    try:
        out = subprocess.run(
            ['ping', '-c', str(count), '-i', '0.2', target_ip],
            capture_output=True, text=True, timeout=count * 0.5 + 15,
        ).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return {'error': str(exc)}

    loss_m = re.search(r'(\d+)% packet loss', out)
    rtt_m = re.search(r'= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)', out)
    result = {
        'packet_loss_pct': float(loss_m.group(1)) if loss_m else None,
        'raw_output': out,
    }
    if rtt_m:
        result.update({
            'rtt_min_ms': float(rtt_m.group(1)),
            'rtt_avg_ms': float(rtt_m.group(2)),
            'rtt_max_ms': float(rtt_m.group(3)),
            'rtt_mdev_ms': float(rtt_m.group(4)),
        })
    return result


def iperf3_test(target_ip, duration, reverse=False):
    cmd = ['iperf3', '-c', target_ip, '-t', str(duration), '-J']
    if reverse:
        cmd.append('--reverse')
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=duration + 15).stdout
        data = json.loads(out)
    except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as exc:
        return {'error': str(exc)}
    if 'error' in data:
        return {'error': data['error']}
    summary = data.get('end', {}).get('sum_received') or data.get('end', {}).get('sum_sent')
    if not summary:
        return {'error': 'no summary in iperf3 output'}
    return {
        'direction': 'pi_download' if reverse else 'pi_upload',
        'bits_per_second': summary['bits_per_second'],
        'mbps': summary['bits_per_second'] / 1e6,
        'retransmits': data.get('end', {}).get('sum_sent', {}).get('retransmits'),
    }


def http_cloud_fetch_test(base_url, n=15):
    """Rough proxy only -- see module docstring for why this isn't a
    substitute for a real browser-side measurement."""
    samples_ms, sizes = [], []
    for _ in range(n):
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(f'{base_url}/merged.bin', timeout=5) as resp:
                body = resp.read()
        except Exception as exc:
            return {'error': str(exc)}
        samples_ms.append((time.monotonic() - t0) * 1000.0)
        sizes.append(len(body))
        time.sleep(0.1)
    return {
        'n_requests': n,
        'latency_ms_mean': statistics.fmean(samples_ms),
        'latency_ms_median': statistics.median(samples_ms),
        'latency_ms_max': max(samples_ms),
        'body_bytes_mean': int(statistics.fmean(sizes)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target_ip', help='IP of the OTHER device (running `iperf3 -s`)')
    ap.add_argument('--duration', type=int, default=20, help='iperf3 test duration, seconds, per direction')
    ap.add_argument('--ping-count', type=int, default=30)
    ap.add_argument('--streamer-url', default='http://localhost:8080',
                     help='base URL of this Pi\'s own fusion_web_streamer, for the HTTP proxy test')
    ap.add_argument('--skip-http', action='store_true')
    ap.add_argument('--skip-iperf3', action='store_true',
                     help='ping + HTTP proxy only -- for a quick check before iperf3/a second device is ready')
    ap.add_argument('--json', default='wifi_test_results.json')
    args = ap.parse_args()

    if not args.skip_iperf3 and not check_iperf3():
        sys.exit(
            'iperf3 not found on this machine. Install it first:\n'
            '  sudo apt install iperf3\n'
            'and make sure the OTHER device is running `iperf3 -s`.\n'
            '(Or pass --skip-iperf3 to run just the ping + HTTP proxy checks.)'
        )

    print(f'Pinging {args.target_ip} ({args.ping_count} packets)...')
    ping = ping_sweep(args.target_ip, args.ping_count)

    if args.skip_iperf3:
        up = down = {'error': 'skipped (--skip-iperf3)'}
    else:
        print(f'iperf3 upload (Pi -> {args.target_ip}), {args.duration}s...')
        up = iperf3_test(args.target_ip, args.duration, reverse=False)

        print(f'iperf3 download ({args.target_ip} -> Pi), {args.duration}s...')
        down = iperf3_test(args.target_ip, args.duration, reverse=True)

    http = None
    if not args.skip_http:
        print(f'HTTP proxy test against {args.streamer_url} (requires the pipeline to be running)...')
        http = http_cloud_fetch_test(args.streamer_url)

    payload = {'target_ip': args.target_ip, 'ping': ping, 'iperf3_upload': up,
               'iperf3_download': down, 'http_proxy': http}
    with open(args.json, 'w') as f:
        json.dump(payload, f, indent=2)

    print('\n--- Ping (round-trip) ---')
    if 'rtt_avg_ms' in ping:
        print(f"  min/avg/max/mdev: {ping['rtt_min_ms']:.2f} / {ping['rtt_avg_ms']:.2f} / "
              f"{ping['rtt_max_ms']:.2f} / {ping['rtt_mdev_ms']:.2f} ms, "
              f"{ping['packet_loss_pct']:.0f}% loss")
    else:
        print(f"  FAILED: {ping.get('error', 'no RTT parsed')}")

    print('\n--- iperf3 throughput ---')
    for label, r in (('upload (Pi -> other)', up), ('download (other -> Pi)', down)):
        if 'mbps' in r:
            print(f'  {label}: {r["mbps"]:.1f} Mbps (retransmits: {r["retransmits"]})')
        else:
            print(f'  {label}: FAILED: {r.get("error")}')

    if http is not None:
        print('\n--- fusion_web_streamer HTTP proxy (this script fetching, not a real browser) ---')
        if 'latency_ms_mean' in http:
            print(f"  /merged.bin: mean={http['latency_ms_mean']:.1f}ms "
                  f"median={http['latency_ms_median']:.1f}ms max={http['latency_ms_max']:.1f}ms, "
                  f"~{http['body_bytes_mean']/1024:.1f}KB/response (gzipped)")
        else:
            print(f"  FAILED: {http.get('error')}")

    print(f'\nWrote {args.json}')


if __name__ == '__main__':
    main()
