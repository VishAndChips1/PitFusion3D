#!/bin/bash
# Pi side of the hotspot range test (run this on the Pi; it keeps running in the background until you stop it).
#   ./wifi_range_pi.sh                  switch the Pi to its PitFusion3D hotspot (10.42.0.1) and start the test server
#   ./wifi_range_pi.sh --ap-band a      same, but on 5 GHz (default is the profile's 2.4 GHz)
#   ./wifi_range_pi.sh --stop           stop it (add --return-home when starting to switch back to home WiFi at the end)
#   ./wifi_range_pi.sh --status
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ "$1" = "--stop" ]; then mkdir -p "$HERE/wifi_range"; touch "$HERE/wifi_range/STOP"; echo "Stopping (logs zip appears in $HERE/wifi_range/)."; exit 0; fi
if [ "$1" = "--status" ]; then systemctl --user is-active thesis-wifirange.service; ls -td "$HERE"/wifi_range/*/ 2>/dev/null | head -1; exit 0; fi
if systemctl --user is-active --quiet thesis-wifirange.service; then echo "Already running. Stop it first: $0 --stop"; exit 1; fi
command -v iperf3 >/dev/null || echo "NOTE: iperf3 is not installed on the Pi. For the iperf3 numbers run once:  sudo apt install -y iperf3 iw   (then start again). Without it the application-traffic test still runs."
command -v iw >/dev/null || echo "NOTE: 'iw' is not installed: the Pi-side client signal/retry log is skipped (sudo apt install -y iw)."
systemd-run --user --unit=thesis-wifirange --collect python3 -u "$HERE/wifi_range_pi.py" "$@" >/dev/null 2>&1 || { echo "could not start"; exit 1; }
echo "Started. In about 30-60 s the Pi is on the PitFusion3D hotspot (10.42.0.1)."
echo "Check it is ready:  cat $(ls -td "$HERE"/wifi_range/*/ 2>/dev/null | head -1)READY.json   (appears when ready; your home WiFi session will drop)."
