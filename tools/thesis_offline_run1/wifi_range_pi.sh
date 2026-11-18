#!/bin/bash
# Hotspot range test, Pi side. Nothing to install or download on the laptop: after this starts, join the laptop to the
# PitFusion3D WiFi and open  http://10.42.0.1:8082/  in any browser.
#   ./wifi_range_pi.sh                  switch the Pi to its hotspot (10.42.0.1) and start the test page
#   ./wifi_range_pi.sh --ap-band a      same on 5 GHz (default: the profile's 2.4 GHz)
#   ./wifi_range_pi.sh --return-home    (with the above) switch back to home WiFi when stopped
#   ./wifi_range_pi.sh --stop           stop it (or press "finish" on the page)
#   ./wifi_range_pi.sh --status
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ "$1" = "--stop" ]; then mkdir -p "$HERE/wifi_range"; touch "$HERE/wifi_range/STOP"; echo "Stopping (logs zip appears in $HERE/wifi_range/)."; exit 0; fi
if [ "$1" = "--status" ]; then systemctl --user is-active thesis-wifirange.service; ls -td "$HERE"/wifi_range/*/ 2>/dev/null | head -1; exit 0; fi
if systemctl --user is-active --quiet thesis-wifirange.service; then echo "Already running. Stop it first: $0 --stop"; exit 1; fi
command -v iw >/dev/null || echo "NOTE: 'iw' is not installed on the Pi, so the signal strength of the laptop as seen by the Pi is not logged. One-time fix:  sudo apt install -y iw   then start again."
systemd-run --user --unit=thesis-wifirange --collect python3 -u "$HERE/wifi_range_pi.py" "$@" >/dev/null 2>&1 || { echo "could not start"; exit 1; }
echo "Started. In about 30-60 s the Pi is on the PitFusion3D hotspot (10.42.0.1); this terminal's home-WiFi session will drop."
echo "Then on the laptop: join the PitFusion3D WiFi and open  http://10.42.0.1:8082/"
