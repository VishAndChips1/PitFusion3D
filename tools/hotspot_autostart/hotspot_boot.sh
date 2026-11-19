#!/bin/bash
# Bring the Pi's own WiFi hotspot (PitFusion3D-AP, fixed address 10.42.0.1) up at boot, before the pipeline starts.
# Mode comes from ~/.config/pitfusion3d/wifi_mode ("hotspot" = this script acts, "home" = it does nothing).
# Band/channel come from ~/.config/pitfusion3d/hotspot.conf. If the preferred channel fails it falls back (2.4 GHz ch 6, ch 1, automatic).
# Always exits 0 so the pipeline starts even if the hotspot could not be brought up. Result: ~/.local/share/pitfusion3d-hotspot/status.json
PROFILE="PitFusion3D-AP"
CONF="$HOME/.config/pitfusion3d"
OUT="$HOME/.local/share/pitfusion3d-hotspot"
mkdir -p "$OUT"
MODE=$(cat "$CONF/wifi_mode" 2>/dev/null || echo hotspot)
BAND=bg; CHANNEL=11
[ -f "$CONF/hotspot.conf" ] && . "$CONF/hotspot.conf"
say() { echo "[$(date '+%F %T')] $*"; }
status() { printf '{"time":"%s","mode":"%s","result":"%s","band":"%s","channel":"%s","ip":"10.42.0.1","attempts":%s}\n' "$(date '+%F %T')" "$MODE" "$1" "$2" "$3" "${4:-0}" > "$OUT/status.json"; }
[ "$MODE" = "home" ] && { say "mode=home: hotspot not started"; status home "" "" 0; exit 0; }

for i in $(seq 1 45); do nmcli -t -g RUNNING general 2>/dev/null | grep -q running && break; sleep 2; done
nmcli radio wifi on 2>/dev/null
is_up() {
  [ "$(nmcli -g GENERAL.STATE device show wlan0 2>/dev/null | cut -c1-3)" = "100" ] && ip -4 -br addr show wlan0 2>/dev/null | grep -q "10.42.0.1/" \
    && [ "$(nmcli -g GENERAL.CONNECTION device show wlan0 2>/dev/null)" = "$PROFILE" ]
}
attempt=0
for plan in "$BAND $CHANNEL" "$BAND $CHANNEL" "bg 6" "bg 1" "bg 0"; do
  set -- $plan; b=$1; c=$2; attempt=$((attempt+1))
  nmcli connection modify "$PROFILE" 802-11-wireless.band "$b" 802-11-wireless.channel "$c" 802-11-wireless.powersave 2 connection.autoconnect yes 2>/dev/null
  say "attempt $attempt: band=$b channel=$c"
  if is_up; then say "already up"; else timeout 60 nmcli connection up "$PROFILE" >/dev/null 2>&1; sleep 3; fi
  for w in 1 2 3 4 5 6; do is_up && break; sleep 2; done
  if is_up; then say "hotspot up on 10.42.0.1 (band=$b channel=$c)"; status up "$b" "$c" "$attempt"; exit 0; fi
done
say "hotspot did NOT come up after $attempt attempts"; status FAILED "$BAND" "$CHANNEL" "$attempt"
exit 0
