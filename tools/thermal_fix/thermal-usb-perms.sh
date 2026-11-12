#!/bin/bash
# Run by udev (as root) when the thermal camera (3474:43e2) is plugged in: give the 'plugdev' group write access to the knobs that can
# reset/power-cycle it, so the user-level watchdog can recover a stuck camera without sudo.  Usage: thermal-usb-perms.sh <devpath-without-/sys>
dev=/sys$1; [ -d "$dev" ] || exit 0
name=$(basename "$dev"); parent=$(dirname "$dev"); hub=$(basename "$parent"); n=${name##*[-.]}
if [[ $hub == usb* ]]; then bus=${hub#usb}; iface="$parent/$bus-0:1.0"; port="$iface/usb$bus-port$n"; else iface="$parent/$hub:1.0"; port="$iface/$hub-port$n"; fi
for f in "$dev/authorized" "$dev/remove" "$port/disable" "$port/power/control"; do [ -e "$f" ] && chgrp plugdev "$f" && chmod g+w "$f"; done
exit 0
