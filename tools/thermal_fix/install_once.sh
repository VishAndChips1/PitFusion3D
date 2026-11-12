#!/bin/bash
# ONE-TIME setup (needs sudo).  Lets the thermal watchdog (runs as user vish) power-cycle/reset the thermal camera's USB port.
#   sudo bash ~/thermal_fix/install_once.sh
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo:  sudo bash $0"; exit 1; }
install -m 755 /home/vish/thermal_fix/thermal-usb-perms.sh /usr/local/sbin/thermal-usb-perms.sh
cat > /etc/udev/rules.d/99-thermal-camera.rules <<'R'
# thermal camera 3474:43e2: user-writable device node + reset/power knobs (see /usr/local/sbin/thermal-usb-perms.sh)
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="3474", ATTR{idProduct}=="43e2", MODE="0664", GROUP="plugdev", RUN+="/usr/local/sbin/thermal-usb-perms.sh %p"
R
udevadm control --reload-rules
# apply immediately to the camera that is already plugged in (does NOT reset it)
for d in /sys/bus/usb/devices/*; do
  [ "$(cat $d/idVendor 2>/dev/null)" = 3474 ] && [ "$(cat $d/idProduct 2>/dev/null)" = 43e2 ] && { /usr/local/sbin/thermal-usb-perms.sh "$(readlink -f $d | sed 's#^/sys##')"; chgrp plugdev /dev/bus/usb/$(printf %03d $(cat $d/busnum))/$(printf %03d $(cat $d/devnum)); chmod 664 /dev/bus/usb/$(printf %03d $(cat $d/busnum))/$(printf %03d $(cat $d/devnum)); echo "permissions applied to $d"; }
done
echo "done. The thermal watchdog (systemctl --user status thermal-watchdog) can now reset the camera automatically."
