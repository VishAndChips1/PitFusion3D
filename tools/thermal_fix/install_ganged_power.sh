#!/bin/bash
# ONE-TIME setup (needs sudo): lets the thermal watchdog (user vish, group plugdev) switch off/on VBUS for the Pi 5's whole onboard USB group
# (the only software way to genuinely power-cycle the thermal camera).  Does NOT change anything right now except file permissions.
#   sudo bash ~/thermal_fix/install_ganged_power.sh
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo:  sudo bash $0"; exit 1; }
install -m 755 /home/vish/thermal_fix/thermal-usb-onboard-perms.sh /usr/local/sbin/thermal-usb-onboard-perms.sh
cat > /etc/udev/rules.d/98-usb-onboard-ports.rules <<'R'
# Pi 5 onboard USB root hubs: user-writable port 'disable' knobs (see /usr/local/sbin/thermal-usb-onboard-perms.sh)
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="1d6b", ATTR{bDeviceClass}=="09", RUN+="/usr/local/sbin/thermal-usb-onboard-perms.sh %p"
R
udevadm control --reload-rules
for d in /sys/bus/usb/devices/usb*; do /usr/local/sbin/thermal-usb-onboard-perms.sh "$(readlink -f $d | sed 's#^/sys##')"; done
id vish | grep -q plugdev || { echo "WARNING: vish is not in group plugdev"; }
echo "done. Test with:  python3 ~/thermal_fix/thermal_watchdog.py --vbus-test   (briefly disconnects USB keyboard/mouse and the thermal camera)"
