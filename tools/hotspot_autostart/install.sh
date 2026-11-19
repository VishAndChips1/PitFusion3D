#!/bin/bash
# Install the hotspot-at-power-on files for the current user (does not change the WiFi mode; run `wifi_mode hotspot` for that).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p ~/.local/share/pitfusion3d-hotspot ~/.config/systemd/user/pitfusion3d.service.d ~/.config/pitfusion3d
cp "$HERE/hotspot_boot.sh" ~/.local/share/pitfusion3d-hotspot/hotspot_boot.sh
cp "$HERE/pitfusion3d-hotspot.service" ~/.config/systemd/user/
cp "$HERE/hotspot-first.conf" ~/.config/systemd/user/pitfusion3d.service.d/hotspot-first.conf
cp "$HERE/wifi_mode" ~/wifi_mode
chmod +x ~/.local/share/pitfusion3d-hotspot/hotspot_boot.sh ~/wifi_mode
systemctl --user daemon-reload
echo "Installed. Turn on demo mode with:  ~/wifi_mode hotspot"
