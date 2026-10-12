#!/bin/bash
# Installs the pitfusion3d.service systemd --user unit and enables
# lingering so the pipeline starts automatically at boot with no login,
# keyboard, or mouse -- see this package's README, "Auto-start on boot".
#
# Safe to re-run any time (e.g. after editing the unit file).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

mkdir -p ~/.config/systemd/user
cp "$PKG_DIR/systemd/pitfusion3d.service" ~/.config/systemd/user/pitfusion3d.service

# No sudo needed for either of these -- polkit allows the logged-in user
# to manage their own lingering state and systemd --user units directly.
loginctl enable-linger "$(whoami)"
systemctl --user daemon-reload
systemctl --user enable pitfusion3d.service

echo
echo "Installed. The pipeline will now start automatically on every boot,"
echo "with no login required."
echo
echo "Start it now without rebooting:  systemctl --user start pitfusion3d.service"
echo "Check status:                    systemctl --user status pitfusion3d.service"
echo "Watch logs live:                 journalctl --user -u pitfusion3d.service -f"
echo "Stop it:                         systemctl --user stop pitfusion3d.service"
echo "Disable auto-start again:        systemctl --user disable pitfusion3d.service"
