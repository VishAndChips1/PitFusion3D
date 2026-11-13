#!/bin/bash
# Run by udev (as root) when an onboard (RP1 xHCI) root hub appears: let group plugdev write the per-port 'disable' knobs, so the user-level
# thermal watchdog can cut VBUS to the whole Pi 5 USB group (all four hubs must be disabled together).  Usage: <script> <devpath-without-/sys>
dev=/sys$1; [ -d "$dev" ] || exit 0
case "$(readlink -f "$dev")" in *1000120000.pcie*) ;; *) exit 0;; esac
for f in "$dev"/*-0:1.0/*-port*/disable; do [ -e "$f" ] && chgrp plugdev "$f" && chmod g+w "$f"; done
exit 0
