#!/usr/bin/env python3
"""Keep the Pi on WiFi: switch the radio back ON if something turned it off, and ask NetworkManager to reconnect
if the Pi has been offline for a while. It never turns WiFi off and never names a network (NetworkManager chooses the
best saved profile, so a hotspot profile still works). Acts only after a grace period, and never faster than every 40 s,
because routers often refuse re-joins that come too quickly.
"""
import subprocess
import sys
import time

IFACE = "wlan0"
POLL_S = 10
RADIO_GRACE_S = 20          # radio must have been off this long before we switch it on (a quick toggle is left alone)
OFFLINE_GRACE_S = 45        # offline this long before asking NetworkManager to reconnect
RETRY_GAP_S = 40            # at most one reconnect request per this many seconds
PROFILE = "Patelnet"        # the saved connection whose access-point pin (if any) this service may remove
PIN_GIVE_UP_S = 150         # offline this long while pinned to one access point: remove the pin so any access point can be used


def run(cmd, timeout=35):
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return res.returncode, (res.stdout + res.stderr).strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, str(exc)


def log(msg):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}", flush=True)


class WifiKeepalive:
    def __init__(self, runner=run, clock=time.monotonic, logger=log):
        self.run, self.clock, self.log = runner, clock, logger
        self.radio_off_since = None
        self.offline_since = None
        self.last_attempt = None

    def pinned(self):
        code, out = self.run(["nmcli", "-g", "802-11-wireless.bssid", "connection", "show", PROFILE])
        return code == 0 and bool(out.strip())

    def radio_enabled(self):
        code, out = self.run(["nmcli", "radio", "wifi"])
        return None if code != 0 else out.strip().lower() == "enabled"

    def connected(self):
        code, out = self.run(["nmcli", "-t", "-g", "GENERAL.STATE", "device", "show", IFACE])
        return None if code != 0 else out.strip().startswith("100")

    def step(self):
        """One check. Returns a short string naming the action taken (or '')."""
        now = self.clock()
        radio = self.radio_enabled()
        if radio is None:                       # nmcli itself failed: say nothing, try again next time
            return ""
        if not radio:
            self.offline_since = self.offline_since or now
            self.radio_off_since = self.radio_off_since or now
            if now - self.radio_off_since >= RADIO_GRACE_S:
                self.log("WiFi radio has been off for %.0f s: switching it on" % (now - self.radio_off_since))
                self.run(["nmcli", "radio", "wifi", "on"])
                self.radio_off_since = None
                self.last_attempt = now
                return "radio-on"
            return ""
        self.radio_off_since = None
        up = self.connected()
        if up is None:
            return ""
        if up:
            if self.offline_since is not None:
                self.log("back online after %.0f s" % (now - self.offline_since))
            self.offline_since = self.last_attempt = None
            return ""
        self.offline_since = self.offline_since or now
        if now - self.offline_since >= PIN_GIVE_UP_S and self.pinned():
            self.log("offline for %.0f s while pinned to one access point: removing the pin so any access point can be used" % (now - self.offline_since))
            self.run(["nmcli", "connection", "modify", PROFILE, "802-11-wireless.bssid", ""])
        if now - self.offline_since >= OFFLINE_GRACE_S and (self.last_attempt is None or now - self.last_attempt >= RETRY_GAP_S):
            self.log("offline for %.0f s: asking NetworkManager to reconnect" % (now - self.offline_since))
            code, out = self.run(["nmcli", "device", "connect", IFACE])
            self.log("reconnect request: " + (out.splitlines()[-1] if out else f"rc={code}"))
            self.last_attempt = now
            return "reconnect"
        return ""


def main():
    keep = WifiKeepalive()
    log("wifi keepalive started")
    while True:
        try:
            keep.step()
        except Exception as exc:                 # a bad check must never end the loop
            log(f"check failed: {exc!r}")
        time.sleep(POLL_S)


if __name__ == "__main__":
    sys.exit(main())
