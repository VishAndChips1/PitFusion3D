"""Logic tests with a fake nmcli and a fake clock: no real WiFi is touched."""
import unittest
from wifi_keepalive import WifiKeepalive, OFFLINE_GRACE_S, RADIO_GRACE_S, RETRY_GAP_S, PIN_GIVE_UP_S


class Fake:
    def __init__(self):
        self.t = 0.0
        self.radio = "enabled"
        self.state = "100 (connected)"
        self.pin = ""
        self.calls = []

    def run(self, cmd, timeout=35):
        self.calls.append(cmd)
        if cmd[:3] == ["nmcli", "radio", "wifi"] and len(cmd) == 3:
            return 0, self.radio
        if cmd[:4] == ["nmcli", "radio", "wifi", "on"]:
            self.radio = "enabled"
            return 0, ""
        if "GENERAL.STATE" in cmd:
            return 0, self.state
        if cmd[:3] == ["nmcli", "device", "connect"]:
            return 0, "Device 'wlan0' successfully activated"
        if "802-11-wireless.bssid" in cmd and "show" in cmd:
            return 0, self.pin
        if "802-11-wireless.bssid" in cmd and "modify" in cmd:
            self.pin = ""
            return 0, ""
        return 1, "unexpected"

    def keep(self):
        return WifiKeepalive(runner=self.run, clock=lambda: self.t, logger=lambda m: None)


class KeepaliveTests(unittest.TestCase):
    def advance(self, f, k, seconds, step=10):
        acts = []
        for _ in range(int(seconds / step)):
            f.t += step
            acts.append(k.step())
        return acts

    def test_does_nothing_while_online(self):
        f = Fake(); k = f.keep()
        self.assertEqual(set(self.advance(f, k, 300)), {""})
        self.assertFalse(any(c[:4] == ["nmcli", "radio", "wifi", "on"] or c[:3] == ["nmcli", "device", "connect"] for c in f.calls))

    def test_a_quick_radio_toggle_is_left_alone_but_a_long_one_is_fixed(self):
        f = Fake(); k = f.keep()
        f.radio = "disabled"; f.t += 10; self.assertEqual(k.step(), "")           # off for 0 s
        f.radio = "enabled"; f.t += 10; self.assertEqual(k.step(), "")            # back on by itself
        f.radio = "disabled"
        acts = self.advance(f, k, RADIO_GRACE_S + 20)
        self.assertIn("radio-on", acts)
        self.assertEqual(f.radio, "enabled")

    def test_reconnect_is_requested_only_after_the_grace_period_and_not_more_often_than_the_gap(self):
        f = Fake(); k = f.keep(); f.state = "30 (disconnected)"
        acts = self.advance(f, k, OFFLINE_GRACE_S - 10)
        self.assertNotIn("reconnect", acts)
        acts = self.advance(f, k, 200)
        self.assertEqual(acts.count("reconnect"), 200 // RETRY_GAP_S or acts.count("reconnect"))
        stamps = [i for i, a in enumerate(acts) if a == "reconnect"]
        self.assertTrue(all(b - a >= RETRY_GAP_S // 10 for a, b in zip(stamps, stamps[1:])))

    def test_it_never_turns_wifi_off_and_never_names_a_network(self):
        f = Fake(); k = f.keep(); f.state = "30 (disconnected)"
        self.advance(f, k, 400)
        for cmd in f.calls:
            self.assertNotIn("off", cmd)
            self.assertFalse(cmd[:3] == ["nmcli", "connection", "up"])      # it never picks a network to join
            if cmd[:3] == ["nmcli", "device", "connect"]:
                self.assertEqual(cmd, ["nmcli", "device", "connect", "wlan0"])   # NetworkManager chooses the best one
        self.assertIn(["nmcli", "device", "connect", "wlan0"], f.calls)

    def test_a_pinned_access_point_is_released_only_after_a_long_outage(self):
        f = Fake(); k = f.keep(); f.state = "30 (disconnected)"; f.pin = "14:3F:C3:A9:50:27"
        self.advance(f, k, PIN_GIVE_UP_S - 20)
        self.assertEqual(f.pin, "14:3F:C3:A9:50:27")        # still within the grace: keep trying the preferred one
        self.advance(f, k, 60)
        self.assertEqual(f.pin, "")                          # released, so any access point can be used

    def test_an_unpinned_profile_is_left_alone(self):
        f = Fake(); k = f.keep(); f.state = "30 (disconnected)"
        self.advance(f, k, 400)
        self.assertFalse(any("modify" in c for c in f.calls))

    def test_a_failing_nmcli_is_ignored(self):
        k = WifiKeepalive(runner=lambda c, timeout=35: (127, "nmcli: not found"), clock=lambda: 0.0, logger=lambda m: None)
        self.assertEqual(k.step(), "")


if __name__ == "__main__":
    unittest.main()
