# Hotspot at power-on (demo mode)

When the Pi is powered on it creates its own WiFi network and starts streaming on it. No keyboard, screen, router or internet is needed.

| | |
|---|---|
| WiFi name (SSID) | `PitFusion3D` |
| Password | the one stored in the `PitFusion3D-AP` NetworkManager profile (see `pitfusion3d_bringup/scripts/start_hotspot_viewing.sh`) |
| Pi address | **10.42.0.1** (fixed: the Pi is the router, so it never changes) |
| Viewer | `http://10.42.0.1:8080/` |
| RGB focus page | `http://10.42.0.1:8080/focus` |
| Base station | scan the network or enter `10.42.0.1` |
| Control (Sleep/Shutdown) | `http://10.42.0.1:8081/` |

Ready about 30 s after power-on (hotspot about 10 s, first data about 20 s later).

## How it works

* `pitfusion3d-hotspot.service` (user service, runs at boot) calls `hotspot_boot.sh`, which brings `PitFusion3D-AP` up on 2.4 GHz channel 11, checks it has 10.42.0.1, and falls back to channel 6, channel 1, then automatic if needed. It always exits successfully so the pipeline starts regardless; result in `~/.local/share/pitfusion3d-hotspot/status.json`.
* `pitfusion3d.service` has `After=`/`Wants=` that service (drop-in `hotspot-first.conf`), so the pipeline always starts on the network that is actually up.
* In this mode the home WiFi profiles do not auto-connect and `pitfusion3d-wifi.service` (home WiFi keep-alive) is disabled, so nothing can pull the Pi off its hotspot.
* Fixed address: NetworkManager "shared" mode gives the Pi `10.42.0.1/24` and hands out `10.42.0.x` to clients.

## Switching (command `~/wifi_mode`)

    wifi_mode status           mode, active network, address, result of the last boot
    wifi_mode hotspot [--now]  demo mode (power-on = own hotspot)
    wifi_mode home [--now]     development mode (power-on = home WiFi Patelnet, internet available)
    wifi_mode band 5           hotspot on 5 GHz channel 149 (less crowded, needs a 5 GHz laptop); `band 2.4` returns to channel 11

Switching to hotspot mode takes the Pi off the internet until `wifi_mode home`.

Install on a fresh Pi: `./install.sh`.
