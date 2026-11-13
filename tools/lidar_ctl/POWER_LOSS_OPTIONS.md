# LiDAR when the Pi loses power or is unplugged: what is possible

## Why software alone cannot do it
The command "go to Standby" has to be sent by something that is still running. When the Pi's supply disappears the CPU stops in about a millisecond (the Pi 5 has almost no stored energy and no power-fail warning pin). Measured on this rig: if the controlling process just vanishes, the LiDAR keeps its Normal mode, stops sending data after 3.2 s, and waits indefinitely (`data/lidar_sudden_loss.json`). The Pi's own 5 V sensor (EXT5V, readable about 65 times a second) gives no usable warning; a recorded power-bank death showed a flat 5.0 V up to the last sample.
**What IS handled in software** (installed, tested on a real reboot at 12:47 on 2026-11-13): a normal shutdown/reboot/power-button press puts the LiDAR in Standby, and boot wakes it.

## Measure your own supply (optional, 2 minutes)
`python3 ~/lidar_ctl/unplug_probe.py` (logs the 5 V rail, ~65 samples/s, each sample forced to the SD card), pull the plug, power up, then `python3 ~/lidar_ctl/unplug_probe.py --report`. If the report shows a sag below 4.8 V for tens of milliseconds, a fast software trigger could be built for that supply; if it says NONE (expected for a power bank), only hardware helps.

## Hardware options (best first)
1. **Relay / MOSFET switch in the LiDAR's own supply, controlled by the Pi's 5 V.** Use a normally-open DC relay (or a logic-level MOSFET / solid-state relay) in the LiDAR's DC feed with its coil driven from the Pi's 5 V pin. Pi loses power -> coil drops -> the LiDAR is cut in milliseconds (off is safer than Standby). Pi powered -> LiDAR powered. Drive the coil from a GPIO instead of the raw 5 V pin if you also want it off after a halted shutdown (the existing service already handles that case with Standby). Cost a few dollars. Rate the relay for the Avia's supply (9-27 V DC, about 10 W) and check polarity/connector; the LiDAR needs its start-up time after power returns (the pipeline already waits for it).
2. **Pi 5 UPS HAT or battery HAT with a power-loss signal.** Gives the Pi seconds to minutes after the supply goes. Software side: on the HAT's power-loss signal run `~/lidar_ctl/lidar_power_hook.sh stop` (stops the pipeline, sends Standby) and then `systemctl poweroff`. I can wire this up once you pick a specific HAT (each has its own signal pin or I2C register).
3. **Supercapacitor / hold-up module on the Pi's 5 V input** with a fast power-fail detector. Works but needs electronics design; a UPS HAT is simpler.
4. **A second always-powered device on the LiDAR's supply** (for example a small board that pings the Pi and commands Standby when the Pi stops answering). Reliable but the most complex.
Not possible: any approach that depends on the Pi running after the power is gone.
