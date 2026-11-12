# PitFusion3D: status in plain language (2026-11-13)

**What works.** The rig builds a live 3D picture (24 000 points, 10 times a second) and a merged picture of the surroundings (about 1.5 times a second). A viewer sees live data 0.08 s old and merged data 0.5 s old. It starts producing data 25 s after power-on, survives crashed components in 2 s, and uses about 4 W (peak 2 A of the 5 A limit).

**Accuracy.** The scan-to-scan alignment error was cut from 32 mm to 18 mm today with a setting change that costs a little CPU. The LiDAR's own noise is about 17 mm and stayed constant for 25 minutes of warm-up.

**Heat.** Under normal operation the Pi sits at about 70 C; the chip begins to slow itself down at 80 C. A deliberate 25-minute overload pushed it to 84 C: it slowed by under 8 %, kept running and cooled back in 8 minutes. There is about 10 C of headroom in this room; a hot environment would remove it, so add a heatsink or shade for field use.

**LiDAR power.** When the Pi is switched off normally, the LiDAR is now put to sleep automatically, and it wakes when the Pi boots (adds 5 s to boot). If the power is simply cut, no software can help, because nothing is running to tell the LiDAR; a relay that cuts the LiDAR's supply along with the Pi's would solve it.

**What needs action.**
1. Thermal camera: it can get stuck and then only comes back after power is removed from it. It is not caused by heat. Needs a switchable USB power line (or replug/replace).
2. Colour alignment: could not be verified in this room; a checkerboard test would settle it.
3. WiFi in the rig room is weak; remote viewing is limited by the radio.
4. Recording all raw sensor streams to the SD card would run at the card's limit; record the processed clouds instead (4.9-21 GB per hour).

**Battery.** About 14 hours on 100 Wh with the LiDAR asleep, about 6 hours with it scanning (conservative).
