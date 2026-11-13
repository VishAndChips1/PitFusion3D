import sys, time; sys.argv=['x']; sys.path.insert(0,'/home/vish/thermal_fix')
import thermal_watchdog as t
t.log('LONG vbus test: all onboard USB off for 300 s'); t.vbus_cycle(300); t.log('LONG vbus test: ports re-enabled')
