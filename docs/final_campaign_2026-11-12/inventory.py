#!/usr/bin/env python3
"""Collect a wide static inventory of the system into data/inventory.json (name -> {cmd, output})."""
import sys, os, json, glob, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import *
R = '/home/vish/ros2_ws/src'; REPO = '/home/vish/PitFusion3D_repo'
CMDS = [
 ('hw.model', 'cat /proc/device-tree/model; echo; cat /proc/device-tree/serial-number 2>/dev/null | tr -d "\\0" | sed "s/./X/1;s/.*\\(....\\)$/serial-ends-\\1/"'),
 ('hw.lscpu', 'lscpu'), ('hw.cpuinfo_tail', 'tail -n 8 /proc/cpuinfo'), ('hw.meminfo', 'head -n 12 /proc/meminfo'),
 ('hw.vcgencmd_version', 'vcgencmd version'), ('hw.bootloader', 'vcgencmd bootloader_version'), ('hw.config_int', 'vcgencmd get_config int'), ('hw.config_str', 'vcgencmd get_config str'),
 ('hw.volts', 'for r in core sdram_c sdram_i sdram_p; do vcgencmd measure_volts $r; done'), ('hw.mem_split', 'vcgencmd get_mem arm; vcgencmd get_mem gpu'),
 ('hw.clocks', 'for c in arm core h264 isp v3d uart pwm emmc pixel vec hdmi dpi; do vcgencmd measure_clock $c; done'),
 ('hw.cpufreq', 'cd /sys/devices/system/cpu/cpu0/cpufreq; for f in scaling_governor scaling_min_freq scaling_max_freq cpuinfo_max_freq scaling_available_frequencies; do echo $f=$(cat $f); done'),
 ('hw.boot_config_txt', 'grep -vE "^\\s*#|^\\s*$" /boot/firmware/config.txt'), ('hw.cmdline', 'cat /proc/cmdline'),
 ('hw.thermal_zones', 'for z in /sys/class/thermal/thermal_zone*; do echo $z $(cat $z/type) $(cat $z/temp); done; for t in /sys/class/thermal/thermal_zone0/trip_point_*; do echo $t $(cat $t); done'),
 ('hw.cooling', 'for c in /sys/class/thermal/cooling_device*; do echo $c $(cat $c/type) cur=$(cat $c/cur_state) max=$(cat $c/max_state); done; cat /sys/class/hwmon/hwmon*/name | tr "\\n" " "; echo; cat /sys/class/hwmon/hwmon2/fan1_input /sys/class/hwmon/hwmon2/pwm1 2>/dev/null'),
 ('hw.power_dt', 'cd /proc/device-tree/chosen/power && for f in *; do echo $f: $(xxd -p $f | head -c 120); done'),
 ('hw.pmic_adc', 'vcgencmd pmic_read_adc'), ('hw.throttled', 'vcgencmd get_throttled; vcgencmd measure_temp'), ('hw.hwmon_volt', 'cat /sys/class/hwmon/hwmon3/in0_lcrit_alarm 2>/dev/null; ls /sys/class/hwmon/hwmon3'),
 ('hw.eeprom', 'vcgencmd bootloader_config 2>/dev/null | head -30'),
 ('os.uname', 'uname -a; cat /etc/os-release | head -4; python3 --version'), ('os.boot', 'systemd-analyze 2>&1; systemd-analyze blame 2>&1 | head -15'),
 ('os.user_boot', 'systemd-analyze --user blame 2>&1 | head -15'), ('os.uptime', 'uptime; who -b'),
 ('os.sysctl_net', 'sysctl net.core.rmem_max net.core.rmem_default net.core.wmem_max net.core.netdev_max_backlog net.ipv4.udp_mem net.ipv4.udp_rmem_min kernel.sched_rt_runtime_us vm.swappiness'),
 ('os.limits', 'ulimit -a'), ('os.swap', 'swapon --show; cat /proc/swaps'),
 ('storage.df', 'df -h; lsblk'), ('storage.mount', 'findmnt / -o SOURCE,FSTYPE,OPTIONS'), ('storage.shm', 'df -h /dev/shm; ls /dev/shm | wc -l; du -sh /dev/shm'),
 ('net.addr', 'ip -br addr; ip route'), ('net.nm_dev', 'nmcli -f GENERAL,IP4,WIFI-PROPERTIES dev show wlan0 2>&1 | head -40'), ('net.nm_conns', 'nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY,DEVICE connection show'),
 ('net.wifi_link', 'iwconfig wlan0 2>&1; cat /proc/net/wireless'), ('net.wifi_scan', 'nmcli -f IN-USE,SSID,BSSID,CHAN,FREQ,RATE,SIGNAL,SECURITY dev wifi list 2>&1 | head -20'),
 ('net.eth0', 'ip -s link show eth0; cat /sys/class/net/eth0/speed /sys/class/net/eth0/duplex'), ('net.udp_snmp', 'grep -E "^Udp" /proc/net/snmp'), ('net.sockets', 'ss -tlnp 2>/dev/null | head -20'),
 ('usb.tree', 'lsusb -t; lsusb'), ('usb.thermal_desc', 'lsusb -v -d 3474:43e2 2>&1 | head -120'), ('usb.ports', 'for p in /sys/bus/usb/devices/usb*/*-port*; do echo $(basename $p) oc=$(cat $p/over_current_count) disable=$(cat $p/disable); done'),
 ('video.list', 'v4l2-ctl --list-devices 2>&1 | head -40'), ('video.thermal_fmt', 'v4l2-ctl -d /dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0 --list-formats-ext 2>&1 | head -20'), ('video.cameras', 'timeout 15 rpicam-hello --list-cameras 2>&1 | head -30'),
 ('sw.pip', 'pip list 2>/dev/null | grep -iE "numpy|scipy|opencv|psutil|matplotlib|pyyaml|transforms3d|message|cv-bridge|setuptools"'),
 ('sw.ros_pkgs', 'dpkg -l | grep -E "ros-jazzy-(rclpy|rclcpp |sensor-msgs|tf2-ros|cv-bridge|message-filters|rmw-fastrtps-cpp|fastrtps|rmw-implementation|ros-base|launch-ros|livox)" | awk \'{print $2, $3}\''),
 ('sw.ros_env', 'printenv | grep -E "ROS|RMW|FASTRTPS|AMENT" | sort'), ('sw.fastdds_xml', 'cat /home/vish/ros2_ws/src/pitfusion3d_bringup/config/fastdds_large_buffers.xml'),
 ('sw.ffmpeg', 'ffmpeg -version | head -1; rpicam-vid --version 2>&1 | head -3'),
 ('ros.nodes', f'{ROS} ros2 node list'), ('ros.topics', f'{ROS} ros2 topic list -t'),
 ('ros.qos_lidar', f'{ROS} ros2 topic info -v /livox/lidar | grep -E "Node name|Reliab|Durab|History|Depth|Topic type|count"'), ('ros.qos_fused', f'{ROS} ros2 topic info -v /landfill/fusion/points_rgbt | grep -E "Node name|Reliab|Durab|History|Depth|count"'),
 ('ros.qos_rgb', f'{ROS} ros2 topic info -v /landfill/rgb/image_raw | grep -E "Node name|Reliab|Durab|History|Depth|count"'), ('ros.qos_merged', f'{ROS} ros2 topic info -v /landfill/fusion/points_merged | grep -E "Node name|Reliab|Durab|History|Depth|count"'),
 ('proc.tree', 'ps -eo pid,ppid,nlwp,ni,psr,cls,rtprio,pcpu,pmem,rss,etimes,args --sort=pid | grep -E "livox|restamp|rgb_camera|thermal_camera|calibration|sensor_fusion|registration_merge|fusion_web|rpicam|ros2 launch|power_logger|usb_watch|run_pipeline|code " | cut -c1-200'),
 ('proc.affinity_sched', 'for p in $(pgrep -f "lib/(livox_ros2_avia|pitfusion3d_bringup|landfill_camera_sim)/|rpicam-vid"); do echo "pid $p: $(taskset -cp $p 2>&1 | cut -d: -f2) | $(chrt -p $p 2>&1 | tr "\\n" " ") | nice=$(ps -o ni= -p $p)"; done'),
 ('systemd.unit', 'systemctl --user cat pitfusion3d.service; systemctl --user show pitfusion3d.service -p ActiveState,SubState,NRestarts,MainPID,MemoryCurrent,CPUUsageNSec,TasksCurrent,KillMode,TimeoutStopUSec,Restart,RestartUSec'), ('systemd.linger', 'loginctl show-user vish | grep -E "Linger|State"'),
 ('systemd.cgroup', 'systemd-cgls --user-unit pitfusion3d.service 2>&1 | head -40'),
 ('cfg.run_pipeline', f'cat {R}/pitfusion3d_bringup/scripts/run_pipeline.sh'), ('cfg.launch_full', f'cat {R}/pitfusion3d_bringup/launch/pitfusion3d_full.launch.py'), ('cfg.launch_sensors', f'cat {R}/pitfusion3d_bringup/launch/real_sensors.launch.py'),
 ('cfg.extrinsics', f'cat {R}/landfill_camera_sim/config/calibration/extrinsics.yaml'), ('cfg.intrinsics', f'cat {R}/landfill_camera_sim/config/calibration/intrinsics.yaml'),
 ('cfg.livox', f'cat {R}/pitfusion3d_bringup/config/livox_lidar_config.json 2>/dev/null || find {R} -name "*livox*config*.json" | head'),
 ('git.log', f'cd {REPO}; git log --format="%h %ad %s" --date=short | head -40; echo; git branch -a; git remote -v'), ('git.stats', f'cd {REPO}; git ls-files | wc -l; git shortlog -sn | head'),
]
def params(node):
    r = sh(f'{ROS} ros2 param dump {node}', 40); return r.stdout[:6000]
if __name__ == '__main__':
    out = {}; t0 = time.time()
    for name, cmd in CMDS:
        r = sh(cmd, 60); out[name] = dict(cmd=cmd if len(cmd) < 300 else cmd[:300] + '...', rc=r.returncode, output=(r.stdout + (('\n[stderr] ' + r.stderr) if r.stderr.strip() else ''))[:30000])
    nodes = [l.strip() for l in out['ros.nodes']['output'].splitlines() if l.strip().startswith('/')]
    for nd in nodes: out['ros.params' + nd] = dict(cmd=f'ros2 param dump {nd}', rc=0, output=params(nd))
    save_json('inventory', out); log(f'inventory: {len(out)} items in {time.time()-t0:.0f}s')
