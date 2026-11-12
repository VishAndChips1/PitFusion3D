#!/usr/bin/env python3
"""Isolated DDS transport test: publish 576 KB clouds @10 Hz for N s between two processes; report delivery and loopback UDP volume.
Usage: transport_test.py <profile.xml|none> <seconds> [domain]"""
import os, sys, time, json, subprocess, numpy as np
prof = sys.argv[1]; secs = float(sys.argv[2]); dom = sys.argv[3] if len(sys.argv) > 3 else '81'
env = dict(os.environ, ROS_DOMAIN_ID=dom, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
if prof != 'none': env['FASTRTPS_DEFAULT_PROFILES_FILE'] = prof
else: env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
SUB = '''
import time,sys,json,rclpy,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
rclpy.init();n=Node('t_sub');st=[]
n.create_subscription(PointCloud2,'/tt/cloud',lambda m:st.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),Q)
t0=time.time()
while time.time()-t0<%f: rclpy.spin_once(n,timeout_sec=0.05)
import os,glob
inodes=set()
for fd in os.listdir(f'/proc/{os.getpid()}/fd'):
    try:
        l=os.readlink(f'/proc/{os.getpid()}/fd/{fd}')
        if l.startswith('socket:'): inodes.add(l[8:-1])
    except OSError: pass
drops=0;socks=0
for l in list(open('/proc/net/udp'))[1:]:
    f=l.split()
    if f[9] in inodes: socks+=1;drops+=int(f[-1])
print(json.dumps(dict(received=len(st),udp_sockets=socks,own_socket_drops=drops)))
'''
PUB = '''
import time,rclpy,numpy as np,sys
sys.path.insert(0,'/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
rclpy.init();n=Node('t_pub');p=n.create_publisher(PointCloud2,'/tt/cloud',Q)
a=np.random.default_rng(1).random((24000,6)).astype(np.float32)
time.sleep(3);sent=0;t0=time.time();nxt=t0
while time.time()-t0<%f:
    h=Header();s=time.time();h.stamp.sec=int(s);h.stamp.nanosec=int((s%%1)*1e9);h.frame_id='x'
    p.publish(cloud_io.make_cloud(h,cloud_io.XYZIRT_FIELDS,a));sent+=1;nxt+=0.1;time.sleep(max(0,nxt-time.time()))
print(sent)
'''
def lo():
    for l in open('/proc/net/dev'):
        if l.strip().startswith('lo:'): f = l.split(); return int(f[1]), int(f[2])
def snmp():
    u = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')][1]; return int(u[1]), int(u[5])
ros = 'source /opt/ros/jazzy/setup.bash; '
sub = subprocess.Popen(['bash', '-c', ros + 'python3 -c "%s"' % (SUB % (secs + 6)).replace('"', '\\"')], env=env, stdout=subprocess.PIPE, text=True)
time.sleep(4); l0 = lo(); u0 = snmp()
pub = subprocess.run(['bash', '-c', ros + 'python3 -c "%s"' % (PUB % secs).replace('"', '\\"')], env=env, capture_output=True, text=True)
l1 = lo(); u1 = snmp(); out = sub.communicate(timeout=60)[0]
sent = int(pub.stdout.strip().splitlines()[-1]); sj = json.loads(out.strip().splitlines()[-1]); rec = sj['received']
print(json.dumps(dict(profile=prof.split('/')[-1], sent=sent, received=rec, delivered_pct=100 * rec / sent, loopback_MB_per_s=(l1[0] - l0[0]) / secs / 1e6, loopback_pkts_per_s=(l1[1] - l0[1]) / secs, subscriber_udp_sockets=sj['udp_sockets'], subscriber_own_socket_drops=sj['own_socket_drops'], expected_payload_MB_per_s=10 * 0.576)))
