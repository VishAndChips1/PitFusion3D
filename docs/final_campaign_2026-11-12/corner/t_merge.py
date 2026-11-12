import os, sys, time, subprocess, threading, signal, psutil, numpy as np
os.environ.update(ROS_DOMAIN_ID='70', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
  FASTRTPS_DEFAULT_PROFILES_FILE='/home/vish/ros2_ws/src/pitfusion3d_bringup/config/fastdds_large_buffers.xml')
SC='/home/vish/ros2_ws/src/landfill_camera_sim/scripts'; sys.path.insert(0, SC)
import rclpy, cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
log = open('merge_node.log','w')
proc = subprocess.Popen([sys.executable, f'{SC}/registration_merge_node.py','--ros-args','-p','window_duration:=0.5'], stdout=log, stderr=subprocess.STDOUT, env=os.environ)
rclpy.init(); n = Node('t_merge')
pub = n.create_publisher(PointCloud2,'/landfill/fusion/points_rgbt',qos_profile_sensor_data)
outs=[]; 
n.create_subscription(PointCloud2,'/landfill/fusion/points_merged',lambda m: outs.append((time.time(), m.header.stamp.sec+m.header.stamp.nanosec*1e-9, m.width*m.height)),qos_profile_sensor_data)
threading.Thread(target=lambda: rclpy.spin(n),daemon=True).start()
rng=np.random.default_rng(1)
def base(k=24000):
    a=np.zeros((k,6),np.float32); a[:,0]=rng.uniform(2,8,k); a[:,1]=rng.uniform(-2,2,k); a[:,2]=rng.uniform(-1,1,k); a[:,3]=1; a[:,4]=1.0; a[:,5]=300.0; return a
KINDS={'normal':base(),'nan':np.where(rng.random((24000,6))<0.1,np.nan,base()).astype(np.float32),
       'empty':np.zeros((0,6),np.float32),'single':base(1),'identical':np.tile(np.array([[3,0,0,1,1,300]],np.float32),(24000,1))}
def send(kind, hz, secs, stamp_offset=0.0):
    t_end=time.time()+secs; nxt=time.time(); sent=0
    while time.time()<t_end:
        h=Header(); st=time.time()+stamp_offset; h.stamp.sec=int(st); h.stamp.nanosec=int((st%1)*1e9); h.frame_id='camera_platform/lidar_frame'
        pub.publish(cloud_io.make_cloud(h, cloud_io.XYZIRT_FIELDS, KINDS[kind])); sent+=1
        nxt+=1/hz; time.sleep(max(0,nxt-time.time()))
    return sent
p=psutil.Process(proc.pid); time.sleep(6)
def phase(name, fn, expect_min=None, max_lat=None, rel_stamp=True):
    c0=len(outs); t0=time.time(); fn(); time.sleep(1.5); got=outs[c0:]
    lat=[w-s for w,s,_ in got] if rel_stamp else []
    alive = proc.poll() is None
    ok = alive and (expect_min is None or len(got)>=expect_min) and (max_lat is None or not lat or max(lat)<=max_lat)
    print(f"[{'PASS' if ok else 'FAIL'}] {name:46s} outputs={len(got):3d} {('max latency %.2fs'%max(lat)) if lat else ''} alive={alive} rss={p.memory_info().rss/1e6:.0f}MB" ,flush=True)
    return ok
phase('P1 normal 24k pts @10Hz, 10s', lambda: send('normal',10,10), 8, 1.5)
phase('P2 10% NaN points, 5s', lambda: send('nan',10,5), None)
phase('P3a empty clouds, 3s', lambda: send('empty',10,3), None)
phase('P3b single-point clouds, 3s', lambda: send('single',10,3), None)
phase('P3c 24k identical points (degenerate), 3s', lambda: send('identical',10,3), None)
phase('P4 recovery after bad data (worker alive?)', lambda: send('normal',10,8), 6, 1.5)
phase('P5 stamps jump BACK 100s, 8s', lambda: send('normal',10,8,-100.0), 4, rel_stamp=False)
phase('P6 stamps jump FORWARD +200s, 8s', lambda: send('normal',10,8,+200.0), 4, rel_stamp=False)
phase('P7 stamps back to real time (clock fixed), 8s', lambda: send('normal',10,8), 5, 2.0)
phase('P8 OVERLOAD 40Hz (4x) for 12s', lambda: send('normal',40,12), 6, 3.0)
phase('P9 right after overload: latency recovered?', lambda: send('normal',10,8), 4, 1.5)
phase('P10 silence 6s then resume 8s', lambda: (time.sleep(6), send('normal',10,8)), 4, 1.5)
t=time.time(); proc.send_signal(signal.SIGINT)
try: proc.wait(timeout=10); print(f"[PASS] SIGINT shutdown in {time.time()-t:.1f}s")
except subprocess.TimeoutExpired: proc.kill(); print("[FAIL] did not exit within 10s of SIGINT")
txt=open('merge_node.log').read()
print('node log: "failed, skipping" count =',txt.count('failed, skipping'),'| "replaced a stale window" =',txt.count('replaced a stale'),'| Tracebacks =',txt.count('Traceback'))
