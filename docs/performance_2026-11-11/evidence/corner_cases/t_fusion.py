import os, sys, time, subprocess, threading, signal, psutil, collections, numpy as np
os.environ.update(ROS_DOMAIN_ID='71', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
  FASTRTPS_DEFAULT_PROFILES_FILE='/home/vish/ros2_ws/src/pitfusion3d_bringup/config/fastdds_large_buffers.xml')
SC='/home/vish/ros2_ws/src/landfill_camera_sim/scripts'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
log=open('fusion_node.log','w')
proc=subprocess.Popen([sys.executable,f'{SC}/sensor_fusion_node.py','--ros-args','-p',f"thermal_timeout_s:={os.environ.get('FUSION_TIMEOUT','1.0')}"],stdout=log,stderr=subprocess.STDOUT,env=os.environ)
rclpy.init(); n=Node('t_fusion'); Q=qos_profile_sensor_data
pl=n.create_publisher(PointCloud2,'/landfill/livox/points',Q); pr=n.create_publisher(Image,'/landfill/rgb/image_raw',Q); pt=n.create_publisher(Image,'/landfill/thermal/image_raw',Q)
outs=[]
n.create_subscription(PointCloud2,'/landfill/fusion/points_rgbt',lambda m: outs.append((time.time(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.width*m.height)),Q)
threading.Thread(target=lambda: rclpy.spin(n),daemon=True).start()
rng=np.random.default_rng(2)
def cloud_msg(arr):
    m=PointCloud2(); m.height=1; m.width=arr.shape[0]; m.header.frame_id='camera_platform/lidar_frame'
    m.fields=[PointField(name=f,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,f in enumerate(['x','y','z','intensity'])]
    m.point_step=16; m.row_step=16*arr.shape[0]; m.data=arr.astype(np.float32).tobytes(); m.is_dense=False; return m
def mk(k,nan=False):
    a=np.zeros((k,4),np.float32); a[:,0]=rng.uniform(2,8,k); a[:,1]=rng.uniform(-2,2,k); a[:,2]=rng.uniform(-1,1,k); a[:,3]=1
    if nan: a[rng.random(k)<0.1,0]=np.nan
    return cloud_msg(a)
LID={'ok':mk(24000),'nan':mk(24000,True),'empty':mk(0)}
def img(w,h,enc,bpp):
    m=Image(); m.width=w; m.height=h; m.encoding=enc; m.step=w*bpp; m.header.frame_id='x'; m.data=bytes(w*h*bpp); return m
RGB=img(1280,720,'rgb8',3); TH=img(640,512,'mono16',2)
def stamp(m,t): m.header.stamp.sec=int(t); m.header.stamp.nanosec=int((t%1)*1e9)
def run(secs, lidar='ok', rgb=True, thermal=True, off=0.0, th_off=None, flap=None):
    th_off=off if th_off is None else th_off; t0=time.time(); nxt=t0
    while time.time()-t0<secs:
        t=time.time()
        stamp(LID[lidar],t+off); pl.publish(LID[lidar])
        if rgb: stamp(RGB,t+off); pr.publish(RGB)
        on = thermal and (flap is None or ((t-t0) % (flap[0]+flap[1])) < flap[0])
        if on: stamp(TH,t+th_off); pt.publish(TH)
        nxt+=0.1; time.sleep(max(0,nxt-time.time()))
p=psutil.Process(proc.pid); time.sleep(7)
def phase(name,fn,expect_min):
    if os.environ.get('ONLY') and os.environ['ONLY'] not in name: return
    c0=len(outs); fn(); time.sleep(1.2); got=outs[c0:]; alive=proc.poll() is None
    ok=alive and len(got)>=expect_min
    print(f"[{'PASS' if ok else 'FAIL'}] {name:56s} outputs={len(got):3d} (need>={expect_min}) alive={alive} rss={p.memory_info().rss/1e6:.0f}MB",flush=True)
phase('F1 all three sensors normal, 8s',lambda: run(8),40)
phase('F2a thermal FLAPPING 1s on/1s off, 20s (ideal 200)',lambda: run(20,flap=(1,1)),150)
phase('F2b realistic dropouts: 6s up/3s down, 27s (ideal 270)',lambda: run(27,flap=(6,3)),230)
phase('F3 thermal gone, 8s',lambda: run(8,thermal=False),40)
phase('F4 thermal arrives but UNSYNCABLE (stamps -30s), 10s',lambda: run(10,th_off=-30.0),50)
phase('F5 thermal back and synced, 8s',lambda: run(8),50)
phase('F6a RGB camera missing (lidar+thermal only), 6s',lambda: run(6,rgb=False),0)
phase('F6b RGB returns, 6s',lambda: run(6),40)
phase('F7a LiDAR 10% NaN + empty clouds, 6s',lambda: (run(3,lidar='nan'),run(3,lidar='empty')),0)
phase('F7b normal after bad lidar, 6s',lambda: run(6),40)
phase('F8a clock REWIND 100s (all sensors), 8s',lambda: run(8,off=-100.0),30)
phase('F8b clock jump FORWARD +200s, 8s',lambda: run(8,off=+200.0),30)
phase('F8c back to real time, 8s',lambda: run(8),30)
stamps=[s for _,s,_ in outs]; 
t=time.time(); proc.send_signal(signal.SIGINT)
try: proc.wait(timeout=10); print(f"[PASS] SIGINT shutdown in {time.time()-t:.1f}s")
except subprocess.TimeoutExpired: proc.kill(); print("[FAIL] no exit within 10s")
txt=open('fusion_node.log').read()
print('duplicate output stamps (within one clock regime):',len(stamps)-len(set(stamps)),'| Tracebacks:',txt.count('Traceback'),'| "failed, skipping":',txt.count('failed, skipping'))
print('mode switches logged: degraded x%d, recovered x%d'%(txt.count('fusing lidar + RGB only'),txt.count('working again')))
