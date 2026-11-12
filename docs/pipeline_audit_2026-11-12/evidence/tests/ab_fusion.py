import os,sys,time,subprocess,threading,numpy as np
SC='/home/vish/ros2_ws/src/landfill_camera_sim/scripts';sys.path.insert(0,SC)
import rclpy,cloud_io
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import json
VAR={k:tuple(v) for k,v in json.loads(sys.argv[2]).items()}
procs=[];topics={}
for i,(name,(q,s)) in enumerate(VAR.items()):
    t=f'/ab/p{i}';topics[name]=t
    procs.append(subprocess.Popen(['nice','-n','5',sys.executable,f'{SC}/sensor_fusion_node.py','--ros-args','-r',f'__node:=ab_fusion{i}','-p',f'output_topic:={t}','-p',f'queue_size:={q}','-p',f'sync_slop:={s}'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
time.sleep(8)
rclpy.init();n=rclpy.create_node('ab_probe');cnt={k:0 for k in topics};pub=n.create_publisher(PointCloud2,'/landfill/livox/points',Q)
for k,t in topics.items(): n.create_subscription(PointCloud2,t,lambda m,k=k:cnt.__setitem__(k,cnt[k]+1),Q)
LIVE='LIVE node (q5 slop.15)';cnt[LIVE]=0;n.create_subscription(PointCloud2,'/landfill/fusion/points_rgbt',lambda m:cnt.__setitem__(LIVE,cnt[LIVE]+1),Q)
threading.Thread(target=lambda:rclpy.spin(n),daemon=True).start()
rng=np.random.default_rng(5);k=24000
a=np.zeros((k,6),np.float32);a[:,0]=rng.uniform(1.5,8,k);a[:,1]=rng.uniform(-3,3,k);a[:,2]=rng.uniform(-1.5,1.5,k);a[:,3]=100;a[:,4]=1
DUR=float(sys.argv[1]);t0=time.time();nxt=t0;sent=0
while time.time()-t0<DUR:
    d=int(time.time()*1e9);h=Header();h.stamp.sec=d//10**9;h.stamp.nanosec=d%10**9;h.frame_id='camera_platform/lidar_frame'
    pub.publish(cloud_io.make_cloud(h,cloud_io.XYZIRT_FIELDS,a));sent+=1;nxt+=0.1;time.sleep(max(0,nxt-time.time()))
time.sleep(1.5);el=time.time()-t0
print(f'input scans: {sent/el:.1f} Hz over {el:.0f}s')
for k in list(VAR)+[LIVE]: print(f'  {k:22s} fused {cnt[k]/el:.2f} Hz  ({100*cnt[k]/sent:.0f}% of scans)')
for p in procs: p.terminate()
for p in procs:
    try:p.wait(5)
    except Exception:p.kill()
