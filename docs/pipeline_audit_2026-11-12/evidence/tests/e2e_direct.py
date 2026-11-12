import os,sys,time,threading,struct,gzip,urllib.request,json,numpy as np
sys.path.insert(0,'/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import rclpy,cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2,Image
from std_msgs.msg import Header
DUR=float(sys.argv[1]) if len(sys.argv)>1 else 60
rclpy.init();n=Node('e2e_fake_lidar');pub=n.create_publisher(PointCloud2,'/landfill/livox/points',Q)
cnt={'rgbt':0,'merged':0,'thermal':0};cov=[]
def on_rgbt(m):
    cnt['rgbt']+=1
    if cnt['rgbt']%25==0:
        a=cloud_io.read_fields(m,['rgb','thermal']);rgb=(a[:,0].view(np.uint32)!=0);th=np.isfinite(a[:,1])
        cov.append((len(a),float(rgb.mean()*100),float(th.mean()*100)))
n.create_subscription(PointCloud2,'/landfill/fusion/points_rgbt',on_rgbt,Q)
n.create_subscription(PointCloud2,'/landfill/fusion/points_merged',lambda m:cnt.__setitem__('merged',cnt['merged']+1),Q)
n.create_subscription(Image,'/landfill/thermal/image_raw',lambda m:cnt.__setitem__('thermal',cnt['thermal']+1),Q)
threading.Thread(target=lambda:rclpy.spin(n),daemon=True).start()
rng=np.random.default_rng(5);k=24000
def scan():
    a=np.zeros((k,6),np.float32);a[:,0]=rng.uniform(1.5,8,k);a[:,1]=rng.uniform(-3,3,k);a[:,2]=rng.uniform(-1.5,1.5,k);a[:,3]=100;a[:,4]=1;a[:,5]=0;return a
dev0=3_000_000_000_000;t0=time.time();i=0;nxt=t0
while time.time()-t0<DUR:
    d=int(time.time()*1e9);h=Header();h.stamp.sec=d//10**9;h.stamp.nanosec=d%10**9;h.frame_id='camera_platform/lidar_frame'
    pub.publish(cloud_io.make_cloud(h,cloud_io.XYZIRT_FIELDS,scan()));i+=1;nxt+=0.1;time.sleep(max(0,nxt-time.time()))
time.sleep(2);el=time.time()-t0
def get(p):
    try:b=gzip.decompress(urllib.request.urlopen('http://localhost:8080/'+p,timeout=5).read());return struct.unpack_from('<II',b,0)
    except Exception as e:return ('err',str(e)[:40])
print(f'fake lidar sent {i} scans in {el:.0f}s ({i/el:.1f} Hz)')
print(f'fused clouds: {cnt["rgbt"]} ({cnt["rgbt"]/el:.1f} Hz)   merged clouds: {cnt["merged"]} ({cnt["merged"]/el:.2f} Hz)   thermal frames: {cnt["thermal"]/el:.1f} Hz')
if cov:
    c=np.array(cov);print(f'points per fused cloud ~{c[:,0].mean():.0f}; with valid RGB {c[:,1].mean():.0f}%; with valid thermal {c[:,2].mean():.0f}%')
print('viewer merged.bin (n,seq):',get('merged.bin'),' live.bin (n,seq):',get('live.bin'))
