import os, sys, time, subprocess, threading, signal, psutil, numpy as np
os.environ.update(ROS_DOMAIN_ID='74', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
  FASTRTPS_DEFAULT_PROFILES_FILE='/home/vish/ros2_ws/src/pitfusion3d_bringup/config/fastdds_large_buffers.xml')
SC='/home/vish/ros2_ws/src/landfill_camera_sim/scripts'; sys.path.insert(0, SC)
import rclpy, cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
log = open('merge_stall_node.log','w')
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
def send_const(k,hz,secs,stamp):
    nxt=time.time();t_end=nxt+secs;sent=0
    while time.time()<t_end:
        h=Header(); h.stamp.sec=int(stamp); h.stamp.nanosec=0; h.frame_id='camera_platform/lidar_frame'
        pub.publish(cloud_io.make_cloud(h, cloud_io.XYZIRT_FIELDS, KINDS['normal'])); sent+=1
        nxt+=1/hz; time.sleep(max(0,nxt-time.time()))
    return sent
r0=p.memory_info().rss/1e6;c0=len(outs)
sent=send_const('normal',10,25,1_800_000_000)   # 250 scans, stamp frozen: window can never close by time
time.sleep(3);got=len(outs)-c0;r1=p.memory_info().rss/1e6
print(f"[{'PASS' if got>=3 and r1-r0<150 and proc.poll() is None else 'FAIL'}] STALLED STAMPS: {sent} scans with a frozen timestamp -> {got} merged outputs, rss {r0:.0f}->{r1:.0f} MB, alive={proc.poll() is None}")
proc.send_signal(signal.SIGINT); proc.wait(10)
