import os, sys, time, subprocess, threading, gzip, struct, socket, http.client, psutil, numpy as np
os.environ.update(ROS_DOMAIN_ID='72', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
  FASTRTPS_DEFAULT_PROFILES_FILE='/home/vish/ros2_ws/src/pitfusion3d_bringup/config/fastdds_large_buffers.xml')
SC='/home/vish/ros2_ws/src/landfill_camera_sim/scripts'; PORT=18080
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, PointField
log=open('streamer_node.log','w')
proc=subprocess.Popen([sys.executable,f'{SC}/fusion_web_streamer.py','--ros-args','-p',f'http_port:={PORT}'],stdout=log,stderr=subprocess.STDOUT,env=os.environ,cwd=SC)
rclpy.init(); n=Node('t_stream'); pm=n.create_publisher(PointCloud2,'/landfill/fusion/points_merged',Q); pl=n.create_publisher(PointCloud2,'/landfill/fusion/points_rgbt',Q)
threading.Thread(target=lambda: rclpy.spin(n),daemon=True).start()
rng=np.random.default_rng(3)
def cloud(k, mode='ok'):
    a=np.zeros((k,6),np.float32)
    if mode=='same': a[:]=[1,1,1,1,0,1]
    else:
        a[:,0]=rng.uniform(0,10,k); a[:,1]=rng.uniform(-3,3,k); a[:,2]=rng.uniform(-1,2,k); a[:,3]=1
        a[:,4]=np.frombuffer(rng.integers(0,2**24,k).astype(np.uint32).tobytes(),np.float32); a[:,5]=rng.uniform(250,320,k)
    if mode=='nan_thermal': a[:,5]=np.nan
    m=PointCloud2(); m.height=1; m.width=k; m.header.frame_id='f'
    m.fields=[PointField(name=f,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,f in enumerate(['x','y','z','intensity','rgb','thermal'])]
    m.point_step=24; m.row_step=24*k; m.data=a.tobytes(); return m
def get(path, etag=None, timeout=5):
    c=http.client.HTTPConnection('127.0.0.1',PORT,timeout=timeout); h={'If-None-Match':etag} if etag else {}
    c.request('GET',path,headers=h); r=c.getresponse(); body=r.read(); return r.status, r.getheader('ETag'), body
def npts(body): b=gzip.decompress(body); return struct.unpack_from('<I',b,0)[0], struct.unpack_from('<I',b,4)[0]
res=[]
def check(name, ok, detail=''):
    res.append(ok); print(f"[{'PASS' if ok else 'FAIL'}] {name:62s} {detail}",flush=True)
for _ in range(60):
    try: socket.create_connection(('127.0.0.1',PORT),timeout=0.5).close(); break
    except OSError: time.sleep(0.5)
p=psutil.Process(proc.pid)
s,e,b=get('/merged.bin'); check('S1 request BEFORE any data -> valid empty payload', s==200 and npts(b)[0]==0, f'status={s} n={npts(b)[0]}')
s,_,b=get('/'); check('S2 viewer page loads', s==200 and b'PitFusion3D' in b, f'{len(b)} bytes')
s,_,_=get('/nonexistent'); check('S3 unknown path -> 404', s==404, f'status={s}')
for _ in range(15): pm.publish(cloud(100000)); pl.publish(cloud(24000)); time.sleep(0.1)
s,e1,b=get('/merged.bin'); n1,q1=npts(b); check('S4 after data: merged capped to 20000 points', s==200 and 0<n1<=20000, f'n={n1} seq={q1}')
s,e2,b=get('/live.bin'); n2,_=npts(b); check('S5 live capped to 5000 points', s==200 and 0<n2<=5000, f'n={n2}')
s,_,b=get('/merged.bin',etag=e1); check('S6 same ETag -> 304 (no re-send)', s==304 and len(b)==0, f'status={s}')
time.sleep(30)   # nobody watching for 30s while data keeps changing
for _ in range(5): pm.publish(cloud(100000)); time.sleep(0.1)
s,e3,b=get('/merged.bin',etag=e1); n3,q3=npts(b) if s==200 else (0,0); check('S7 after 30s idle: first request returns NEWEST data (lazy encode)', s==200 and q3>q1, f'status={s} seq {q1}->{q3}')
errs=[]; cnt=[0]; stop=time.time()+10
def hammer():
    c=http.client.HTTPConnection('127.0.0.1',PORT,timeout=5); et=None
    while time.time()<stop:
        try:
            c.request('GET',random_path(),headers={'If-None-Match':et} if et and random_path.last=='/merged.bin' else {}); r=c.getresponse(); r.read(); cnt[0]+=1
            if r.status not in (200,304): errs.append(r.status)
            et=r.getheader('ETag') or et
        except Exception as ex: errs.append(repr(ex)); c=http.client.HTTPConnection('127.0.0.1',PORT,timeout=5)
class RP:
    last=''
    def __call__(self): RP.last=['/merged.bin','/live.bin'][int(time.time()*1000)%2]; self.last=RP.last; return RP.last
random_path=RP()
pub_stop=time.time()+10
def feeder():
    while time.time()<pub_stop: pm.publish(cloud(100000)); pl.publish(cloud(24000)); time.sleep(0.1)
th=[threading.Thread(target=hammer) for _ in range(16)]+[threading.Thread(target=feeder)]
cpu0=p.cpu_times(); [t.start() for t in th]; [t.join() for t in th]; cpu1=p.cpu_times()
check('S8 16 concurrent clients hammering 10s', not errs and cnt[0]>500, f'{cnt[0]} requests, errors={len(errs)}, streamer cpu={(sum(cpu1[:2])-sum(cpu0[:2]))/10*100:.0f}% of a core, rss={p.memory_info().rss/1e6:.0f}MB')
bad=0
for i in range(60):
    try:
        so=socket.create_connection(('127.0.0.1',PORT),timeout=2); so.sendall(b'GET /merged.bin HTTP/1.1\r\nHost: x\r\n\r\n'); so.recv(10); so.close()
    except Exception: bad+=1
s,_,_=get('/live.bin'); check('S9 60 clients that disconnect mid-response', s==200 and proc.poll() is None, f'server alive, later request status={s}')
for payload in (b'\x00\xff\xfe garbage\r\n\r\n', b'GET / HTTP/1.1\r\n'+b'X: '+b'a'*70000+b'\r\n\r\n', b'POST /merged.bin HTTP/1.1\r\nContent-Length: 5\r\n\r\nhello'):
    try:
        so=socket.create_connection(('127.0.0.1',PORT),timeout=2); so.sendall(payload); so.settimeout(2)
        try: so.recv(100)
        except Exception: pass
        so.close()
    except Exception: pass
s,_,_=get('/merged.bin'); check('S10 garbage / oversized-header / POST requests', s in (200,304) and proc.poll() is None, f'alive, next request status={s}')
pm.publish(cloud(5000,'same')); time.sleep(0.3); s,_,b=get('/merged.bin'); check('S11 zero-extent cloud (all points identical)', s==200 and proc.poll() is None, f'status={s} n={npts(b)[0]}')
pm.publish(cloud(5000,'nan_thermal')); time.sleep(0.3); s,_,b=get('/merged.bin'); check('S12 all-NaN thermal field', s==200 and proc.poll() is None, f'status={s} n={npts(b)[0]}')
pm.publish(cloud(0)); time.sleep(0.3); s,_,b=get('/merged.bin'); check('S13 empty cloud', s==200 and proc.poll() is None, f'status={s} n={npts(b)[0]}')
for _ in range(5): pm.publish(cloud(100000)); time.sleep(0.1)
s,_,b=get('/merged.bin'); check('S14 recovers to normal data after degenerate clouds', s==200 and npts(b)[0]>1000, f'n={npts(b)[0]}')
import signal; t=time.time(); proc.send_signal(signal.SIGINT)
try: proc.wait(timeout=10); check('S15 SIGINT shutdown', True, f'{time.time()-t:.1f}s')
except subprocess.TimeoutExpired: proc.kill(); check('S15 SIGINT shutdown', False, 'hung >10s')
txt=open('streamer_node.log').read(); print('node log: Tracebacks =',txt.count('Traceback'),'| "failed" errors =',txt.count('failed'))
print(f"RESULT: {sum(res)}/{len(res)} passed")
