import os,sys,importlib.util,numpy as np
os.environ.update(ROS_DOMAIN_ID='73',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
import rclpy
SRC='/home/vish/ros2_ws/src/pitfusion3d_bringup/pitfusion3d_bringup/livox_restamp_node.py'
OLD='/home/vish/thesis_tests/audit_20261112/livox_restamp_node.py.pre_clockstep'
def load(path,name):
    from importlib.machinery import SourceFileLoader
    return SourceFileLoader(name,path).load_module()
rclpy.init()
def make(mod):
    n=mod.LivoxRestampNode()
    if not hasattr(n,'_estimate'):   # old code: emulate its single-sample logic by calling its callback body
        class M: pass
        def est(arrival,dev,n=n):
            from sensor_msgs.msg import PointCloud2
            m=PointCloud2();m.header.stamp.sec=dev//10**9;m.header.stamp.nanosec=dev%10**9
            n.get_clock=lambda:type('C',(),{'now':lambda s:type('T',(),{'nanoseconds':arrival})()})()
            n._publisher=type('P',(),{'publish':lambda s,x:None})()
            n._on_cloud(m);return m.header.stamp.sec*10**9+m.header.stamp.nanosec
        n._estimate=est
    return n
def stream(n,dev0,host0,secs,rng,hz=10,jit=8e6,step=0):
    """returns max |corrected - arrival| error in ms after each sample"""
    errs=[];d=dev0;h=host0
    for i in range(int(secs*hz)):
        d+=int(1e9/hz);h+=int(1e9/hz)+step if i==0 else int(1e9/hz)
        arr=h+int(abs(rng.normal(0,jit)))
        errs.append((n._estimate(arr,d)-arr)/1e6)
    return errs,d,h
def case(name,mod):
    rng=np.random.default_rng(1);n=make(mod)
    dev=3_000_000_000_000;host=1_794_000_000_000_000_000
    e,dev,host=stream(n,dev,host,30,rng)                       # warm up 30 s
    out={}
    out['steady_err_ms']=float(np.percentile(np.abs(e[-100:]),95))
    for label,step_s in (('fwd+300s',300),('back-100s',-100),('fwd+0.2s(below thr)',0.2),('fwd+3s',3)):
        e,dev,host=stream(n,dev,host,25,rng,step=int(step_s*1e9))
        e=np.array(e);lag=np.abs(e)>50                     # >50 ms stamp error
        out[label]=f'wrong for {lag.sum()/10:.1f}s (first-sample err {e[0]:+.0f} ms)'
    # lidar restart: device counter goes back
    e,dev,host=stream(n,3_000_000_000,host,10,rng); out['lidar restart']=f'wrong for {(np.abs(e)>50).sum()/10:.1f}s'
    # 5 s stall, no clock change
    pre=getattr(n,'clock_resets',0);dev+=5*10**9;host+=5*10**9
    e,dev,host=stream(n,dev,host,5,rng); out['5s stall']=f'wrong for {(np.abs(e)>50).sum()/10:.1f}s, resets={getattr(n,"clock_resets",0)-pre}'
    # single 0.8 s latency spike
    d=dev+int(1e8);h=host+int(1e8);err=(n._estimate(h+800_000_000,d)-(h+800_000_000))/1e6
    d+=int(1e8);h+=int(1e8);err2=(n._estimate(h+int(5e6),d)-(h+int(5e6)))/1e6
    out['0.8s latency spike']=f'spike err {err:+.0f} ms, next sample err {err2:+.0f} ms'
    print(f'--- {name}'); [print(f'   {k:22s} {v}') for k,v in out.items()]
    n.destroy_node()
case('OLD (min over 20 s window)',load(OLD,'old'))
case('NEW (with clock-step reset)',load(SRC,'new'))
rclpy.shutdown()
