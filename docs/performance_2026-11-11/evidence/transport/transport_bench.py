import sys, time, threading, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
role, dur = sys.argv[1], float(sys.argv[2])
rclpy.init(); n = Node('tb_' + role)
SPECS = [('/tb/rgb', Image, 15, (1280, 720, 'rgb8', 3)), ('/tb/thermal', Image, 30, (640, 512, 'mono16', 2)),
         ('/tb/cloud', PointCloud2, 10, None)]
if role == 'pub':
    pubs = [(n.create_publisher(t, tp, qos_profile_sensor_data), t, hz, sp) for tp, t, hz, sp in SPECS]
    def mk(t, sp):
        m = t()
        if t is Image:
            m.width, m.height, m.encoding, bpp = sp[0], sp[1], sp[2], sp[3]; m.step = m.width * bpp
            m.data = bytes(m.width * m.height * bpp)
        else:
            m.height = 1; m.width = 24000; m.point_step = 24; m.row_step = 24 * 24000; m.data = bytes(24 * 24000)
        return m
    msgs = [mk(t, sp) for _, t, _, sp in pubs]
    def loop(i):
        p, t, hz, sp = pubs[i]; per = 1.0 / hz; nxt = time.time(); end = time.time() + dur
        while time.time() < end:
            msgs[i].header.stamp = n.get_clock().now().to_msg(); p.publish(msgs[i])
            nxt += per; time.sleep(max(0, nxt - time.time()))
    ths = [threading.Thread(target=loop, args=(i,)) for i in range(3)]
    [t.start() for t in ths]; [t.join() for t in ths]
else:
    cnt = collections.Counter(); first = {}
    for tp, t, hz, sp in SPECS:
        n.create_subscription(t, tp, (lambda m, tp=tp: (cnt.update([tp]), first.setdefault(tp, time.time()))), qos_profile_sensor_data)
    th = threading.Thread(target=lambda: rclpy.spin(n), daemon=True); th.start()
    time.sleep(dur)
    for tp, t, hz, sp in SPECS:
        print('  %-12s sent %2d Hz -> received %5.2f Hz' % (tp, hz, cnt[tp] / (dur - 3)))
