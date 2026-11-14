"""Extract RGB frames and LiDAR scans from the 5 checkerboard bags into npz/png files."""
import os, sys
import numpy as np, cv2
sys.path.insert(0, os.path.expanduser('~/SensorCalibration/calibration/common'))
from bag_reader import read_bag, image_msg_to_numpy, pointcloud2_to_xyz

BAGS = os.path.expanduser('~/SensorCalibration/updated_bags')
OUT = os.path.expanduser('~/checkerboard_analysis/data')
os.makedirs(OUT, exist_ok=True)
for name in ['close', 'far', 'left', 'right', 'centre']:
    imgs, itimes, scans, stimes = [], [], [], []
    for topic, msg, t in read_bag(f'{BAGS}/{name}', ['/landfill/rgb/image_raw', '/landfill/livox/points']):
        if topic.endswith('image_raw'):
            imgs.append(image_msg_to_numpy(msg).copy()); itimes.append(t)
        else:
            xyz = pointcloud2_to_xyz(msg)
            scans.append(np.asarray(xyz, np.float32)); stimes.append(t)
    os.makedirs(f'{OUT}/{name}', exist_ok=True)
    for i in (0, len(imgs) // 2, len(imgs) - 1):
        cv2.imwrite(f'{OUT}/{name}/frame_{i:02d}.png', imgs[i])
    np.savez_compressed(f'{OUT}/{name}.npz', imgs=np.stack(imgs), itimes=itimes,
                        scans=np.array(scans, dtype=object), stimes=stimes, allow_pickle=True)
    print(name, len(imgs), 'imgs', len(scans), 'scans', sum(len(s) for s in scans), 'pts')
