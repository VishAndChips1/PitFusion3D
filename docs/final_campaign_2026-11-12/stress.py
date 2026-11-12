#!/usr/bin/env python3
"""CPU/memory stress workers. Usage: stress.py <n_workers> <seconds> <abort_file> [mem_mb]. Workers exit early if abort_file exists."""
import sys, time, os, multiprocessing as mp, numpy as np
n, secs, abort = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]; mem = int(sys.argv[4]) if len(sys.argv) > 4 else 0
def worker(i):
    end = time.time() + secs; a = np.random.rand(300, 300); buf = np.zeros(mem * 1024 * 1024 // 8) if (mem and i == 0) else None; x = 1.0000001; k = 0
    while time.time() < end:
        if k % 20 == 0 and os.path.exists(abort): return
        a = a @ a; a /= np.abs(a).max() + 1e-9; x = x * 1.0000001 % 7.0; k += 1
        if buf is not None: buf[(k * 4099) % len(buf)::4099] += 1.0
if __name__ == '__main__':
    ps = [mp.Process(target=worker, args=(i,)) for i in range(n)]; [p.start() for p in ps]; [p.join() for p in ps]; print('done')
