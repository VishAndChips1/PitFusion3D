#!/usr/bin/env python3
"""Remove Fast DDS shared-memory files in /dev/shm that no running process
has mapped/open. Fast DDS only deletes its segments on a clean exit, so a
crash / kill -9 / OOM leaves one segment per participant behind; with the
64MB segments used here that is ~0.5GB of RAM per unclean shutdown.
Only touches this user's fastrtps files and never a file a live process
uses (so it is safe even if another ROS tool is running).
Usage: clean_stale_shm.py [--dry-run]"""
import glob, os, sys, time
import psutil

dry = '--dry-run' in sys.argv
in_use = set()
for p in psutil.process_iter():
    try:
        for m in p.memory_maps(grouped=False):
            if m.path.startswith('/dev/shm/'):
                in_use.add(m.path)
        for f in p.open_files():
            if f.path.startswith('/dev/shm/'):
                in_use.add(f.path)
    except (psutil.Error, PermissionError):
        continue

uid = os.getuid()
freed = removed = kept = 0
now = time.time()
for path in glob.glob('/dev/shm/fastrtps_*') + glob.glob('/dev/shm/sem.fastrtps_*'):
    try:
        st = os.stat(path)
    except FileNotFoundError:
        continue
    if st.st_uid != uid:
        continue
    if path in in_use or now - st.st_mtime < 3:   # in use, or created a moment ago (race with a starting node)
        kept += 1
        continue
    freed += st.st_blocks * 512
    removed += 1
    if not dry:
        try: os.unlink(path)
        except FileNotFoundError: pass
print(f"{'would remove' if dry else 'removed'} {removed} stale fastrtps shm files "
      f"({freed/1e6:.0f} MB), kept {kept} in use")
