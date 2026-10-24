#!/usr/bin/env python3
"""Continuously tail the kernel log (dmesg-equivalent) to a persistent file.

Runs `journalctl -k -f` (follow mode) as a subprocess and copies every
line out as it arrives, fsync'ing after each one. This is the most
likely place to see USB disconnect/reconnect events, undervoltage
warnings, or anything else the kernel notices right up to the moment of
a crash -- things the PMIC polling loop in power_logger.py won't
necessarily catch between its 0.2s samples.

No sudo needed: this account is in the 'adm' group, which journalctl
already respects for reading the kernel log.
"""
import os
import subprocess
import sys
import time

LOG_PATH = '/home/vish/power_logs/kernel_log.txt'


def main():
    with open(LOG_PATH, 'a') as out:
        out.write(f'\n--- kernel_log_tail started {time.strftime("%Y-%m-%d %H:%M:%S")} ---\n')
        out.flush()
        os.fsync(out.fileno())

        while True:
            try:
                proc = subprocess.Popen(
                    ['journalctl', '-k', '-f', '-n', '0', '-o', 'short-iso', '--no-pager'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
                )
                for line in proc.stdout:
                    out.write(line)
                    out.flush()
                    os.fsync(out.fileno())
            except Exception as exc:
                print(f'error: {exc}', file=sys.stderr)
            # journalctl -f shouldn't normally exit; if it does (journald
            # restart, etc.) wait briefly and reattach rather than busy-loop.
            time.sleep(1.0)


if __name__ == '__main__':
    main()
