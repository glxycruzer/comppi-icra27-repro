#!/usr/bin/env python3
"""Per-cycle CPU cost of a running controller_server, measured externally.

Samples /proc/<pid>/task/*/stat (utime+stime, all threads) while counting
/cmd_vel_nav messages (= control cycles). Reports CPU-ms per cycle for the
busiest thread (the control loop) and for the whole process (control loop +
costmap + executor). Works for any controller plugin, so Nav2-MPPI and CoMPPI
are measured by the same method on the same board.

  python3 cpu_cycle_probe.py <pid> <seconds> <out.csv>
"""
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CLK = os.sysconf("SC_CLK_TCK")


def thread_cpu(pid):
    """dict tid -> cpu seconds (utime+stime) for every thread of pid."""
    out = {}
    for tid in os.listdir(f"/proc/{pid}/task"):
        try:
            with open(f"/proc/{pid}/task/{tid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
            out[tid] = (int(fields[11]) + int(fields[12])) / CLK
        except (FileNotFoundError, IndexError, ValueError):
            pass
    return out


class Probe(Node):
    def __init__(self):
        super().__init__("cpu_cycle_probe")
        self.n = 0
        # standalone harness publishes /cmd_vel; the robot bring-up remaps the
        # controller output to /cmd_vel_nav -- count whichever is present
        self.create_subscription(Twist, "cmd_vel", self.cb, 50)
        self.create_subscription(Twist, "cmd_vel_nav", self.cb, 50)

    def cb(self, _):
        self.n += 1


def main():
    pid, secs, out = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    rclpy.init()
    node = Probe()
    t0 = time.monotonic()
    c0 = thread_cpu(pid)
    n0 = node.n
    rows = []
    last_t, last_c, last_n = t0, c0, n0
    while time.monotonic() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_t >= 1.0:
            c = thread_cpu(pid)
            dn = node.n - last_n
            dcpu = sum(c.values()) - sum(last_c.values())
            dbusy = max((c.get(t, 0) - last_c.get(t, 0)) for t in c) if c else 0
            rows.append((round(now - t0, 1), dn, round(1000 * dcpu, 1), round(1000 * dbusy, 1)))
            last_t, last_c, last_n = now, c, node.n
    c1 = thread_cpu(pid)
    cycles = node.n - n0
    total_ms = 1000 * (sum(c1.values()) - sum(c0.values()))
    busiest_ms = 1000 * max((c1.get(t, 0) - c0.get(t, 0)) for t in c1)
    with open(out, "w") as f:
        f.write("t_s,cycles_in_window,cpu_ms_all_threads,cpu_ms_busiest_thread\n")
        for r in rows:
            f.write(",".join(map(str, r)) + "\n")
    print(f"cycles={cycles}  cpu_all={total_ms:.0f} ms  cpu_busiest_thread={busiest_ms:.0f} ms")
    if cycles:
        print(f"per cycle: busiest thread {busiest_ms / cycles:.2f} ms, "
              f"all threads {total_ms / cycles:.2f} ms  (threads: {len(c1)})")
    # steady-state (exclude first/last 3 s): median per-second rate
    mid = [r for r in rows if 3 < r[0] < secs - 3 and r[1] > 0]
    if mid:
        per = sorted(r[3] / r[1] for r in mid)
        print(f"steady-state busiest-thread ms/cycle: median {per[len(per)//2]:.2f}, "
              f"p90 {per[int(0.9*len(per))-1]:.2f}, max {per[-1]:.2f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
