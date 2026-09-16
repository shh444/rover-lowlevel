#!/usr/bin/env python3
"""run.py 기록(trace.csv) 요약: 단계별 시간·추종 오차·토크·몸체 높이·중력 z.

    python tools/trace_stats.py runs/standup-01 [--joint FL_calf] [--every 0.5]
"""
import argparse
import csv
from collections import OrderedDict
from pathlib import Path

import numpy as np

JOINTS = [f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("abad", "thigh", "calf")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path, help="run.py 의 --out 폴더")
    ap.add_argument("--joint", default="FL_calf", choices=JOINTS)
    ap.add_argument("--every", type=float, default=0.5, help="표본 출력 간격 s")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.run / "trace.csv", encoding="utf-8")))
    t = np.array([float(r["t"]) for r in rows])
    phase = np.array([r["phase"] for r in rows])
    q = np.array([[float(r[f"q_{j}"]) for j in JOINTS] for r in rows])
    qd = np.array([[float(r[f"qdes_{j}"]) for j in JOINTS] for r in rows])
    tau = np.array([[float(r[f"tau_{j}"]) for j in JOINTS] for r in rows])
    gz = np.array([float(r["grav_z"]) for r in rows])
    bz = np.array([float(r["base_z"]) for r in rows])
    kp = np.array([float(r["kp"]) for r in rows])
    err = np.abs(qd - q)
    err[kp == 0] = 0.0          # 댐핑 구간은 목표각이 힘을 내지 않으므로 오차로 세지 않는다

    print("%-12s%7s%7s%10s%9s%9s%15s%11s" % ("phase", "t0", "t1", "err_mean", "err_max",
                                             "tau_max", "base_z", "grav_z_min"))
    for ph in OrderedDict.fromkeys(phase):
        m = phase == ph
        print(f"{ph:<12}{t[m].min():7.2f}{t[m].max():7.2f}{err[m].mean():10.3f}{err[m].max():9.3f}"
              f"{np.abs(tau[m]).max():9.1f}   {bz[m].min():.3f}~{bz[m].max():.3f}{gz[m].min():11.3f}")
    j = JOINTS.index(a.joint)
    print(f"\n{a.joint}:   t      phase        q      q_des    tau")
    step = max(1, int(round(a.every / max(t[1] - t[0], 1e-9))))
    for i in range(0, len(rows), step):
        print(f"{t[i]:7.2f}  {phase[i]:<12} {q[i, j]:+.3f}  {qd[i, j]:+.3f}  {tau[i, j]:+7.2f}")


if __name__ == "__main__":
    main()
