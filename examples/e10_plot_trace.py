#!/usr/bin/env python3
"""예제 10: 실행 기록(trace.csv) 그래프 — 관절각(q, q_des)·토크·몸체 높이·중력 z 를 PNG 로 저장.

matplotlib 이 필요하다 (pip install matplotlib). 실기 기록도 같은 형식이라 그대로 쓸 수 있다.

    python examples/e10_plot_trace.py runs/real-sine-01 --joints FL_thigh,FR_thigh --out runs/real-sine-01/plot.png
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import JOINT_SHORT   # noqa: E402


def load(run_dir: Path):
    rows = list(csv.DictReader(open(run_dir / "trace.csv", encoding="utf-8")))
    col = lambda name: np.array([float(r[name]) for r in rows])   # noqa: E731
    data = {"t": col("t"), "phase": [r["phase"] for r in rows], "grav_z": col("grav_z"), "base_z": col("base_z")}
    for j in JOINT_SHORT:
        data[f"q_{j}"], data[f"qdes_{j}"], data[f"tau_{j}"] = col(f"q_{j}"), col(f"qdes_{j}"), col(f"tau_{j}")
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path, help="run.py/예제의 기록 폴더 (trace.csv 가 있는 곳)")
    ap.add_argument("--joints", default="FL_thigh,FL_calf", help="쉼표로 구분한 관절 이름")
    ap.add_argument("--out", type=Path, default=None, help="PNG 경로 (기본 <run>/plot.png)")
    a = ap.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 이 없습니다: pip install matplotlib")
        return 1

    d = load(a.run)
    joints = [j.strip() for j in a.joints.split(",") if j.strip()]
    for j in joints:
        if j not in JOINT_SHORT:
            print(f"알 수 없는 관절 {j}. 가능: {JOINT_SHORT}")
            return 1
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for j in joints:
        axes[0].plot(d["t"], d[f"qdes_{j}"], "--", label=f"{j} q_des")
        axes[0].plot(d["t"], d[f"q_{j}"], label=f"{j} q")
        axes[1].plot(d["t"], d[f"tau_{j}"], label=f"{j} tau")
    axes[0].set_ylabel("angle [rad]")
    axes[1].set_ylabel("torque [N·m]")
    axes[2].plot(d["t"], d["grav_z"], label="gravity_z (body)")
    if np.isfinite(d["base_z"]).any():
        axes[2].plot(d["t"], d["base_z"], label="base_z [m]")
    axes[2].set_ylabel("posture")
    axes[2].set_xlabel("t [s]")
    # 단계 경계 표시
    phases = d["phase"]
    for i in range(1, len(phases)):
        if phases[i] != phases[i - 1]:
            for ax in axes:
                ax.axvline(d["t"][i], color="gray", alpha=0.3, lw=0.8)
            axes[0].text(d["t"][i], axes[0].get_ylim()[1], phases[i], fontsize=7, va="top", color="gray")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(f"{a.run}  ({len(d['t'])} ticks)")
    out = a.out or (a.run / "plot.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"저장: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
