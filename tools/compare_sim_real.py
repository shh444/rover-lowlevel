#!/usr/bin/env python3
"""MuJoCo 와 실기 비교: 실기 기록(trace.csv)의 명령(q_des, kp, kd)을 시뮬에 그대로 재생해 관절각·토크를 비교한다.

절차
  1. 실기 기록의 첫 관절각으로 시뮬을 초기화하고 몸통을 --base-z 높이에 둔다 (실기 READY 자세는 무릎을 꿇고
     정강이를 바닥에 댄 자세라 몸통이 약 0.2 m 높이에 있다).
  2. --place 초 동안 kp 30 으로 그 자세를 잡은 채 바닥에 안착시킨다 (안착 후 자세 변화량을 보고한다).
  3. 실기 기록의 모든 틱을 같은 주기로 재생한다: 틱마다 JointCmd.pd(q_des_i, kp_i, kd_i).
  4. 관절별 RMSE·최대차, 구동 관절의 진폭비(실기/시뮬 vs 명령)와 명령 대비 지연(상호상관), 토크 피크를 계산한다.

    python tools/compare_sim_real.py runs/real-sine-01 --joints FL_thigh,FR_thigh --out runs/compare-sine-01

matplotlib 이 있으면 PNG 그래프도 저장한다.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lowlevel.backend_mujoco import MujocoBackend      # noqa: E402
from lowlevel.common import JOINT_SHORT, JointCmd      # noqa: E402


def load_trace(run_dir: Path):
    rows = list(csv.DictReader(open(run_dir / "trace.csv", encoding="utf-8")))
    col = lambda name: np.array([float(r[name]) for r in rows])   # noqa: E731
    return {
        "t": col("t"), "phase": [r["phase"] for r in rows], "kp": col("kp"), "kd": col("kd"),
        "q": np.column_stack([col(f"q_{j}") for j in JOINT_SHORT]),
        "qdes": np.column_stack([col(f"qdes_{j}") for j in JOINT_SHORT]),
        "tau": np.column_stack([col(f"tau_{j}") for j in JOINT_SHORT]),
    }


def lag_ms(reference: np.ndarray, signal: np.ndarray, dt: float, max_lag_s: float = 0.5):
    """signal 이 reference 보다 얼마나 늦는지(ms). 평균을 뺀 신호의 상호상관 최대 지점."""
    a = reference - reference.mean()
    b = signal - signal.mean()
    if a.std() < 1e-6 or b.std() < 1e-6:
        return None
    max_lag = int(max_lag_s / dt)
    best, best_corr = 0, -np.inf
    for lag in range(0, max_lag + 1):          # signal 이 lag 만큼 늦다고 가정
        c = float(np.dot(a[:len(a) - lag], b[lag:]))
        if c > best_corr:
            best, best_corr = lag, c
    return best * dt * 1e3


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real", type=Path, help="실기 기록 폴더 (trace.csv)")
    ap.add_argument("--out", type=Path, default=None, help="결과 폴더 (기본 <real>/compare)")
    ap.add_argument("--joints", default="FL_thigh,FR_thigh", help="그래프·지연 계산에 쓸 관절")
    ap.add_argument("--base-z", type=float, default=0.22, help="시뮬 초기 몸통 높이 m (바닥 모드)")
    ap.add_argument("--fixed-base", action="store_true",
                    help="몸통을 공중에 고정(거치대에 지지된 로봇, 다리는 자유). 실기 GRF≈0·토크≈0 이면 이 모드가 맞다")
    ap.add_argument("--place", type=float, default=1.0, help="안착 시간 s (kp 30 으로 초기 자세 유지)")
    ap.add_argument("--physics-dt", type=float, default=0.001)
    ap.add_argument("--snapshot", action="store_true", help="안착 직후 시뮬 이미지 저장")
    a = ap.parse_args()
    out = a.out or (a.real / "compare")
    out.mkdir(parents=True, exist_ok=True)

    real = load_trace(a.real)
    n = len(real["t"])
    dt = float(np.median(np.diff(real["t"]))) if n > 1 else 0.005
    q0 = real["q"][0]
    io = MujocoBackend(dt=dt, physics_dt=a.physics_dt, init_q=q0, init_base_z=a.base_z, fixed_base=a.fixed_base)

    # 2) 안착
    for _ in range(int(round(a.place / dt))):
        io.send(JointCmd.pd(q0, 30.0, 1.2))
    placed = io.read()
    drift = placed.q - q0
    base_after_place = float(placed.extra["base_pos"][2])
    if a.snapshot:
        io.snapshot(out / "sim_placed.jpg")

    # 3) 재생
    q_sim = np.zeros_like(real["q"])
    tau_sim = np.zeros_like(real["tau"])
    base_z = np.zeros(n)
    for i in range(n):
        st = io.read()
        q_sim[i], tau_sim[i], base_z[i] = st.q, st.tau_est, st.extra["base_pos"][2]
        io.send(JointCmd.pd(real["qdes"][i], real["kp"][i], real["kd"][i]))
    io.close()

    # 4) 지표
    diff = q_sim - real["q"]
    driven = [j for j in range(12) if real["qdes"][:, j].std() > 0.01]
    per_joint = []
    for j in range(12):
        row = {"joint": JOINT_SHORT[j], "driven": j in driven,
               "rmse_rad": float(np.sqrt(np.mean(diff[:, j] ** 2))), "maxabs_rad": float(np.abs(diff[:, j]).max()),
               "tau_peak_real": float(np.abs(real["tau"][:, j]).max()), "tau_peak_sim": float(np.abs(tau_sim[:, j]).max()),
               "placement_drift_rad": float(drift[j])}
        if j in driven:
            p2p = lambda x: float(x.max() - x.min())   # noqa: E731
            row.update({"p2p_cmd": p2p(real["qdes"][:, j]), "p2p_real": p2p(real["q"][:, j]), "p2p_sim": p2p(q_sim[:, j]),
                        "amp_ratio_real": p2p(real["q"][:, j]) / max(p2p(real["qdes"][:, j]), 1e-9),
                        "amp_ratio_sim": p2p(q_sim[:, j]) / max(p2p(real["qdes"][:, j]), 1e-9),
                        "lag_real_ms": lag_ms(real["qdes"][:, j], real["q"][:, j], dt),
                        "lag_sim_ms": lag_ms(real["qdes"][:, j], q_sim[:, j], dt),
                        "track_rmse_real": float(np.sqrt(np.mean((real["q"][:, j] - real["qdes"][:, j]) ** 2))),
                        "track_rmse_sim": float(np.sqrt(np.mean((q_sim[:, j] - real["qdes"][:, j]) ** 2)))})
        per_joint.append(row)
    summary = {
        "real_run": str(a.real), "ticks": n, "dt_s": dt, "driven_joints": [JOINT_SHORT[j] for j in driven],
        "sim": {"xml": str(io.xml), "mode": "fixed_base" if a.fixed_base else "ground",
                "physics_dt": a.physics_dt, "init_base_z": a.base_z, "base_z_after_place": base_after_place,
                "base_z_min": float(base_z.min()), "base_z_max": float(base_z.max()),
                "placement_drift_max_rad": float(np.abs(drift).max())},
        "all_joints_rmse_rad": float(np.sqrt(np.mean(diff ** 2))),
        "driven_joints_rmse_rad": float(np.sqrt(np.mean(diff[:, driven] ** 2))) if driven else None,
        "per_joint": per_joint,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    np.savetxt(out / "q_sim.csv", np.column_stack([real["t"], q_sim, tau_sim, base_z]), delimiter=",",
               header="t," + ",".join(f"qsim_{j}" for j in JOINT_SHORT) + "," + ",".join(f"tausim_{j}" for j in JOINT_SHORT) + ",base_z",
               comments="", fmt="%.5f")

    print(f"실기 {a.real}: {n}틱 dt={dt * 1e3:.1f}ms, 구동 관절 {summary['driven_joints']}")
    print(f"시뮬 모드: {summary['sim']['mode']}, 안착: 몸통 {a.base_z:.3f} → {base_after_place:.3f} m, "
          f"관절 변화 최대 {np.abs(drift).max():.3f} rad")
    print(f"관절각 차이 RMSE: 전체 {summary['all_joints_rmse_rad']:.4f} rad, 구동 관절 {summary['driven_joints_rmse_rad'] or 0:.4f} rad")
    print(f"{'joint':<9}{'rmse':>7}{'max':>7}{'ratio_r':>9}{'ratio_s':>9}{'lag_r':>8}{'lag_s':>8}{'tau_r':>7}{'tau_s':>7}")
    for r in per_joint:
        if r["driven"]:
            print(f"{r['joint']:<9}{r['rmse_rad']:7.3f}{r['maxabs_rad']:7.3f}{r['amp_ratio_real']:9.2f}{r['amp_ratio_sim']:9.2f}"
                  f"{r['lag_real_ms'] or 0:8.0f}{r['lag_sim_ms'] or 0:8.0f}{r['tau_peak_real']:7.2f}{r['tau_peak_sim']:7.2f}")
        else:
            print(f"{r['joint']:<9}{r['rmse_rad']:7.3f}{r['maxabs_rad']:7.3f}{'-':>9}{'-':>9}{'-':>8}{'-':>8}{r['tau_peak_real']:7.2f}{r['tau_peak_sim']:7.2f}")

    # 5) 그래프 (선택)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음: 그래프 생략")
        return 0
    joints = [j.strip() for j in a.joints.split(",") if j.strip()]
    fig, axes = plt.subplots(len(joints) + 1, 1, figsize=(10, 3 * (len(joints) + 1)), sharex=True)
    t = real["t"]
    for ax, name in zip(axes, joints):
        j = JOINT_SHORT.index(name)
        ax.plot(t, real["qdes"][:, j], "k--", lw=1, label="q_des (command)")
        ax.plot(t, real["q"][:, j], label="real q")
        ax.plot(t, q_sim[:, j], label="sim q")
        ax.set_ylabel(f"{name} [rad]")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    j = JOINT_SHORT.index(joints[0])
    axes[-1].plot(t, real["tau"][:, j], label=f"real tau_est {joints[0]}")
    axes[-1].plot(t, tau_sim[:, j], label=f"sim tau {joints[0]}")
    axes[-1].set_ylabel("torque [N·m]")
    axes[-1].set_xlabel("t [s]")
    axes[-1].grid(alpha=0.3)
    axes[-1].legend(loc="upper right", fontsize=8)
    fig.suptitle(f"real {a.real.name} vs MuJoCo replay ({summary['sim']['mode']}, same q_des/kp/kd)")
    fig.tight_layout()
    fig.savefig(out / "compare.png", dpi=110)
    print(f"그래프: {out / 'compare.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
