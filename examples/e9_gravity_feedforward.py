#!/usr/bin/env python3
"""예제 9: tau 피드포워드(중력 보상)와 몸통 고정 시험 모드 — 시뮬 전용.

SDK 토크 공식 tau = kp(q_des-q) + kd(dq_des-dq) + tau 의 마지막 항을 쓰는 예다. 몸통을 공중에 고정한
모델(fixed_base=True, '로봇을 지지한 상태')에서 다리를 늘어뜨린 기본 자세를 낮은 게인(kp 10)으로 유지하면
다리 무게 때문에 처진다. 명목 모델의 중력 토크 g(q) 를 tau 로 함께 보내면 처짐이 거의 사라진다.
   A 구간(3s): tau=0        → 정상 상태 오차 큼
   B 구간(3s): tau=g(q)     → 오차 작음
실기에서 같은 방식을 쓰려면 명목 모델(질량·관성)이 실물과 맞아야 하고, 지지된 상태에서만 시험한다.

    MUJOCO_GL=egl python examples/e9_gravity_feedforward.py --kp 10 --kd 1
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.backend_mujoco import MujocoBackend      # noqa: E402
from lowlevel.common import Q_DEFAULT, JointCmd, fmt   # noqa: E402
from lowlevel.runtime import Session                   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=3.0, help="구간당 시간 s")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    io = MujocoBackend(fixed_base=True)                 # 몸통 고정, 다리는 기본 자세로 늘어뜨린 채 시작
    io.wait_ready()
    errors = {"A_no_ff": [], "B_gravity_ff": []}
    with Session(io, dt=0.005, realtime=False, exit_mode="damp", log_path=a.log) as s:
        for t, state in s.run(2 * a.seconds):
            phase = "A_no_ff" if t < a.seconds else "B_gravity_ff"
            tau_ff = np.zeros(12) if phase == "A_no_ff" else io.nominal_gravity(state.q, state.quat_wxyz)
            s.phase = phase
            cmd = s.send(JointCmd.pd(Q_DEFAULT, a.kp, a.kd, tau=tau_ff))
            err = np.abs(cmd.q - state.q)
            if (t % a.seconds) > a.seconds - 0.5:       # 각 구간 마지막 0.5초를 정상 상태로 본다
                errors[phase].append(err)
            if s.tick % 100 == 0:
                print(f"t={t:4.2f}s {phase:<12} |err|max={err.max():.3f} |tau_ff|max={abs(tau_ff).max():.2f} "
                      f"|tau|max={abs(state.tau_est).max():.2f}")
    for k, v in errors.items():
        v = np.array(v)
        print(f"{k:<12} 정상 상태 오차: 평균 {v.mean():.4f} rad, 최대 {v.max():.4f} rad, 관절별 {fmt(v.mean(axis=0))}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
