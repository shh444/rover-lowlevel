#!/usr/bin/env python3
"""예제 8: 서 있는 자세에서 자세 보간 — 스쿼트(앉았다 서기) N 회. 자세 공간에서 동작을 만드는 법.

두 자세(서기 Q_DEFAULT ↔ 낮은 자세 Q_LOW)를 코사인 보간으로 오가며 기립 게인(kp 60, kd 1.8)으로 추종한다.
Q_LOW 는 발끝이 엉덩이 아래에 오도록 다리 길이만 줄인 자세다 (thigh 를 늘리고 calf 를 더 접음).
시뮬은 --start standing 으로 서 있는 상태에서 시작한다. 실기는 기립이 검증된 뒤에만 쓴다.

    MUJOCO_GL=egl python examples/e8_squat.py --backend mujoco --reps 2 --snapshot
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_STAND, KP_STAND, Q_DEFAULT, JointCmd, cosine_interp   # noqa: E402
from lowlevel.runtime import Session, make_backend                                     # noqa: E402

# 낮은 자세: 앞다리 thigh 1.15 / calf -1.85, 뒷다리 1.05 / -1.78 → 다리 높이 약 0.36 m → 0.28 m
Q_LOW = np.array([0.0, 1.15, -1.85, 0.0, 1.15, -1.85,
                  0.0, 1.05, -1.78, 0.0, 1.05, -1.78])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--down-s", type=float, default=1.5, help="내려가는/올라오는 시간 s")
    ap.add_argument("--hold-s", type=float, default=1.0, help="아래에서 머무는 시간 s")
    ap.add_argument("--snapshot", action="store_true", help="가장 낮은 시점 이미지 저장 (시뮬)")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    io = make_backend(a.backend, yes=a.yes, start="standing")
    state0 = io.wait_ready()
    q_from = state0.q.copy()                        # 처음 0.5초는 현재 자세 → 서기 자세로 맞춘다
    cycle = 2 * a.down_s + a.hold_s
    total = 0.5 + a.reps * cycle
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="crouch", log_path=a.log) as s:
        for t, state in s.run(total):
            if t < 0.5:
                q = cosine_interp(q_from, Q_DEFAULT, t / 0.5)
                s.phase = "align"
            else:
                u = (t - 0.5) % cycle
                rep = int((t - 0.5) // cycle) + 1
                if u < a.down_s:
                    q, s.phase = cosine_interp(Q_DEFAULT, Q_LOW, u / a.down_s), f"down{rep}"
                elif u < a.down_s + a.hold_s:
                    q, s.phase = Q_LOW, f"low{rep}"
                    if a.snapshot and abs(u - a.down_s - a.hold_s / 2) < 0.0026:
                        io.snapshot(Path("runs") / f"squat_low{rep}.jpg")
                else:
                    q, s.phase = cosine_interp(Q_LOW, Q_DEFAULT, (u - a.down_s - a.hold_s) / a.down_s), f"up{rep}"
            cmd = s.send(JointCmd.pd(q, KP_STAND, KD_STAND))
            if s.tick % 100 == 0:
                base_z = state.extra.get("base_pos", (math.nan,) * 3)[2]
                print(f"t={t:4.2f}s {s.phase:<6} base_z={base_z:.3f} grav_z={state.gravity_body()[2]:+.3f} "
                      f"|err|max={abs(cmd.q - state.q).max():.3f} |tau|max={abs(state.tau_est).max():.1f}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
