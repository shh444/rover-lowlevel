#!/usr/bin/env python3
"""예제 4: SDK E9 식 사인 구동 — 1초 댐핑으로 초기각을 잡고, 진폭을 1초에 걸쳐 키운 뒤 q = q0 + amp·sin(2πft).

E9 원본은 12관절 ±0.2 rad, kp 30, kd 1.2 이다. 실기 첫 동작은 thigh ±0.1 rad 로 검증했다 (2026-09-16).
목표각은 가드가 URDF 범위와 변화율(2 rad/s)로 제한하고, 종료 시 댐핑 1초를 보낸다.

    python examples/e4_sine_joint.py --backend mujoco --joints thigh --amp 0.1
    python examples/e4_sine_joint.py --backend dds --joints thigh --amp 0.1 --seconds 5
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_DAMP, KD_SINE, KP_SINE, JointCmd, joint_mask   # noqa: E402
from lowlevel.runtime import Session, make_backend                             # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--seconds", type=float, default=5.0, help="사인 구동 시간 (앞의 1초 댐핑은 별도)")
    ap.add_argument("--joints", default="thigh", help="all | abad,thigh,calf | FL,FR,RL,RR | 0,1,2")
    ap.add_argument("--amp", type=float, default=0.1, help="진폭 rad")
    ap.add_argument("--freq", type=float, default=0.9, help="주파수 Hz")
    ap.add_argument("--kp", type=float, default=KP_SINE)
    ap.add_argument("--kd", type=float, default=KD_SINE)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    mask = joint_mask(a.joints)
    io = make_backend(a.backend, yes=a.yes)
    io.wait_ready()
    q0 = None
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="damp", log_path=a.log) as s:
        for t, state in s.run(1.0 + a.seconds):
            if t < 1.0:                                   # E9: 초기 위치 수집 (여기서는 댐핑 상태로 1초)
                s.phase = "settle"
                s.send(JointCmd.damping(KD_DAMP, state.q))
                continue
            if q0 is None:
                q0 = state.q.copy()
            tt = t - 1.0
            env = min(1.0, tt / 1.0)                      # 진폭 램프 1초
            s.phase = "sine"
            q_des = q0 + mask * a.amp * env * math.sin(2.0 * math.pi * a.freq * tt)
            cmd = s.send(JointCmd.pd(q_des, a.kp, a.kd))
            if s.tick % 100 == 0:
                j = int(mask.argmax())
                print(f"t={t:4.2f}s joint[{j}] q={state.q[j]:+.3f} q_des={cmd.q[j]:+.3f} "
                      f"tau={state.tau_est[j]:+.2f} |err|max={abs(cmd.q - state.q).max():.3f}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
