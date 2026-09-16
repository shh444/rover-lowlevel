#!/usr/bin/env python3
"""예제 3: 현재 자세 유지. kp 를 1초 동안 5 → kp 로 올려 위치 제어가 걸리는지 본다.

실기에서는 로봇을 지지한 상태(또는 엎드린 상태)에서 먼저 돌려 각도 부호와 토크 방향을 확인하는 단계다.
시뮬에서 --start standing 으로 서 있는 자세를 kp 60 으로 유지하면 중력 처짐(약 0.1 rad)을 볼 수 있다.

    python examples/e3_hold_pose.py --backend mujoco --start standing --kp 60 --kd 1.8
    python examples/e3_hold_pose.py --backend dds --seconds 5
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_SINE, KP_SINE, JointCmd   # noqa: E402
from lowlevel.runtime import Session, make_backend       # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--kp", type=float, default=KP_SINE)
    ap.add_argument("--kd", type=float, default=KD_SINE)
    ap.add_argument("--start", choices=("lying", "standing"), default="lying", help="시뮬 초기 자세")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    io = make_backend(a.backend, yes=a.yes, start=a.start)
    q0 = io.wait_ready().q.copy()                    # 유지할 자세 = 시작 시점의 측정각
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="damp", log_path=a.log) as s:
        for t, state in s.run(a.seconds):
            kp = 5.0 + (a.kp - 5.0) * min(1.0, t / 1.0)         # 1초 램프
            cmd = s.send(JointCmd.pd(q0, kp, a.kd))
            if s.tick % 200 == 0:
                print(f"t={t:4.1f}s kp={kp:4.1f} |q_des-q|max={abs(cmd.q - state.q).max():.3f} "
                      f"|tau|max={abs(state.tau_est).max():.2f}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
