#!/usr/bin/env python3
"""예제 1: 상태 모니터. 관절각·속도·토크·IMU·모터 mode 를 0.5초마다 출력한다.

실기에서는 읽기 전용(LowerCmd writer 를 만들지 않음)이라 kill_robot 없이도 안전하게 돌릴 수 있다.

    python examples/e1_state_monitor.py --backend mujoco --seconds 3
    python examples/e1_state_monitor.py --backend dds --seconds 5      # 실기: CYCLONEDDS_URI 필요
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_DAMP, LEGS, JointCmd, fmt   # noqa: E402
from lowlevel.runtime import make_backend                  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--seconds", type=float, default=3.0)
    a = ap.parse_args()

    io = make_backend(a.backend, read_only=True)     # dds: writer 없음 → 명령을 보낼 수 없다
    state = io.wait_ready()
    print(f"첫 상태 수신. 관절 순서 FL(abad,thigh,calf) FR RL RR, 오프셋 제거된 논리각. "
          f"backend={io.name}")
    dt, t0, last_print = 0.005, time.monotonic(), -1.0
    try:
        while True:
            t = time.monotonic() - t0
            if t >= a.seconds:
                break
            state = io.read()
            if io.name == "mujoco":
                io.send(JointCmd.damping(KD_DAMP, state.q))   # 시뮬은 명령을 보내야 물리가 진행된다 (댐핑 = 힘 없음)
            if t - last_print >= 0.5:
                last_print = t
                print(f"t={t:4.1f}s age={state.age * 1e3:.0f}ms grav_z={state.gravity_body()[2]:+.3f} "
                      f"|dq|max={abs(state.dq).max():.2f} |tau|max={abs(state.tau_est).max():.2f}")
                for k, leg in enumerate(LEGS):
                    print(f"   {leg}: q={fmt(state.q[3 * k:3 * k + 3])} tau={fmt(state.tau_est[3 * k:3 * k + 3], 2)}")
                if state.motor_mode is not None:
                    print(f"   mode={state.motor_mode.tolist()} temp={state.extra.get('temp_hw')}")
            time.sleep(dt)
    finally:
        io.close()


if __name__ == "__main__":
    main()
