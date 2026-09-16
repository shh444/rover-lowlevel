#!/usr/bin/env python3
"""예제 2: 댐핑 명령만 보내기 — 실기에서 보내는 첫 저수준 명령으로 적합하다.

kp 0, kd 3 이므로 움직이지 않는다. 명령이 수락되는지(mode 유지), 토크 추정값이 작은지만 본다.
Session 이 페이싱(200Hz)·가드·Ctrl+C·안전 종료(댐핑 1초)를 맡는다.

    python examples/e2_damping.py --backend mujoco
    python examples/e2_damping.py --backend dds --seconds 3      # 실기: kill_robot 이후
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_DAMP, JointCmd, fmt   # noqa: E402
from lowlevel.runtime import Session, make_backend   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--yes", action="store_true", help="실기 체크리스트 프롬프트 생략")
    ap.add_argument("--log", default=None, help="trace.csv 저장 경로 (선택)")
    a = ap.parse_args()

    io = make_backend(a.backend, yes=a.yes)          # dds: 주 제어기 검사 + 체크리스트
    io.wait_ready()
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="damp", log_path=a.log) as s:
        for t, state in s.run(a.seconds):
            s.send(JointCmd.damping(KD_DAMP, state.q))
            if s.tick % 200 == 0:
                print(f"t={t:4.1f}s q={fmt(state.q)} |tau|max={abs(state.tau_est).max():.2f} age={state.age * 1e3:.0f}ms")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
