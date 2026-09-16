#!/usr/bin/env python3
"""예제 5: dobot_sim2real 의 3단계 기립 → 유지 → (안전 종료) 엎드림 → 댐핑.

    1) 댐핑 2s → 2) 엎드림 보간 3s (kp 5→60) → 3) 기립 보간 3s (kp 60) → 4) --hold 초 유지
시뮬에서는 언제든 돌려도 된다. 실기는 평평한 바닥에 엎드린 상태에서, 주변을 비우고 전원 차단을 준비한 뒤에만.
(실기 기립은 2026-09-16 기준 아직 실행하지 않았다.)

    MUJOCO_GL=egl python examples/e5_standup.py --backend mujoco --hold 3
    python examples/e5_standup.py --backend dds --hold 3
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel import programs                       # noqa: E402
from lowlevel.runtime import Session, make_backend  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--hold", type=float, default=3.0, help="선 뒤 유지 시간 s")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    io = make_backend(a.backend, yes=a.yes)
    state0 = io.wait_ready()
    prog = programs.StandUpProgram(state0, hold_s=a.hold)
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="crouch", log_path=a.log) as s:
        for t, state in s.run():
            cmd = prog.step(t, state)
            if cmd is None:                          # 프로그램 끝 → with 를 빠져나가며 엎드림 + 댐핑
                break
            s.phase = prog.phase
            s.send(cmd)
            if s.tick % 200 == 0:
                base_z = state.extra.get("base_pos", (math.nan,) * 3)[2]
                print(f"t={t:5.2f}s phase={prog.phase:<6} grav_z={state.gravity_body()[2]:+.3f} "
                      f"base_z={base_z:.3f} |err|max={abs(cmd.q - state.q).max():.3f} "
                      f"|tau|max={abs(state.tau_est).max():.1f}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
