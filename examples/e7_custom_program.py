#!/usr/bin/env python3
"""예제 7: 나만의 동작(Program) 만들기 — "앞발 들기" (엎드린 상태에서 FL 허벅지를 들었다 내리기 N 회).

Program 은 step(t, state) 가 JointCmd 를 돌려주고 끝나면 None 을 돌려주는 객체면 된다.
목표각은 논리 관절 순서(FL/FR/RL/RR × abad/thigh/calf) 의 rad 값이고, 오프셋·슬롯·범위·변화율은
백엔드와 가드가 알아서 처리한다. 여기서 만든 클래스는 e5 처럼 Session 으로 돌린다.

    python examples/e7_custom_program.py --backend mujoco --reps 3
    python examples/e7_custom_program.py --backend dds --amp 0.15 --reps 2      # 실기: kill_robot 이후, 엎드린 상태
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import KD_DAMP, KD_SINE, KP_SINE, JointCmd, State, cosine_interp   # noqa: E402
from lowlevel.programs import Program                                                     # noqa: E402
from lowlevel.runtime import Session, make_backend                                        # noqa: E402

FL_THIGH = 1          # 논리 인덱스: FL abad=0, thigh=1, calf=2, FR 3-5, RL 6-8, RR 9-11


class PawLift(Program):
    """시작 자세 q0 에서 FL 허벅지를 amp 만큼 앞으로(각도 감소) 올렸다가 내리기를 reps 회 반복.
    각 동작은 코사인 보간으로 부드럽게 하고, 처음 1초는 댐핑으로 초기각을 잡는다."""
    name = "paw_lift"

    def __init__(self, state0: State, amp: float = 0.3, reps: int = 3, up_s: float = 0.8,
                 hold_s: float = 0.4, kp: float = KP_SINE, kd: float = KD_SINE):
        self.q0 = state0.q.copy()
        self.amp, self.reps, self.up_s, self.hold_s, self.kp, self.kd = amp, reps, up_s, hold_s, kp, kd
        self.cycle = 2 * up_s + hold_s                  # 올리기 → 유지 → 내리기
        self.settle_s = 1.0
        self._captured = False

    def step(self, t, state):
        if t < self.settle_s:
            self.phase = "settle"
            return JointCmd.damping(KD_DAMP, state.q)
        if not self._captured:
            self.q0 = state.q.copy()                    # 댐핑이 끝난 시점의 실제 자세를 기준으로
            self._captured = True
        tt = t - self.settle_s
        rep = int(tt // self.cycle)
        if rep >= self.reps:
            return None
        u = tt - rep * self.cycle
        if u < self.up_s:
            self.phase, s = f"up{rep + 1}", u / self.up_s
        elif u < self.up_s + self.hold_s:
            self.phase, s = f"hold{rep + 1}", 1.0
        else:
            self.phase, s = f"down{rep + 1}", 1.0 - (u - self.up_s - self.hold_s) / self.up_s
        q = self.q0.copy()
        q[FL_THIGH] = cosine_interp(np.array([self.q0[FL_THIGH]]), np.array([self.q0[FL_THIGH] - self.amp]), s)[0]
        return JointCmd.pd(q, self.kp, self.kd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("mujoco", "dds"), default="mujoco")
    ap.add_argument("--amp", type=float, default=0.3, help="허벅지 들어올리는 각도 rad")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()

    io = make_backend(a.backend, yes=a.yes)
    prog = PawLift(io.wait_ready(), amp=a.amp, reps=a.reps)
    with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="damp", log_path=a.log) as s:
        for t, state in s.run():
            cmd = prog.step(t, state)
            if cmd is None:
                break
            s.phase = prog.phase
            s.send(cmd)
            if s.tick % 100 == 0:
                print(f"t={t:4.2f}s {prog.phase:<7} FL_thigh q={state.q[FL_THIGH]:+.3f} "
                      f"q_des={cmd.q[FL_THIGH]:+.3f} tau={state.tau_est[FL_THIGH]:+.2f}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
