#!/usr/bin/env python3
"""Isaac Sim 백엔드 연기 시험: URDF import → 관절 이름 매핑 → 댐핑 2 s → 기본 자세 유지 3 s. Isaac Sim 5.x + Python 3.11 필요.

    .venv-isaac\\Scripts\\python tools\\isaac_smoke.py --viewer        # 창을 띄워 확인
    .venv-isaac\\Scripts\\python tools\\isaac_smoke.py                 # 헤드리스

통과 기준: 오류 없이 끝나고, 유지 구간 추종 오차가 kp 30 에서 0.1 rad 이하(몸통 고정, 다리 늘어뜨림), 기록이 runs/isaac-smoke/ 에 남는다.
그 기록은 datalab 에서 MuJoCo 기록과 그대로 비교할 수 있다 (같은 형식).
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.backend_isaac import IsaacBackend                # noqa: E402
from lowlevel.common import KD_DAMP, Q_DEFAULT, JointCmd, fmt   # noqa: E402
from lowlevel.runtime import Session                           # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--viewer", action="store_true", help="Isaac Sim 창 표시 (기본 헤드리스)")
    ap.add_argument("--free-base", action="store_true", help="몸통 자유 (기본은 공중 고정)")
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--kd", type=float, default=1.2)
    a = ap.parse_args()

    io = IsaacBackend(urdf_path=a.urdf, headless=not a.viewer, fixed_base=not a.free_base)
    state0 = io.wait_ready()
    print("dof 순서 OK, 첫 상태 q =", fmt(state0.q), "base_z =", round(float(state0.extra["base_pos"][2]), 3))
    errs = []
    with Session(io, dt=0.005, realtime=a.viewer, exit_mode="damp", log_path=Path("runs/isaac-smoke/trace.csv"),
                 meta={"program": "isaac_smoke", "params": {"kp": a.kp, "kd": a.kd}, "tags": ["isaac"]}) as s:
        for t, state in s.run(5.0):
            if t < 2.0:
                s.phase = "damp"
                cmd = s.send(JointCmd.damping(KD_DAMP, state.q))
            else:
                s.phase = "hold"
                cmd = s.send(JointCmd.pd(Q_DEFAULT, a.kp, a.kd))
                if t > 4.5:
                    errs.append(np.abs(cmd.q - state.q))
            if s.tick % 200 == 0:
                print(f"t={t:4.1f}s {s.phase:<5} q={fmt(state.q)} |tau|max={abs(state.tau_est).max():.2f}")
    if errs:
        e = np.array(errs)
        print(f"유지 오차: 평균 {e.mean():.4f} rad, 최대 {e.max():.4f} rad → {'PASS' if e.max() < 0.1 else 'CHECK'}")
    print("종료 사유:", s.reason)


if __name__ == "__main__":
    main()
