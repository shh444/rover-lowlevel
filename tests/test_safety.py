"""안전 가드·공통 함수·프로그램 단계 검증 (MuJoCo, DDS 불필요).

    python tests/test_safety.py
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lowlevel import programs                                                   # noqa: E402
from lowlevel.common import (KP_MAX, Q_DEFAULT, Q_UPPER, JointCmd, State,       # noqa: E402
                             cosine_interp, joint_mask)
from lowlevel.safety import Guard, SafetyAbort                                  # noqa: E402


def mk(q=0.0, quat=(1, 0, 0, 0), age=0.0, dq=0.0):
    return State(t=0.0, q=np.full(12, q, float), dq=np.full(12, dq, float),
                 quat_wxyz=np.array(quat, float), gyro=np.zeros(3), acc=np.zeros(3),
                 tau_est=np.zeros(12), age=age)


def expect_abort(guard, state, key):
    try:
        guard.check(state)
    except SafetyAbort as exc:
        assert key in str(exc), (key, str(exc))
        return
    raise AssertionError(f"SafetyAbort({key}) 가 나야 함")


def main():
    # 몸체 좌표계 중력 방향
    assert np.allclose(mk().gravity_body(), [0, 0, -1])
    r = math.radians(90)
    assert np.allclose(mk(quat=(math.cos(r / 2), math.sin(r / 2), 0, 0)).gravity_body(), [0, -1, 0], atol=1e-9)
    assert np.allclose(mk(quat=(math.cos(r / 2), 0, math.sin(r / 2), 0)).gravity_body(), [1, 0, 0], atol=1e-9)
    tilt45 = (math.cos(math.radians(22.5)), math.sin(math.radians(22.5)), 0, 0)
    assert math.isclose(mk(quat=tilt45).gravity_body()[2], -math.cos(math.radians(45)), abs_tol=1e-9)

    # 가드: 정상 / 넘어짐 / 오래된 상태 / 과속 / NaN
    g = Guard(dt=0.005)
    g.check(mk())
    expect_abort(g, mk(quat=tilt45), "fallen")
    expect_abort(g, mk(age=0.5), "state_stale")
    expect_abort(g, mk(dq=30.0), "overspeed")
    expect_abort(g, mk(q=float("nan")), "nonfinite")
    Guard(dt=0.005, check_fall=False).check(mk(quat=tilt45))

    # 목표 제한: 범위 자르기, 변화율 제한, 게인 상한, 댐핑 시 기준 추종
    g = Guard(dt=0.005, slew=2.0)
    st = mk(q=0.0)
    out = g.limit(JointCmd.pd(10.0, 1000.0, 50.0), st)
    assert np.allclose(out.q, 2.0 * 0.005) and np.all(out.kp == KP_MAX) and np.all(out.kd == 5.0)
    for _ in range(1000):
        out = g.limit(JointCmd.pd(10.0, 30.0, 1.0), st)
    assert np.allclose(out.q, Q_UPPER)                      # 결국 URDF 상한에서 멈춘다
    out = g.limit(JointCmd.damping(3.0, 0.0), mk(q=0.3))
    assert out.is_damping()
    out = g.limit(JointCmd.pd(0.3, 30.0, 1.0), mk(q=0.3))
    assert np.allclose(out.q, 0.3)                          # 댐핑 후 실제각에서 점프 없이 시작

    # 보간·마스크
    assert np.allclose(cosine_interp(np.zeros(3), np.ones(3), 0.5), 0.5)
    assert np.allclose(cosine_interp(np.zeros(3), np.ones(3), 1.7), 1.0)
    assert joint_mask("calf").tolist() == [False, False, True] * 4
    assert joint_mask("FL,RR").sum() == 6 and joint_mask("0,5").sum() == 2 and joint_mask("all").all()

    # 기립 프로그램 단계 순서와 게인
    p = programs.StandUpProgram(mk(q=0.5), hold_s=1.0)
    c = p.step(0.0, mk(q=0.5)); assert p.phase == "damp" and c.is_damping()
    c = p.step(2.0, mk(q=0.5)); assert p.phase == "crouch" and math.isclose(c.kp[0], 5.0) and np.allclose(c.q, 0.5)
    c = p.step(3.5, mk(q=0.5)); assert p.phase == "crouch" and 5.0 < c.kp[0] < 60.0
    c = p.step(5.0, mk(q=0.5)); assert p.phase == "stand" and np.allclose(c.q, programs.Q_CROUCH_SAFE)
    c = p.step(8.0, mk(q=0.5)); assert p.phase == "hold" and np.allclose(c.q, Q_DEFAULT) and c.kp[0] == 60.0
    assert p.step(9.0, mk(q=0.5)) is None

    # 사인 프로그램: settle 후 초기각 기준, 마스크 적용
    p = programs.SineProgram(mk(q=0.1), duration=2.0, amp=0.2, freq=1.0, mask=joint_mask("calf"), ramp_s=0.0)
    assert p.step(0.5, mk(q=0.1)).is_damping()
    c = p.step(1.25, mk(q=0.4))                              # settle 끝난 시점의 각도(0.4)가 기준
    assert np.allclose(c.q[2::3], 0.4 + 0.2) and np.allclose(c.q[0::3], 0.4)
    assert p.step(3.1, mk(q=0.4)) is None
    print("test_safety: OK")


if __name__ == "__main__":
    main()
