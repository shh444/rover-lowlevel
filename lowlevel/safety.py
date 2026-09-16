"""명령/상태 안전 가드.

인증된 보호 장치가 아니라 실습용 최소 방어선이다. 실기에서는 독립된 비상정지(전원 차단)를
반드시 따로 준비한다. Guard 는 다음을 한다.
  check(state)      : 비정상(NaN)·오래된 상태·과속·넘어짐 → SafetyAbort (→ 실행기가 댐핑으로 안전 종료)
  limit(cmd, state) : 목표각을 URDF 범위로 자르고 변화율(slew)을 제한, kp/kd/tau_ff 상한 적용
"""
from __future__ import annotations

import numpy as np

from .common import (DQ_MAX, JOINT_SHORT, KD_MAX, KP_MAX, Q_LOWER, Q_UPPER, TAU_FF_MAX,
                     JointCmd, State)


class SafetyAbort(RuntimeError):
    """즉시 댐핑으로 넘어가야 하는 상태 이상."""


class Guard:
    def __init__(self, dt: float, slew: float = 2.0, state_timeout: float = 0.2,
                 fall_threshold: float = -0.866, check_fall: bool = True,
                 range_margin: float = 0.3):
        self.dt = float(dt)
        self.slew = float(slew)                      # rad/s
        self.state_timeout = float(state_timeout)    # s
        self.fall_threshold = float(fall_threshold)  # gravity_z 가 이보다 크면 넘어짐 (-0.866 = 30도)
        self.check_fall = bool(check_fall)
        self.range_margin = float(range_margin)
        self._q_ref: np.ndarray | None = None
        self._warned: set[int] = set()

    def check(self, state: State) -> None:
        for name in ("q", "dq", "quat_wxyz", "gyro"):
            if not np.all(np.isfinite(getattr(state, name))):
                raise SafetyAbort(f"nonfinite:{name}")
        if state.age > self.state_timeout:
            raise SafetyAbort(f"state_stale:{state.age * 1e3:.0f}ms")
        speed = float(np.max(np.abs(state.dq)))
        if speed > DQ_MAX:
            raise SafetyAbort(f"overspeed:{speed:.1f}rad/s")
        if self.check_fall:
            gz = float(state.gravity_body()[2])
            if gz > self.fall_threshold:
                raise SafetyAbort(f"fallen:gravity_z={gz:+.2f}")
        outside = (state.q < Q_LOWER - self.range_margin) | (state.q > Q_UPPER + self.range_margin)
        for j in np.flatnonzero(outside):
            j = int(j)
            if j not in self._warned:
                self._warned.add(j)
                print(f"[경고] {JOINT_SHORT[j]} 측정각 {state.q[j]:+.3f} rad 가 URDF 범위 밖")

    def limit(self, cmd: JointCmd, state: State) -> JointCmd:
        kp = np.clip(cmd.kp, 0.0, KP_MAX)
        kd = np.clip(cmd.kd, 0.0, KD_MAX)
        tau = np.clip(cmd.tau, -TAU_FF_MAX, TAU_FF_MAX)
        dq = np.clip(cmd.dq, -DQ_MAX, DQ_MAX)
        q = np.clip(cmd.q, Q_LOWER, Q_UPPER)
        if np.all(kp == 0.0):
            # 댐핑 중엔 목표각이 힘을 내지 않는다. 기준을 실제각에 맞춰 두어야
            # 다음에 kp 가 켜질 때 점프가 없다.
            self._q_ref = state.q.copy()
        else:
            if self._q_ref is None:
                self._q_ref = state.q.copy()
            step = self.slew * self.dt
            q = self._q_ref + np.clip(q - self._q_ref, -step, step)
            self._q_ref = q
        return JointCmd(q, dq, kp, kd, tau)

    def reset_reference(self, q: np.ndarray) -> None:
        self._q_ref = np.asarray(q, dtype=float).copy()
