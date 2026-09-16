"""관절 목표 생성 프로그램. 백엔드와 무관하게 논리 관절(12) 공간에서만 동작한다.

step(t, state) 가 JointCmd 를 돌려주고, 끝나면 None 을 돌려준다. t 는 프로그램 시작 후 경과 시간(s).
phase 속성은 기록용 단계 이름이다.
"""
from __future__ import annotations

import math

import numpy as np

from .common import (KD_DAMP, KD_SINE, KD_STAND, KP_SINE, KP_STAND, NUM_JOINTS, Q_CROUCH,
                     Q_DEFAULT, Q_LOWER, Q_UPPER, JointCmd, State, cosine_interp)

Q_CROUCH_SAFE = np.clip(Q_CROUCH, Q_LOWER, Q_UPPER)   # calf 는 -2.53 으로 잘림


class Program:
    name = "program"
    phase = "init"

    def step(self, t: float, state: State):
        raise NotImplementedError


class DampProgram(Program):
    """kp=0, kd=KD_DAMP 만 보낸다. 실기에서 최초 통신·모터 응답 확인용."""
    name = "damp"

    def __init__(self, duration: float, kd: float = KD_DAMP):
        self.duration, self.kd = float(duration), float(kd)

    def step(self, t, state):
        if t >= self.duration:
            return None
        self.phase = "damp"
        return JointCmd.damping(self.kd, state.q)


class HoldProgram(Program):
    """시작 시점의 관절각을 그대로 유지. kp 는 ramp_s 동안 5 → kp 로 올린다.
    실기 첫 단계(로봇을 지지한 상태에서 힘이 들어가는지 확인)에 쓴다."""
    name = "hold"

    def __init__(self, state0: State, duration: float, kp: float = KP_SINE, kd: float = KD_SINE,
                 ramp_s: float = 1.0):
        self.q0 = state0.q.copy()
        self.duration, self.kp, self.kd = float(duration), float(kp), float(kd)
        self.ramp_s = float(ramp_s)

    def step(self, t, state):
        if t >= self.duration:
            return None
        s = min(1.0, t / self.ramp_s) if self.ramp_s > 0 else 1.0
        self.phase = "ramp" if s < 1.0 else "hold"
        return JointCmd.pd(self.q0, 5.0 + (self.kp - 5.0) * s, self.kd)


class SineProgram(Program):
    """SDK E9 와 같은 사인 구동: q = q0 + amp*sin(2*pi*f*t). E9 기본값 amp 0.2 rad, kp 30, kd 1.2.
    E9 처럼 먼저 잠깐 댐핑으로 초기각을 읽은 뒤(settle), 진폭을 ramp_s 에 걸쳐 키운다."""
    name = "sine"

    def __init__(self, state0: State, duration: float, amp: float = 0.2, freq: float = 0.9,
                 mask: np.ndarray | None = None, kp: float = KP_SINE, kd: float = KD_SINE,
                 settle_s: float = 1.0, ramp_s: float = 1.0, kd_damp: float = KD_DAMP):
        self.duration, self.amp, self.freq = float(duration), float(amp), float(freq)
        self.mask = np.ones(NUM_JOINTS, dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
        self.kp, self.kd = float(kp), float(kd)
        self.settle_s, self.ramp_s, self.kd_damp = float(settle_s), float(ramp_s), float(kd_damp)
        self.q0: np.ndarray | None = None

    def step(self, t, state):
        if t < self.settle_s:
            self.phase = "settle"
            return JointCmd.damping(self.kd_damp, state.q)
        if self.q0 is None:
            self.q0 = state.q.copy()      # E9: 초기 위치 수집 완료 후 시작
        tt = t - self.settle_s
        if tt >= self.duration:
            return None
        env = min(1.0, tt / self.ramp_s) if self.ramp_s > 0 else 1.0
        self.phase = "ramp" if env < 1.0 else "sine"
        q = self.q0 + self.mask * self.amp * env * math.sin(2.0 * math.pi * self.freq * tt)
        return JointCmd.pd(q, self.kp, self.kd)


class StandUpProgram(Program):
    """dobot_sim2real standup_procedure 와 같은 3단계 기립 후 유지.
        1) 댐핑 2s            : kp 0, kd 3       (엎드린 자세로 자연스럽게 안착)
        2) 엎드림 보간 3s     : 현재 → crouch, kp 5→60, kd 1.8
        3) 기립 보간 3s       : crouch → default, kp 60, kd 1.8
        4) 유지 hold_s        : default, kp 60, kd 1.8
    """
    name = "standup"
    DAMP_S, CROUCH_S, STAND_S = 2.0, 3.0, 3.0

    def __init__(self, state0: State, hold_s: float = 5.0, kp_stand: float = KP_STAND,
                 kd_stand: float = KD_STAND, kd_damp: float = KD_DAMP):
        self.hold_s = float(hold_s)
        self.kp_stand, self.kd_stand, self.kd_damp = float(kp_stand), float(kd_stand), float(kd_damp)
        self.q_start: np.ndarray | None = None

    def step(self, t, state):
        if t < self.DAMP_S:
            self.phase = "damp"
            return JointCmd.damping(self.kd_damp, state.q)
        if self.q_start is None:
            self.q_start = state.q.copy()
        t1 = t - self.DAMP_S
        if t1 < self.CROUCH_S:
            self.phase = "crouch"
            s = t1 / self.CROUCH_S
            kp = 5.0 + (self.kp_stand - 5.0) * s
            return JointCmd.pd(cosine_interp(self.q_start, Q_CROUCH_SAFE, s), kp, self.kd_stand)
        t2 = t1 - self.CROUCH_S
        if t2 < self.STAND_S:
            self.phase = "stand"
            return JointCmd.pd(cosine_interp(Q_CROUCH_SAFE, Q_DEFAULT, t2 / self.STAND_S),
                               self.kp_stand, self.kd_stand)
        t3 = t2 - self.STAND_S
        if t3 < self.hold_s:
            self.phase = "hold"
            return JointCmd.pd(Q_DEFAULT, self.kp_stand, self.kd_stand)
        return None
