#!/usr/bin/env python3
"""임의의 MJCF 로봇을 MuJoCo 에서 돌리며 플랫폼 기록 형식(meta.json + trace.csv)으로 남긴다 — 휴머노이드 등 다른 로봇용.

관절 목록은 모델의 액추에이터가 붙은 힌지/슬라이드 관절이다(이름은 모델 그대로). 제어는 Rover 와 같은 PD:
tau = kp(q_des−q) + kd(dq_des−dq), 액추에이터 ctrlrange(없으면 --tau-max)로 자른다.
기록은 datalab 에서 Rover·실기 기록과 같은 화면에 나타나고(robot 필드로 구분), 관절 이름이 겹치는 기록끼리 비교할 수 있다.

    python tools/mujoco_record.py --xml ../thor-backup-20260916/extracted/atom-max-lab/atom_upper.xml --robot atom_upper \
        --program sine --joints left_shoulder_pitch,right_shoulder_pitch --amp 0.3 --seconds 6 --viewer
    python tools/mujoco_record.py --xml vendor/dobot_rover_simulation/dobot_sim2real/resources/mujoco/dobot_quad.xml --robot rover --program hold
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
import types
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if sys.platform.startswith("linux"):
    os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco                                                     # noqa: E402

from lowlevel.common import State                                 # noqa: E402
from lowlevel.dataset import update_meta, utc_now, write_meta     # noqa: E402
from lowlevel.runtime import Interrupts, Pacer, TraceLog          # noqa: E402


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class MjcfRobot:
    """액추에이터가 붙은 관절만 다루는 범용 MuJoCo 로봇."""

    def __init__(self, xml: Path, physics_dt: float, keyframe: int | None, tau_max_default: float):
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.m.opt.timestep = physics_dt
        self.d = mujoco.MjData(self.m)
        if keyframe is not None and self.m.nkey > keyframe:
            mujoco.mj_resetDataKeyframe(self.m, self.d, keyframe)
        acts, jids = [], []
        for a in range(self.m.nu):
            if self.m.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT:
                continue
            jid = int(self.m.actuator_trnid[a, 0])
            if self.m.jnt_type[jid] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                acts.append(a)
                jids.append(jid)
        if not acts:
            raise RuntimeError("액추에이터가 붙은 힌지/슬라이드 관절이 없습니다")
        self.aa, self.jids = np.array(acts), np.array(jids)
        self.names = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}" for j in jids]
        self.qa, self.va = self.m.jnt_qposadr[self.jids], self.m.jnt_dofadr[self.jids]
        limited = self.m.actuator_ctrllimited[self.aa].astype(bool)
        self.tau_max = np.where(limited, np.abs(self.m.actuator_ctrlrange[self.aa]).max(axis=1), tau_max_default)
        self.lower = np.where(self.m.jnt_limited[self.jids].astype(bool), self.m.jnt_range[self.jids, 0], -np.inf)
        self.upper = np.where(self.m.jnt_limited[self.jids].astype(bool), self.m.jnt_range[self.jids, 1], np.inf)
        free = np.flatnonzero(self.m.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        self.root_body = int(self.m.jnt_bodyid[free[0]]) if len(free) else (1 if self.m.nbody > 1 else 0)
        self.has_free = len(free) > 0
        mujoco.mj_forward(self.m, self.d)
        self.last_tau = np.zeros(len(self.names))

    def state(self, t: float) -> State:
        quat = self.d.xquat[self.root_body].copy()
        rot = quat_to_rot(quat)
        ang_w = self.d.cvel[self.root_body][:3].copy()           # 세계 좌표계 각속도
        return State(t=t, q=self.d.qpos[self.qa].copy(), dq=self.d.qvel[self.va].copy(), quat_wxyz=quat,
                     gyro=rot.T @ ang_w, acc=rot.T @ np.array([0.0, 0.0, 9.81]), tau_est=self.last_tau.copy(),
                     age=0.0, extra={"base_pos": self.d.xpos[self.root_body].copy(), "sim_time": float(self.d.time)})

    def step(self, q_des, kp, kd, substeps: int):
        tau = self.last_tau
        for _ in range(substeps):
            q, dq = self.d.qpos[self.qa], self.d.qvel[self.va]
            tau = np.clip(kp * (q_des - q) - kd * dq, -self.tau_max, self.tau_max)
            self.d.ctrl[self.aa] = tau
            mujoco.mj_step(self.m, self.d)
        self.last_tau = tau


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xml", type=Path, required=True)
    ap.add_argument("--robot", default=None, help="기록의 로봇 이름 (기본: xml 파일 이름)")
    ap.add_argument("--program", choices=("hold", "sine", "damp"), default="sine")
    ap.add_argument("--joints", default="all", help="sine 대상: all | 이름 또는 부분 문자열을 쉼표로")
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--kd-damp", type=float, default=2.0)
    ap.add_argument("--seconds", type=float, default=5.0, help="본동작 시간 (앞의 --settle 초 댐핑은 별도)")
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--physics-dt", type=float, default=0.001)
    ap.add_argument("--tau-max", type=float, default=50.0, help="ctrlrange 가 없는 액추에이터의 토크 한계")
    ap.add_argument("--keyframe", type=int, default=0, help="초기 자세 keyframe 번호 (없으면 무시)")
    ap.add_argument("--slew", type=float, default=2.0, help="목표각 변화율 한계 rad/s")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tag", action="append", default=[])
    ap.add_argument("--note", default="")
    a = ap.parse_args()
    if a.viewer:
        a.realtime = True

    robot_name = a.robot or a.xml.stem
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = a.out or (ROOT / "runs" / f"{stamp}-sim-{a.program}-{robot_name}")
    out.mkdir(parents=True, exist_ok=True)
    robot = MjcfRobot(a.xml.resolve(), a.physics_dt, a.keyframe, a.tau_max)
    n = len(robot.names)
    substeps = max(1, int(round(a.dt / a.physics_dt)))
    if a.joints.strip().lower() == "all":
        mask = np.ones(n, dtype=bool)
    else:
        keys = [k.strip() for k in a.joints.split(",") if k.strip()]
        mask = np.array([any(k == nm or k in nm for k in keys) for nm in robot.names])
        if not mask.any():
            print(f"--joints 와 맞는 관절이 없습니다. 관절 목록: {robot.names}")
            return 1
    print(f"[mjcf] {a.xml.name}: 관절 {n}개 {robot.names}")
    print(f"[mjcf] 대상 {int(mask.sum())}개, 토크 한계 {robot.tau_max.round(1).tolist()}, 기록 {out}")
    write_meta(out, source="mujoco", program=a.program, dt=a.dt, tags=a.tag + ["mjcf"], note=a.note,
               robot=robot_name, joints=robot.names,
               params={"duration": a.seconds, "amp": a.amp, "freq": a.freq, "joints": a.joints, "kp": a.kp, "kd": a.kd,
                       "xml": str(a.xml)})

    viewer = None
    if a.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(robot.m, robot.d)
    log = TraceLog(out / "trace.csv", joints=robot.names)
    interrupts = Interrupts()
    interrupts.install()
    pacer = Pacer(a.dt, enabled=a.realtime)
    q_ref = robot.d.qpos[robot.qa].copy()
    q0 = None
    reason, tick = "completed", 0
    total = a.settle + a.seconds
    try:
        while True:
            t = tick * a.dt
            if t >= total:
                break
            pacer.wait(tick)
            st = robot.state(t)
            if t < a.settle:
                phase, q_des, kp, kd = "settle", st.q.copy(), 0.0, a.kd_damp
                q_ref = st.q.copy()
            else:
                if q0 is None:
                    q0 = st.q.copy()
                tt = t - a.settle
                if a.program == "sine":
                    env = min(1.0, tt)
                    q_des = q0 + mask * a.amp * env * math.sin(2 * math.pi * a.freq * tt)
                    phase = "ramp" if env < 1.0 else "sine"
                elif a.program == "hold":
                    q_des, phase = q0.copy(), "hold"
                else:
                    q_des, phase = st.q.copy(), "damp"
                kp, kd = (a.kp, a.kd) if a.program != "damp" else (0.0, a.kd_damp)
                q_des = np.clip(q_des, robot.lower, robot.upper)
                step = a.slew * a.dt
                q_des = q_ref + np.clip(q_des - q_ref, -step, step)
                q_ref = q_des
            robot.step(q_des, kp, kd, substeps)
            cmd = types.SimpleNamespace(q=q_des, kp=np.full(n, kp), kd=np.full(n, kd))
            log.row(t, phase, st, cmd)
            if viewer is not None:
                if not viewer.is_running():
                    raise KeyboardInterrupt
                if tick % 4 == 0:
                    viewer.sync()
            if tick % int(round(1.0 / a.dt)) == 0:
                j = int(np.flatnonzero(mask)[0])
                print(f"t={t:5.2f}s {phase:<6} {robot.names[j]} q={st.q[j]:+.3f} q_des={q_des[j]:+.3f} "
                      f"|tau|max={abs(st.tau_est).max():.2f}")
            tick += 1
    except KeyboardInterrupt:
        reason = "keyboard_interrupt"
    interrupts.during_exit = True
    for k in range(int(round(1.0 / a.dt))):                 # 항상 댐핑 1초로 마무리
        pacer.wait(tick)
        st = robot.state(tick * a.dt)
        robot.step(st.q, 0.0, a.kd_damp, substeps)
        log.row(tick * a.dt, "exit_damp", st, types.SimpleNamespace(q=st.q, kp=np.zeros(n), kd=np.full(n, a.kd_damp)))
        tick += 1
    log.close()
    if viewer is not None:
        viewer.close()
    update_meta(out, status=reason, ended_at=utc_now(), ticks=tick, program_seconds=tick * a.dt,
                deadline_misses=pacer.misses, max_late_ms=pacer.max_late * 1e3)
    print(f"[mjcf] 종료 {reason}, {tick}틱, 기한 초과 {pacer.misses}회 → {out}")
    return 0 if reason == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
