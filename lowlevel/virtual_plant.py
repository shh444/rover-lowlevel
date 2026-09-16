"""시험용 가상 Rover 몸체(MuJoCo). 실기 없이 DDS 경로를 검증할 때 '로봇 쪽'을 맡는다.

실기 규약을 로봇 관점에서 흉내 낸다:

- 16 하드웨어 슬롯. 관절 12개는 ABS2HW 슬롯에 있고, 하드웨어 각도 = 관절각 + MOTOR_OFFSET[slot]
- 모터 드라이버 PD: tau = kp*(q_cmd - q_hw) + kd*(dq_cmd - dq) + tau_ff (슬롯 단위), MJCF ctrlrange 로 제한
- 명령을 받기 전에는 토크 0 (kill_robot 직후 PASSIVE 와 비슷)

tests/fake_dds(가짜 SDK, in-process) 와 tools/virtual_robot_dds(실제 SDK, DDS 통신) 가 함께 쓴다.
제어 코드(backend_*.py, programs.py, safety.py)는 이 모듈을 쓰지 않는다.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np

from .common import (HW_INDEX, HW_OFFSET, JOINT_NAMES, NUM_HW_MOTORS, NUM_JOINTS, Q_CROUCH,
                     Q_DEFAULT, Q_LOWER, Q_UPPER, find_default_xml)


class VirtualPlant:
    def __init__(self, xml=None, physics_dt: float = 0.001, seed: int = 1, start: str = "lying"):
        import mujoco
        self.mujoco = mujoco
        xml = Path(xml) if xml else find_default_xml()
        if xml is None or not Path(xml).exists():
            raise FileNotFoundError(f"dobot_quad.xml 없음 ({xml}). FAKE_DDS_XML 또는 ROVER_VENDOR 를 지정하세요")
        self.xml = Path(xml)
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.m.opt.timestep = float(physics_dt)
        self.physics_dt = float(physics_dt)
        self.d = mujoco.MjData(self.m)
        jid = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES])
        assert np.all(jid >= 0), "MJCF 관절 이름 불일치"
        self.qa, self.va = self.m.jnt_qposadr[jid].copy(), self.m.jnt_dofadr[jid].copy()
        self.aa = np.array([int(np.flatnonzero(self.m.actuator_trnid[:, 0] == j)[0]) for j in jid])
        self.tau_max = self.m.actuator_ctrlrange[self.aa, 1].copy()
        rng = np.random.default_rng(seed)
        if start == "standing":
            self.d.qpos[:3] = [0.0, 0.0, 0.37]
            q0 = Q_DEFAULT.copy()
        else:
            self.d.qpos[:3] = [0.0, 0.0, 0.32]
            q0 = np.clip(np.clip(Q_CROUCH, Q_LOWER, Q_UPPER) + rng.uniform(-0.3, 0.3, NUM_JOINTS), Q_LOWER, Q_UPPER)
        self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qpos[self.qa] = q0
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

        self._cmd_lock = threading.Lock()
        self.cmd = None
        self._last_cmd_t = None
        self._last_tau = np.zeros(NUM_JOINTS)
        self.stats = {"cmds": 0, "cmd_gap_max_ms": 0.0, "kp_max_seen": 0.0, "tau_max_seen": 0.0,
                      "base_z_min": 9.0, "base_z_max": 0.0, "last_cmd_modes": None}

    # ---- 명령 (하드웨어 슬롯 16개) ----
    def set_cmd(self, q, dq, tau, kp, kd, mode) -> None:
        cmd = {k: np.asarray(v, dtype=float).reshape(NUM_HW_MOTORS)
               for k, v in (("q", q), ("dq", dq), ("tau", tau), ("kp", kp), ("kd", kd))}
        modes = [int(x) for x in mode]
        now = time.monotonic()
        with self._cmd_lock:
            if self._last_cmd_t is not None:
                self.stats["cmd_gap_max_ms"] = max(self.stats["cmd_gap_max_ms"], (now - self._last_cmd_t) * 1e3)
            self._last_cmd_t = now
            self.stats["cmds"] += 1
            self.stats["kp_max_seen"] = max(self.stats["kp_max_seen"], float(cmd["kp"].max()))
            self.stats["last_cmd_modes"] = modes
            self.cmd = cmd

    def _torque(self) -> np.ndarray:
        cmd = self.cmd
        if cmd is None:
            return np.zeros(NUM_JOINTS)
        q_hw = self.d.qpos[self.qa] + HW_OFFSET
        dq = self.d.qvel[self.va]
        tau = (cmd["kp"][HW_INDEX] * (cmd["q"][HW_INDEX] - q_hw)
               + cmd["kd"][HW_INDEX] * (cmd["dq"][HW_INDEX] - dq) + cmd["tau"][HW_INDEX])
        return np.clip(tau, -self.tau_max, self.tau_max)

    # ---- 물리 진행과 관측 ----
    def step(self, dt: float) -> None:
        n = max(1, int(round(dt / self.physics_dt)))
        tau = self._last_tau
        for _ in range(n):
            tau = self._torque()
            self.d.ctrl[self.aa] = tau
            self.mujoco.mj_step(self.m, self.d)
        self._last_tau = tau
        base_z = float(self.d.qpos[2])
        self.stats["tau_max_seen"] = max(self.stats["tau_max_seen"], float(np.abs(tau).max()))
        self.stats["base_z_min"] = min(self.stats["base_z_min"], base_z)
        self.stats["base_z_max"] = max(self.stats["base_z_max"], base_z)

    def sample(self) -> dict:
        q_hw, dq_hw, tau_hw = np.zeros(NUM_HW_MOTORS), np.zeros(NUM_HW_MOTORS), np.zeros(NUM_HW_MOTORS)
        mode_hw = np.zeros(NUM_HW_MOTORS, dtype=int)
        q_hw[HW_INDEX] = self.d.qpos[self.qa] + HW_OFFSET
        dq_hw[HW_INDEX] = self.d.qvel[self.va]
        tau_hw[HW_INDEX] = self.d.actuator_force[self.aa]
        mode_hw[HW_INDEX] = 4 if self.cmd is not None else 3
        return {"q_hw": q_hw, "dq_hw": dq_hw, "tau_hw": tau_hw, "mode_hw": mode_hw,
                "quat_wxyz": self.d.sensor("orientation").data.copy(),
                "gyro": self.d.sensor("angular-velocity").data.copy(),
                "acc": self.d.sensor("linear-acceleration").data.copy(),
                "base_z": float(self.d.qpos[2]), "sim_time": float(self.d.time),
                "q_logical": self.d.qpos[self.qa].copy()}

    def report(self) -> dict:
        out = dict(self.stats)
        out["final_q"] = [round(float(x), 4) for x in self.d.qpos[self.qa]]
        out["final_base_z"] = float(self.d.qpos[2])
        out["sim_time"] = float(self.d.time)
        out["xml"] = str(self.xml)
        return out
