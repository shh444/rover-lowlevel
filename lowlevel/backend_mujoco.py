"""MuJoCo 백엔드: dobot_sim2real 의 dobot_quad.xml 로 실기와 같은 인터페이스를 흉내 낸다.

실기 모터 드라이버가 하는 PD(tau = kp(q_des-q) + kd(dq_des-dq) + tau_ff)를 물리 substep(기본 1ms)마다
다시 계산해 토크 액추에이터(mt00~mt11)에 넣는다. 토크는 MJCF ctrlrange(23/23/55 N·m)로 자른다.
IMU 는 MJCF 의 imu 사이트 센서(orientation, angular-velocity, linear-acceleration)에서 읽는다.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")   # SSH/헤드리스 렌더링. import 전에 설정해야 한다.
import mujoco
import numpy as np

from .common import (JOINT_NAMES, NUM_JOINTS, Q_CROUCH, Q_DEFAULT, Q_LOWER, Q_UPPER,
                     JointCmd, State, find_default_xml)


class MujocoBackend:
    name = "mujoco"

    def __init__(self, xml_path=None, dt: float = 0.005, physics_dt: float = 0.001,
                 start: str = "lying", seed: int = 0, frames_dir=None, render_every: int = 0,
                 size=(960, 540)):
        xml = Path(xml_path) if xml_path else find_default_xml()
        if xml is None or not xml.exists():
            raise FileNotFoundError(f"MJCF 없음: {xml} (--xml 또는 ROVER_VENDOR 로 지정. "
                                    "dobot_rover_simulation 저장소의 dobot_sim2real/resources/mujoco/dobot_quad.xml)")
        self.xml = xml
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.substeps = max(1, int(round(dt / physics_dt)))
        if not math.isclose(self.substeps * physics_dt, dt, rel_tol=1e-6):
            raise ValueError("dt 는 physics_dt 의 정수배여야 합니다")
        self.m.opt.timestep = physics_dt
        self.d = mujoco.MjData(self.m)

        jid = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES])
        if np.any(jid < 0):
            missing = [n for n, j in zip(JOINT_NAMES, jid) if j < 0]
            raise RuntimeError(f"MJCF 에 관절이 없습니다: {missing}")
        self.qa = self.m.jnt_qposadr[jid].copy()
        self.va = self.m.jnt_dofadr[jid].copy()
        aa = []
        for j in jid:
            cand = np.flatnonzero(self.m.actuator_trnid[:, 0] == j)
            if len(cand) != 1:
                raise RuntimeError(f"관절 id {j} 에 연결된 액추에이터가 정확히 1개가 아님")
            aa.append(int(cand[0]))
        self.aa = np.array(aa)
        self.tau_max = self.m.actuator_ctrlrange[self.aa, 1].copy()
        for s in ("orientation", "angular-velocity", "linear-acceleration"):
            self.d.sensor(s)   # 없으면 KeyError

        rng = np.random.default_rng(seed)
        if start == "standing":
            self.d.qpos[:3] = [0.0, 0.0, 0.37]
            q0 = Q_DEFAULT.copy()
        else:   # lying: sim2real init_mujoco 와 같이 엎드림 자세 + 잡음, 공중에서 낙하
            self.d.qpos[:3] = [0.0, 0.0, 0.32]
            q0 = np.clip(Q_CROUCH, Q_LOWER, Q_UPPER) + rng.uniform(-0.3, 0.3, NUM_JOINTS)
            q0 = np.clip(q0, Q_LOWER, Q_UPPER)
        self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qpos[self.qa] = q0
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

        self.tick = 0
        self.last_tau = np.zeros(NUM_JOINTS)
        self.frames_dir = Path(frames_dir) if frames_dir else None
        self.render_every = int(render_every)
        self.size = size
        if self.frames_dir:
            self.frames_dir.mkdir(parents=True, exist_ok=True)
        self._renderer = None
        self._cam = None

    # ---- 공통 인터페이스 ----
    def wait_ready(self) -> State:
        return self.read()

    def read(self) -> State:
        d = self.d
        return State(t=float(d.time), q=d.qpos[self.qa].copy(), dq=d.qvel[self.va].copy(),
                     quat_wxyz=d.sensor("orientation").data.copy(),
                     gyro=d.sensor("angular-velocity").data.copy(),
                     acc=d.sensor("linear-acceleration").data.copy(),
                     tau_est=self.last_tau.copy(), age=0.0,
                     extra={"base_pos": d.qpos[:3].copy(), "sim_time": float(d.time)})

    def send(self, cmd: JointCmd) -> None:
        d = self.d
        tau = self.last_tau
        for _ in range(self.substeps):
            tau = np.clip(cmd.torque(d.qpos[self.qa], d.qvel[self.va]), -self.tau_max, self.tau_max)
            d.ctrl[self.aa] = tau
            mujoco.mj_step(self.m, d)
        self.last_tau = tau
        self.tick += 1
        if self.render_every and self.tick % self.render_every == 0:
            self.snapshot(self.frames_dir / f"{self.tick:06d}.jpg")

    def snapshot(self, path) -> bool:
        try:
            width, height = self.size
            if self._renderer is None:
                self.m.vis.global_.offwidth = max(width, int(self.m.vis.global_.offwidth))
                self.m.vis.global_.offheight = max(height, int(self.m.vis.global_.offheight))
                self._renderer = mujoco.Renderer(self.m, height=height, width=width)
                self._cam = mujoco.MjvCamera()
                self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                self._cam.distance, self._cam.azimuth, self._cam.elevation = 1.8, 135, -20
            self._cam.lookat[:] = self.d.qpos[:3] + np.array([0.0, 0.0, -0.05])
            self._renderer.update_scene(self.d, camera=self._cam)
            pixels = self._renderer.render()
            from PIL import Image
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(pixels).save(str(path), quality=88)
            return True
        except Exception as exc:   # 렌더링 실패는 제어를 막지 않는다
            print(f"[render] 실패: {exc!r}")
            return False

    def close(self) -> None:
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:   # EGL 컨텍스트 해제 시 무해한 EGLError 가 날 수 있다
                pass
            self._renderer = None
