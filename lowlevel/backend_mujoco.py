"""MuJoCo 백엔드: dobot_sim2real 의 dobot_quad.xml 로 실기와 같은 인터페이스를 흉내 낸다.

실기 모터 드라이버가 하는 PD(tau = kp(q_des-q) + kd(dq_des-dq) + tau_ff)를 물리 substep(기본 1ms)마다
다시 계산해 토크 액추에이터(mt00~mt11)에 넣는다. 토크는 MJCF ctrlrange(23/23/55 N·m)로 자른다.
IMU 는 MJCF 의 imu 사이트 센서(orientation, angular-velocity, linear-acceleration)에서 읽는다.
"""
from __future__ import annotations

import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")   # SSH/헤드리스 렌더링. import 전에 설정해야 한다.
import mujoco
import numpy as np

from .common import (JOINT_NAMES, NUM_JOINTS, Q_CROUCH, Q_DEFAULT, Q_LOWER, Q_UPPER,
                     JointCmd, State, find_default_xml)


def make_fixed_base_xml(xml: Path, height: float = 0.85) -> Path:
    """몸통(link_trunk)의 free joint 를 없애고 공중에 고정한 MJCF 를 임시 폴더에 만든다.
    '로봇을 지지한 상태에서 다리만 시험' 하는 상황의 시뮬 대응 (control_lab 의 fixed_model 과 같은 방식)."""
    tree = ET.parse(xml)
    root = tree.getroot()
    for inc in root.iter("include"):
        inc.set("file", str((xml.parent / inc.get("file")).resolve()))
    compiler = root.find("compiler")
    if compiler is not None and compiler.get("meshdir"):
        compiler.set("meshdir", str((xml.parent / compiler.get("meshdir")).resolve()))
    trunk = root.find("worldbody/body[@name='link_trunk']")
    free = trunk.find("joint[@type='free']") if trunk is not None else None
    if trunk is None or free is None:
        raise RuntimeError("link_trunk 의 free joint 를 찾지 못했습니다")
    trunk.remove(free)
    trunk.set("pos", f"0 0 {height}")
    for key in list(root.findall("keyframe")):
        root.remove(key)
    out = Path(tempfile.mkdtemp(prefix="rover_fixed_")) / "fixed_body_model.xml"
    tree.write(out, encoding="unicode")
    return out


class MujocoBackend:
    name = "mujoco"

    def __init__(self, xml_path=None, dt: float = 0.005, physics_dt: float = 0.001,
                 start: str = "lying", seed: int = 0, frames_dir=None, render_every: int = 0,
                 size=(960, 540), fixed_base: bool = False):
        xml = Path(xml_path) if xml_path else find_default_xml()
        if xml is None or not xml.exists():
            raise FileNotFoundError(f"MJCF 없음: {xml} (--xml 또는 ROVER_VENDOR 로 지정. "
                                    "dobot_rover_simulation 저장소의 dobot_sim2real/resources/mujoco/dobot_quad.xml)")
        if fixed_base:
            xml = make_fixed_base_xml(xml)
        self.xml = xml
        self.fixed_base = bool(fixed_base)
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

        self.has_free = bool(np.any(self.m.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
        self.trunk_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "link_trunk")
        rng = np.random.default_rng(seed)
        if start == "standing" or not self.has_free:   # 몸통 고정이면 다리를 늘어뜨린 기본 자세에서 시작
            q0 = Q_DEFAULT.copy()
            if self.has_free:
                self.d.qpos[:3] = [0.0, 0.0, 0.37]
        else:   # lying: sim2real init_mujoco 와 같이 엎드림 자세 + 잡음, 공중에서 낙하
            self.d.qpos[:3] = [0.0, 0.0, 0.32]
            q0 = np.clip(Q_CROUCH, Q_LOWER, Q_UPPER) + rng.uniform(-0.3, 0.3, NUM_JOINTS)
            q0 = np.clip(q0, Q_LOWER, Q_UPPER)
        if self.has_free:
            self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.qpos[self.qa] = q0
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        self._scratch = mujoco.MjData(self.m)          # nominal_gravity 용 (물리 상태를 건드리지 않는다)

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

    def base_pos(self) -> np.ndarray:
        return self.d.qpos[:3].copy() if self.has_free else self.d.xpos[self.trunk_id].copy()

    def read(self) -> State:
        d = self.d
        return State(t=float(d.time), q=d.qpos[self.qa].copy(), dq=d.qvel[self.va].copy(),
                     quat_wxyz=d.sensor("orientation").data.copy(),
                     gyro=d.sensor("angular-velocity").data.copy(),
                     acc=d.sensor("linear-acceleration").data.copy(),
                     tau_est=self.last_tau.copy(), age=0.0,
                     extra={"base_pos": self.base_pos(), "sim_time": float(d.time)})

    def nominal_gravity(self, q: np.ndarray, quat_wxyz=None) -> np.ndarray:
        """명목 모델의 중력 토크 g(q) (12,). 속도 0 이므로 코리올리 항은 없다. tau_ff 로 쓰면 중력을 상쇄한다.
        몸통이 자유로우면 quat_wxyz(IMU)로 몸통 기울기를 반영한다. 접촉력은 포함하지 않는다(공중 지지 상황용)."""
        s = self._scratch
        s.qpos[:] = self.d.qpos
        if self.has_free and quat_wxyz is not None:
            s.qpos[3:7] = quat_wxyz
        s.qpos[self.qa] = q
        s.qvel[:] = 0.0
        s.qacc[:] = 0.0
        mujoco.mj_forward(self.m, s)
        return s.qfrc_bias[self.va].copy()

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
            self._cam.lookat[:] = self.base_pos() + np.array([0.0, 0.0, -0.05])
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
