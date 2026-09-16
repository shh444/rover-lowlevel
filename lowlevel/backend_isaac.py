"""Isaac Sim 백엔드 (실험적, 아직 실기·실장비 검증 없음): NVIDIA Isaac Sim 5.x 에 URDF 를 불러와
MuJoCo 백엔드와 같은 인터페이스(wait_ready / read / send / close)로 돌린다.

요구 사항
  - x86_64 + NVIDIA RTX GPU (드라이버 570+), Isaac Sim 5.x. 이 저장소의 나머지와 달리 **Python 3.11 환경** 이 필요하다.
      py -3.11 -m venv .venv-isaac
      .venv-isaac\\Scripts\\pip install "isaacsim[all,extscache]==5.*" --extra-index-url https://pypi.nvidia.com
      .venv-isaac\\Scripts\\pip install numpy
    첫 실행 때 EULA 동의 프롬프트가 뜬다 (또는 환경변수 OMNI_KIT_ACCEPT_EULA=YES).
  - URDF: dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf (vendor/ 에 두면 자동 인식)

동작
  - SimulationApp(headless) → World(physics_dt) → URDF import(드라이브 없음, 고정/자유 베이스 선택) → SingleArticulation.
  - 관절 순서는 이름(JOINT_NAMES)으로 맞춘다. 드라이브 게인은 0 으로 두고, SDK 공식
    tau = kp(q_des-q) + kd(dq_des-dq) + tau_ff 를 substep 마다 계산해 set_joint_efforts 로 넣는다 (URDF effort 한계로 자름).
  - IMU: 몸통 프림의 자세(wxyz)·각속도에서 만들고, 가속도는 중력 방향 근사(R^T·[0,0,9.81]) 다.
  - Isaac Sim 의 API 이름은 버전마다 조금씩 바뀐다. 아래 import 는 5.0/5.1 기준이며, 실패하면 오류 메시지에 버전을 적어 둔다.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

from .common import (JOINT_NAMES, NUM_JOINTS, Q_CROUCH, Q_DEFAULT, Q_LOWER, Q_UPPER, TAU_MAX,
                     JointCmd, State)

URDF_REL = Path("dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf")


def find_default_urdf() -> Path | None:
    here = Path(__file__).resolve().parents[1]
    candidates = [here / "vendor", Path("/work/vendor"), Path.home() / "rover-mujoco-poc/vendor"]
    if os.environ.get("ROVER_VENDOR"):
        candidates.insert(0, Path(os.environ["ROVER_VENDOR"]))
    for base in candidates:
        if (base / URDF_REL).exists():
            return base / URDF_REL
    return None


def _quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class IsaacBackend:
    name = "isaac"

    def __init__(self, urdf_path=None, dt: float = 0.005, physics_dt: float = 0.001, start: str = "lying",
                 fixed_base: bool = False, headless: bool = True, init_q=None, init_base_z=None,
                 render_every_tick: int = 4):
        urdf = Path(urdf_path) if urdf_path else find_default_urdf()
        if urdf is None or not urdf.exists():
            raise FileNotFoundError(f"URDF 없음: {urdf} (--urdf 또는 ROVER_VENDOR 로 지정)")
        self.urdf = urdf
        self.dt, self.physics_dt = float(dt), float(physics_dt)
        self.substeps = max(1, int(round(dt / physics_dt)))
        if not math.isclose(self.substeps * physics_dt, dt, rel_tol=1e-6):
            raise ValueError("dt 는 physics_dt 의 정수배여야 합니다")
        self.headless = bool(headless)
        self.render_every_tick = max(1, int(render_every_tick))
        self.fixed_base = bool(fixed_base)

        try:
            from isaacsim import SimulationApp
        except ImportError as exc:
            raise ImportError("isaacsim 패키지가 없습니다. Python 3.11 환경에 Isaac Sim 5.x 를 설치하세요 "
                              "(lowlevel/backend_isaac.py 상단 참고)") from exc
        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        self.app = SimulationApp({"headless": self.headless})
        try:
            self._build_world(start, init_q, init_base_z)
        except Exception:
            self.app.close()
            raise
        self.tick = 0
        self.last_tau = np.zeros(NUM_JOINTS)

    # ---- 장면 구성 ----
    def _build_world(self, start, init_q, init_base_z):
        import omni.kit.commands
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import _urdf

        self.world = World(physics_dt=self.physics_dt, rendering_dt=self.dt, stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        ok, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        cfg.merge_fixed_joints = False
        cfg.fix_base = self.fixed_base
        cfg.make_default_prim = True
        cfg.create_physics_scene = False
        cfg.self_collision = False
        cfg.distance_scale = 1.0
        cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_NONE
        cfg.default_drive_strength = 0.0
        cfg.default_position_drive_damping = 0.0
        ok, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(self.urdf),
                                                  import_config=cfg, get_articulation_root=True)
        if not ok:
            raise RuntimeError(f"URDF import 실패: {self.urdf}")
        self.robot = SingleArticulation(prim_path=prim_path, name="rover")
        self.world.scene.add(self.robot)
        self.world.reset()

        names = list(self.robot.dof_names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if missing:
            raise RuntimeError(f"URDF 관절 이름 불일치: {missing} (있는 이름: {names})")
        self.idx = np.array([names.index(n) for n in JOINT_NAMES])     # 논리 순서 → Isaac dof 순서
        n = len(names)
        ctrl = self.robot.get_articulation_controller()
        ctrl.set_gains(kps=np.zeros(n), kds=np.zeros(n))                 # 드라이브 끔: 우리가 토크를 넣는다
        try:
            ctrl.set_effort_modes("force")
        except Exception:
            pass
        self.tau_max = TAU_MAX.copy()

        if init_q is not None:
            q0 = np.clip(np.asarray(init_q, dtype=float).reshape(NUM_JOINTS), Q_LOWER, Q_UPPER)
        elif start == "standing" or self.fixed_base:
            q0 = Q_DEFAULT.copy()
        else:
            q0 = np.clip(Q_CROUCH, Q_LOWER, Q_UPPER)
        z = init_base_z if init_base_z is not None else (0.85 if self.fixed_base else (0.37 if start == "standing" else 0.32))
        full = np.zeros(n)
        full[self.idx] = q0
        self.robot.set_world_pose(position=np.array([0.0, 0.0, z]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
        self.robot.set_joint_positions(full)
        self.robot.set_joint_velocities(np.zeros(n))
        self.world.step(render=not self.headless)

    # ---- 공통 인터페이스 ----
    def wait_ready(self) -> State:
        return self.read()

    def _joint_state(self):
        q = np.asarray(self.robot.get_joint_positions(), dtype=float)[self.idx]
        dq = np.asarray(self.robot.get_joint_velocities(), dtype=float)[self.idx]
        return q, dq

    def base_pos(self) -> np.ndarray:
        pos, _ = self.robot.get_world_pose()
        return np.asarray(pos, dtype=float)

    def read(self) -> State:
        q, dq = self._joint_state()
        pos, quat = self.robot.get_world_pose()
        quat = np.asarray(quat, dtype=float)                       # Isaac: wxyz
        rot = _quat_to_rot(quat)
        omega_w = np.asarray(self.robot.get_angular_velocity(), dtype=float)
        gyro = rot.T @ omega_w
        acc = rot.T @ np.array([0.0, 0.0, 9.81])                 # 근사: 중력 반작용만 (정지 시 실기와 같은 방향)
        t = self.tick * self.dt
        return State(t=t, q=q, dq=dq, quat_wxyz=quat, gyro=gyro, acc=acc, tau_est=self.last_tau.copy(), age=0.0,
                     extra={"base_pos": np.asarray(pos, dtype=float), "sim_time": t})

    def send(self, cmd: JointCmd) -> None:
        n = len(self.robot.dof_names)
        tau = self.last_tau
        for k in range(self.substeps):
            q, dq = self._joint_state()
            tau = np.clip(cmd.torque(q, dq), -self.tau_max, self.tau_max)
            full = np.zeros(n)
            full[self.idx] = tau
            self.robot.set_joint_efforts(full)
            render = (not self.headless) and k == self.substeps - 1 and (self.tick % self.render_every_tick == 0)
            self.world.step(render=render)
        self.last_tau = tau
        self.tick += 1
        if not self.headless and not self.app.is_running():
            raise KeyboardInterrupt

    def snapshot(self, path) -> bool:
        return False

    def close(self) -> None:
        try:
            self.app.close()
        except Exception:
            pass
