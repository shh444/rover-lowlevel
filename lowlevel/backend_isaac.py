"""Isaac Sim 백엔드 (Isaac Sim 6.1 API): NVIDIA Isaac Sim 에 Rover URDF 를 불러와
MuJoCo 백엔드와 같은 인터페이스(wait_ready / read / send / close)로 돌린다.

요구 사항
  - x86_64 + NVIDIA RTX GPU (드라이버 570 이상), Isaac Sim 6.1 pip 패키지 (Python 3.12 지원).
    Windows 에서는 경로 길이 제한(MAX_PATH 260) 때문에 **짧은 경로** 에 가상환경을 만들어야 한다:
      py -3.12 -m venv C:\\isaac-venv
      C:\\isaac-venv\\Scripts\\pip install "isaacsim[all,extscache]==6.1.0.0" --extra-index-url https://pypi.nvidia.com
    첫 실행에는 NVIDIA Omniverse Kit EULA 동의가 필요하다. 이 코드는 **대신 동의하지 않는다**: 사용자가 직접
    `python -c "import isaacsim"` 으로 EULA 를 읽고 Yes 를 입력하거나(패키지 안에 EULA_ACCEPTED 파일이 남는다),
    동의한 뒤 환경변수 OMNI_KIT_ACCEPT_EULA=YES 를 설정해야 한다.
  - URDF: dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf (vendor/ 에 두면 자동 인식).
    관절 이름이 MuJoCo 모델·JOINT_NAMES 와 같다 (joint_front_left_abad …).

동작 (6.1 API)
  - SimulationApp → SimulationManager.setup_simulation(physics_dt, cpu) → URDFImporter(드라이브 없음, 고정/자유 베이스)
    → 결과 USD 를 스테이지에 참조 → isaacsim.core.experimental.prims.Articulation.
  - URDF→USD 변환 결과는 .isaac_cache/ 에 두고 다음 실행부터 재사용한다.
  - 물리는 타임라인을 재생하지 않고 SimulationManager.step() 으로 substep(1 ms)마다 직접 진행한다. substep 마다 SDK 공식
    tau = kp(q_des−q) + kd(dq_des−dq) + tau_ff 를 계산해 set_dof_efforts 로 넣는다 (URDF effort 한계로 자름).
    수동 초기화가 안 되는 환경이면 타임라인 재생 방식(틱마다 토크 1회 갱신, app.update 로 진행)으로 자동 전환한다.
  - 물리 엔진은 physx(기본) 또는 newton(MuJoCo-Warp 솔버) 을 고를 수 있다 (engine=).
  - IMU: 몸통 프림의 자세(wxyz)·각속도(몸체 좌표계로 변환). 가속도는 정지 상태 근사(중력 반작용)다.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from .common import (JOINT_NAMES, NUM_JOINTS, Q_CROUCH, Q_DEFAULT, Q_LOWER, Q_UPPER, TAU_MAX,
                     JointCmd, State)

ROOT = Path(__file__).resolve().parents[1]
URDF_REL = Path("dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf")
ROBOT_PRIM = "/World/rover"

EULA_MSG = (
    "Isaac Sim(Omniverse Kit) 첫 실행에는 NVIDIA EULA 동의가 필요합니다. 이 프로그램은 대신 동의하지 않습니다.\n"
    "  1) 터미널에서  <isaac venv>\\Scripts\\python.exe -c \"import isaacsim\"  를 실행해 EULA 를 읽고 Yes 를 입력하거나\n"
    "  2) 동의한다면 환경변수 OMNI_KIT_ACCEPT_EULA=YES 를 설정한 뒤 다시 실행하세요."
)


def find_default_urdf() -> Path | None:
    candidates = [ROOT / "vendor", Path("/work/vendor"), Path.home() / "rover-mujoco-poc/vendor"]
    if os.environ.get("ROVER_VENDOR"):
        candidates.insert(0, Path(os.environ["ROVER_VENDOR"]))
    for base in candidates:
        if (base / URDF_REL).exists():
            return base / URDF_REL
    return None


def eula_accepted() -> bool:
    """NVIDIA Omniverse Kit EULA 동의 여부: 환경변수 OMNI_KIT_ACCEPT_EULA 또는 패키지 안의 EULA_ACCEPTED 파일
    (isaacsim/kit/kit_app.py 가 확인하는 두 가지와 같다). 패키지를 import 하지 않고 위치만 찾는다."""
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "N").lower() in ("y", "yes", "1"):
        return True
    try:
        import importlib.util
        spec = importlib.util.find_spec("isaacsim")
    except Exception:
        spec = None
    if spec is not None and spec.origin:
        return (Path(spec.origin).parent / "kit" / "EULA_ACCEPTED").is_file()
    return False


def _quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class IsaacBackend:
    name = "isaac"

    def __init__(self, urdf_path=None, dt: float = 0.005, physics_dt: float = 0.001, start: str = "lying",
                 fixed_base: bool = False, headless: bool = True, init_q=None, init_base_z=None,
                 render_every_tick: int = 4, engine: str = "physx", device: str = "cpu", usd_cache=None,
                 physics_variant: str | None = None, render: bool = False, fabric: bool = True,
                 render_fps: float = 30.0):
        urdf = Path(urdf_path) if urdf_path else find_default_urdf()
        if urdf is None or not urdf.exists():
            raise FileNotFoundError(f"URDF 없음: {urdf} (--urdf 또는 ROVER_VENDOR 로 지정)")
        self.urdf = urdf.resolve()
        self.dt, self.physics_dt = float(dt), float(physics_dt)
        self.substeps = max(1, int(round(dt / physics_dt)))
        if not math.isclose(self.substeps * physics_dt, dt, rel_tol=1e-6):
            raise ValueError("dt 는 physics_dt 의 정수배여야 합니다")
        self.headless = bool(headless)
        self.render_every_tick = max(1, int(render_every_tick))
        self.render_enabled = (not self.headless) or bool(render)     # 창이 있거나, 헤드리스여도 스냅샷용 렌더링을 켠 경우
        self.fixed_base = bool(fixed_base)
        self.engine = engine
        self.device = device
        self.usd_cache = Path(usd_cache) if usd_cache else ROOT / ".isaac_cache"
        self.physics_variant = physics_variant
        self.fabric = bool(fabric)
        self.render_period = 1.0 / max(1.0, float(render_fps))
        self._last_render = 0.0
        self.mode = "manual"
        self.tick = 0
        self.last_tau = np.zeros(NUM_JOINTS)
        self.tau_max = TAU_MAX.copy()

        try:
            import importlib.util
            if importlib.util.find_spec("isaacsim") is None:
                raise ImportError
        except ImportError as exc:
            raise ImportError("isaacsim 패키지가 없습니다. Isaac Sim 6.1 이 설치된 python 으로 실행하세요 "
                              "(lowlevel/backend_isaac.py 상단 참고)") from exc
        if not eula_accepted():
            raise RuntimeError(EULA_MSG)

        from isaacsim import SimulationApp                         # 여기서 Kit 커널이 뜬다 (수십 초)
        cfg = {"headless": self.headless, "renderer": "RaytracedLighting",
               # fast_shutdown=True(기본) 는 프로세스를 즉시 끝내므로 close() 가 진행 중 예외를 먼저 출력한다.
               # 정상 종료(느리고 종료 시 segfault 가 날 수 있음)를 원하면 ROVER_ISAAC_FAST_SHUTDOWN=0
               "fast_shutdown": os.environ.get("ROVER_ISAAC_FAST_SHUTDOWN", "1") == "1"}
        if not self.render_enabled:
            cfg["disable_viewport_updates"] = True
        t0 = time.perf_counter()
        self.app = SimulationApp(cfg)
        print(f"[isaac] SimulationApp 시작 {time.perf_counter() - t0:.1f}s (headless={self.headless})")
        try:
            self._build_world(start, init_q, init_base_z)
        except Exception:
            import traceback
            traceback.print_exc()
            sys.stdout.flush(); sys.stderr.flush()
            self._close_now()
            raise

    # ---- 장면 구성 ----
    def _import_urdf(self) -> Path:
        """URDF → USD 변환 (캐시). 드라이브는 끄고(강성·감쇠 0, 목표 없음) 토크만 넣을 수 있게 한다."""
        import isaacsim.core.experimental.utils.app as app_utils
        key = f"{self.urdf.stem}-{'fixed' if self.fixed_base else 'free'}"
        cache = self.usd_cache / key
        index = cache / "index.json"
        if index.is_file():
            try:
                info = json.loads(index.read_text(encoding="utf-8"))
                usd = Path(info["usd"])
                if usd.is_file() and info.get("urdf_mtime") == self.urdf.stat().st_mtime and info.get("urdf") == str(self.urdf):
                    print(f"[isaac] URDF 변환 캐시 사용: {usd}")
                    return usd
            except Exception:
                pass
        app_utils.enable_extension("isaacsim.asset.importer.urdf")
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        cache.mkdir(parents=True, exist_ok=True)
        cfg = URDFImporterConfig(
            urdf_path=str(self.urdf), usd_path=str(cache), merge_fixed_joints=False, fix_base=self.fixed_base,
            collision_from_visuals=False, allow_self_collision=False, robot_type="Quadruped",
            joint_drive_type="force", joint_target_type="none",
            override_joint_stiffness=0.0, override_joint_damping=0.0,
            run_asset_transformer=True, run_multi_physics_conversion=True)
        t0 = time.perf_counter()
        out = URDFImporter(cfg).import_urdf()
        usd = Path(out).resolve()
        if not usd.is_file():
            raise RuntimeError(f"URDF import 결과 파일이 없습니다: {out}")
        index.write_text(json.dumps({"usd": str(usd), "urdf": str(self.urdf), "urdf_mtime": self.urdf.stat().st_mtime,
                                     "fixed_base": self.fixed_base}, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"[isaac] URDF → USD 변환 {time.perf_counter() - t0:.1f}s: {usd}")
        return usd

    def _build_world(self, start, init_q, init_base_z):
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager

        self._app_utils, self._stage_utils, self._sm = app_utils, stage_utils, SimulationManager

        # 물리 엔진 선택 (physx / newton). newton 은 기본 python 앱에 안 실려 있어 확장을 먼저 켠다
        if self.engine == "newton":
            for ext in ("isaacsim.physics.newton", "isaacsim.physics.newton.tensors"):   # 엔진 + 텐서 API 백엔드
                ok = app_utils.enable_extension(ext)
                print(f"[isaac] {ext} 확장 켜기: {'OK' if ok else '실패'}", flush=True)
            self.app.update()
        engines = dict(SimulationManager.get_available_physics_engines())
        if self.engine in engines:
            if not engines[self.engine]:
                ok = SimulationManager.switch_physics_engine(self.engine)
                print(f"[isaac] 물리 엔진 {self.engine} 전환: {'OK' if ok else '실패'}")
        elif engines:
            print(f"[isaac] 엔진 {self.engine} 없음 → 사용 가능: {list(engines)} (활성 엔진 그대로)")
        self.engine_active = SimulationManager.get_active_physics_engine() if engines else "?"
        SimulationManager.setup_simulation(dt=self.physics_dt, device=self.device)

        stage = stage_utils.get_current_stage()
        from omni.physx.scripts import physicsUtils
        from pxr import Gf
        # 바닥: 자유 베이스는 접촉면, 고정 베이스(0.85 m)는 발이 닿지 않아 물리 영향 없이 기준면 역할만 한다
        physicsUtils.add_ground_plane(stage, "/World/ground", "Z", 50.0, Gf.Vec3f(0.0), Gf.Vec3f(0.18, 0.2, 0.22))
        if self.render_enabled:                                       # 빈 스테이지에는 조명이 없어 로봇이 검게 보인다
            from pxr import UsdGeom, UsdLux
            dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
            dome.CreateIntensityAttr(350.0)
            key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
            key.CreateIntensityAttr(1800.0)
            key.CreateAngleAttr(1.0)
            UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 35.0))
        usd = self._import_urdf()
        # 변환 결과는 다중 물리(variant set "Physics": physx / mujoco / physics / none) 패키지라 엔진에 맞는 variant 를 골라야
        # 관절·ArticulationRootAPI 가 생긴다 (기본 선택이 none 이면 물리 없음)
        variant = self.physics_variant or ("mujoco" if self.engine == "newton" else "physx")
        stage_utils.add_reference_to_stage(usd_path=str(usd), path=ROBOT_PRIM, variants=[("Physics", variant)])
        print(f"[isaac] Physics variant = {variant}", flush=True)
        t0 = time.perf_counter()
        while stage_utils.is_stage_loading():
            self.app.update()
        print(f"[isaac] 로봇 USD 로드 {time.perf_counter() - t0:.1f}s", flush=True)

        # 초기 자세
        if init_q is not None:
            q0 = np.clip(np.asarray(init_q, dtype=float).reshape(NUM_JOINTS), Q_LOWER, Q_UPPER)
        elif start == "standing" or self.fixed_base:
            q0 = Q_DEFAULT.copy()
        else:
            q0 = np.clip(Q_CROUCH, Q_LOWER, Q_UPPER)
        z = init_base_z if init_base_z is not None else (0.85 if self.fixed_base else (0.37 if start == "standing" else 0.32))
        # 로봇을 높이 z 에 놓는다: 참조 프림(/World/rover) 자체를 옮긴다. 고정 베이스의 root_joint 는 body0 이 이 프림이라
        # 앵커가 같이 따라오고, 자유 베이스는 몸통 시작 높이가 된다. (articulation root 프림을 직접 옮기면 안 된다)
        from isaacsim.core.experimental.prims import XformPrim
        XformPrim(ROBOT_PRIM, positions=np.array([[0.0, 0.0, z]]), orientations=np.array([[1.0, 0.0, 0.0, 0.0]]),
                  reset_xform_op_properties=True)
        self.robot = Articulation(ROBOT_PRIM)
        print(f"[isaac] Articulation: {self.robot.paths}", flush=True)

        # PhysX 결과를 substep(1 ms)마다 USD 에 쓰면 매우 느리다 → fabric 으로 보내고(USD 갱신 끔) 화면 갱신 때만 반영한다
        if self.fabric:
            try:
                SimulationManager.enable_fabric(True)
            except Exception as exc:
                print(f"[isaac] fabric 켜기 실패 (USD 갱신으로 진행): {exc}")
        # 물리 초기화: 타임라인 없이 수동 (실패 시 타임라인 재생 방식으로 전환)
        SimulationManager.initialize_physics()
        print(f"[isaac] 물리 초기화 (수동) → tensor valid={self.robot.is_physics_tensor_entity_valid()}", flush=True)
        if not self.robot.is_physics_tensor_entity_valid() or os.environ.get("ROVER_ISAAC_MODE") == "timeline":
            self._setup_timeline()
            app_utils.play()
            self.app.update()
            if not self.robot.is_physics_tensor_entity_valid():
                raise RuntimeError("Isaac Sim 물리 텐서 뷰를 만들지 못했습니다 (articulation 초기화 실패)")
            self.mode = "timeline"
        print(f"[isaac] 물리 엔진 {self.engine_active}, 진행 방식 {self.mode}, physics_dt {self.physics_dt * 1e3:.1f} ms × {self.substeps}")

        names = list(self.robot.dof_names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if missing:
            raise RuntimeError(f"URDF 관절 이름 불일치: {missing} (있는 이름: {names})")
        self.idx = np.array([names.index(n) for n in JOINT_NAMES])     # 논리 순서 → Isaac dof 순서
        self.ndof = len(names)
        self.robot.switch_dof_control_mode("effort")                      # 드라이브 게인 0: 우리가 토크를 넣는다
        try:
            full_max = np.full((1, self.ndof), 1e3)
            full_max[0, self.idx] = self.tau_max
            self.robot.set_dof_max_efforts(full_max)
        except Exception as exc:  # 한계 설정은 선택 사항 (우리가 직접 자른다)
            print(f"[isaac] max effort 설정 생략: {exc}")
        full = np.zeros((1, self.ndof))
        full[0, self.idx] = q0
        self.robot.set_dof_positions(full)
        self.robot.set_dof_velocities(np.zeros((1, self.ndof)))
        if not self.fixed_base:
            # 물리 초기화의 워밍업 스텝은 관절 0(다리를 편 자세)으로 돌아서 발이 바닥을 뚫고, 그 반발로 몸통에 속도가 생긴다.
            # 관절을 시작 자세로 옮긴 뒤 몸통 자세와 속도도 다시 맞춘다 (안 하면 시작하자마자 솟구치며 기울어 넘어짐 판정).
            self.robot.set_world_poses(positions=np.array([[0.0, 0.0, z]]), orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            self.robot.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
        self._step_physics(1)
        if self.render_enabled:
            try:
                from isaacsim.core.rendering_manager import ViewportManager
                ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.7, 1.5, z + 0.45], target=[0.0, 0.0, z - 0.25])
            except Exception as exc:
                print(f"[isaac] 카메라 설정 생략: {exc}")
            for _ in range(3):
                self._render()

    def _setup_timeline(self):
        """타임라인 재생 방식: app.update() 한 번이 제어 주기 dt 만큼 진행하도록 고정 시간 간격을 건다."""
        import carb
        import omni.timeline
        tl = omni.timeline.get_timeline_interface()
        rate = round(1.0 / self.dt)
        for name, fn in (("useFixedTimeStepping", lambda: carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)),
                         ("target_framerate", lambda: tl.set_target_framerate(rate)),
                         ("time_codes_per_second", lambda: tl.set_time_codes_per_second(rate))):
            try:
                fn()
            except Exception as exc:
                print(f"[isaac] 타임라인 설정 {name} 실패: {exc}")

    # ---- 물리 진행 ----
    def _step_physics(self, n: int):
        if self.mode == "manual":
            self._sm.step(steps=n)
        else:
            self.app.update()

    # ---- 공통 인터페이스 ----
    def wait_ready(self) -> State:
        return self.read()

    def _joint_state(self):
        q = self.robot.get_dof_positions().numpy()[0][self.idx]
        dq = self.robot.get_dof_velocities().numpy()[0][self.idx]
        return np.asarray(q, dtype=float), np.asarray(dq, dtype=float)

    def base_pos(self) -> np.ndarray:
        pos, _ = self.robot.get_world_poses()
        return np.asarray(pos.numpy()[0], dtype=float)

    def read(self) -> State:
        q, dq = self._joint_state()
        pos, quat = self.robot.get_world_poses()
        pos = np.asarray(pos.numpy()[0], dtype=float)
        quat = np.asarray(quat.numpy()[0], dtype=float)                 # wxyz
        rot = _quat_to_rot(quat)
        try:
            _, ang = self.robot.get_velocities()
            omega_w = np.asarray(ang.numpy()[0], dtype=float)
        except Exception:
            omega_w = np.zeros(3)
        gyro = rot.T @ omega_w
        acc = rot.T @ np.array([0.0, 0.0, 9.81])                       # 근사: 중력 반작용만 (정지 시 실기와 같은 방향)
        t = self.tick * self.dt
        return State(t=t, q=q, dq=dq, quat_wxyz=quat, gyro=gyro, acc=acc, tau_est=self.last_tau.copy(), age=0.0,
                     extra={"base_pos": pos, "sim_time": t})

    def send(self, cmd: JointCmd) -> None:
        tau = self.last_tau
        full = np.zeros((1, self.ndof))
        if self.mode == "manual":
            for _ in range(self.substeps):
                q, dq = self._joint_state()
                tau = np.clip(cmd.torque(q, dq), -self.tau_max, self.tau_max)
                full[0, self.idx] = tau
                self.robot.set_dof_efforts(full)
                self._sm.step(steps=1)
        else:                                                          # 타임라인 방식: 틱마다 토크 1회
            q, dq = self._joint_state()
            tau = np.clip(cmd.torque(q, dq), -self.tau_max, self.tau_max)
            full[0, self.idx] = tau
            self.robot.set_dof_efforts(full)
            self.app.update()
        self.last_tau = tau
        self.tick += 1
        if not self.headless:
            now = time.perf_counter()                                  # 화면은 벽시계 기준 render_fps 로만 갱신
            if self.mode == "manual" and now - self._last_render >= self.render_period:
                self._last_render = now
                self._render()
            if not self.app.is_running():
                raise KeyboardInterrupt

    def nominal_gravity(self, q=None, quat=None) -> np.ndarray:
        """현재 자세에서 중력을 상쇄하는 관절 토크 (Isaac 의 gravity compensation). q 인자는 무시하고 현재 상태를 쓴다."""
        g = self.robot.get_dof_gravity_compensation_forces().numpy()[0][self.idx]
        return np.asarray(g, dtype=float)

    def _render(self) -> None:
        """물리를 진행하지 않고 화면만 갱신한다. PhysX fabric 이 켜져 있으면 최신 물리 결과를 먼저 fabric 에 반영한다."""
        try:
            if self.mode == "manual" and self._sm.is_fabric_enabled():
                import omni.physxfabric
                view = self._sm.get_physics_sim_view()
                if view is not None and hasattr(view, "update_articulations_kinematic"):
                    view.update_articulations_kinematic()              # 링크 자세를 렌더링용으로 갱신 (Isaac Lab 방식)
                omni.physxfabric.get_physx_fabric_interface().update(0.0, 0.0)   # (0, 0): 시간과 무관하게 즉시 반영
        except Exception as exc:
            if not getattr(self, "_fabric_warned", False):
                self._fabric_warned = True
                print(f"[isaac] fabric 갱신 실패: {type(exc).__name__}: {exc}", flush=True)
        try:
            from isaacsim.core.rendering_manager import RenderingManager
            RenderingManager.render()
        except Exception:
            self.app.update()

    def snapshot(self, path) -> bool:
        """뷰포트 이미지를 저장한다 (창 모드 또는 render=True 인 헤드리스). 확장자에 따라 png/jpg."""
        if not self.render_enabled:
            return False
        try:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists():
                path.unlink()
            vp = get_active_viewport()
            if vp is None:
                return False
            capture_viewport_to_file(vp, str(path))
            for _ in range(90):
                self._render()
                if path.exists() and path.stat().st_size > 0:
                    return True
            return path.exists()
        except Exception as exc:
            print(f"[isaac] 스냅샷 실패: {exc}")
            return False

    def close(self) -> None:
        """Kit 종료는 인터프리터가 끝날 때로 미룬다. fast_shutdown(기본) 은 app.close() 에서 프로세스를 즉시 끝내므로
        여기서 바로 닫으면 호출자의 뒷정리(요약 파일·메타 갱신·출력)가 사라진다. 호출 시 SimulationApp 의 atexit 훅이 닫는다."""
        if getattr(self, "_closed", False):
            return
        self._closed = True
        sys.stdout.flush(); sys.stderr.flush()
        if not getattr(self, "app", None):
            return
        if self.app.config.get("fast_shutdown", True):
            import atexit
            atexit.register(self._close_now)          # LIFO: SimulationApp 자체 훅보다 먼저 돈다
        else:
            self._close_now()

    def _close_now(self) -> None:
        try:
            self.app.close()
        except Exception:
            pass
