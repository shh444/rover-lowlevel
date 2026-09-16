"""Rover(Dobot Quad) 저수준 제어 공통 정의.

논리 관절 순서(12개)는 SDK E9 예제와 dobot_sim2real 의 train_joint_order 와 같다.
    0-2  : FL abad / thigh / calf
    3-5  : FR
    6-8  : RL
    9-11 : RR
하드웨어 슬롯(16개)과의 대응은 ABS2HW, 영점 보정은 MOTOR_OFFSET 을 SDK 값 그대로 쓴다.
    읽기: q_logical = q_hw[ABS2HW[i]] - MOTOR_OFFSET[ABS2HW[i]]
    쓰기: q_hw[hw]   = q_logical[i]   + MOTOR_OFFSET[hw]
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path

import numpy as np

NUM_JOINTS = 12
NUM_HW_MOTORS = 16
LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("abad", "thigh", "calf")
JOINT_SHORT = [f"{leg}_{part}" for leg in LEGS for part in PARTS]
JOINT_NAMES = [
    "joint_front_left_abad", "joint_front_left_thigh_pitch", "joint_front_left_calf_pitch",
    "joint_front_right_abad", "joint_front_right_thigh_pitch", "joint_front_right_calf_pitch",
    "joint_rear_left_abad", "joint_rear_left_thigh_pitch", "joint_rear_left_calf_pitch",
    "joint_rear_right_abad", "joint_rear_right_thigh_pitch", "joint_rear_right_calf_pitch",
]

# --- SDK low_level/python/e9_motor_cmd_pub.py 와 동일한 하드웨어 매핑 ---
ABS2HW = (0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14)
MOTOR_OFFSET = (-0.05, -0.5, 1.17, 0.0,
                0.05, -0.5, 1.17, 0.0,
                -0.05, 0.5, -1.17, 0.0,
                0.05, 0.5, -1.17, 0.0)
HW_INDEX = np.array(ABS2HW)
HW_OFFSET = np.array(MOTOR_OFFSET)[HW_INDEX]        # 논리 순서로 늘어놓은 오프셋 (12,)

# --- URDF/MJCF 한계 (resources/dobot_quad/urdf/dobot_quad_ros.urdf) ---
Q_LOWER = np.tile([-0.6632, -2.618, -2.53], 4)
Q_UPPER = np.tile([0.6632, 2.618, 2.53], 4)
TAU_MAX = np.tile([23.0, 23.0, 55.0], 4)            # N·m, MJCF ctrlrange 와 같음
DQ_MAX = 25.0                                       # rad/s, URDF velocity(21~23) 초과 → 이상

# --- dobot_sim2real/config/robot.yaml 의 자세와 게인 ---
Q_DEFAULT = np.array([0.0, 0.84, -1.30, 0.0, 0.84, -1.30,
                      0.0, 0.75, -1.22, 0.0, 0.75, -1.22])   # 서기 자세
Q_CROUCH = np.array([0.0, 1.5, -2.65] * 4)                   # 엎드림. calf -2.65 는 URDF 한계(-2.53) 밖 → 잘라 씀
KP_STAND, KD_STAND = 60.0, 1.8                               # 기립 단계
KP_WALK, KD_WALK = 10.0, 1.0                                 # 보행 정책 단계 (참고용)
KD_DAMP = 3.0                                                # 댐핑 보호 모드
KP_SINE, KD_SINE = 30.0, 1.2                                 # SDK E9 사인 구동

# --- 명령 상한 (안전 가드) ---
KP_MAX, KD_MAX, TAU_FF_MAX = 100.0, 5.0, 10.0


@dataclass
class State:
    """백엔드가 돌려주는 로봇 상태 (논리 관절 순서, 오프셋 제거 후)."""
    t: float                         # 수신(또는 시뮬) 시각 s
    q: np.ndarray                    # (12,) rad
    dq: np.ndarray                   # (12,) rad/s
    quat_wxyz: np.ndarray            # (4,) 몸체 자세
    gyro: np.ndarray                 # (3,) rad/s, 몸체 좌표계
    acc: np.ndarray                  # (3,) m/s², 몸체 좌표계
    tau_est: np.ndarray              # (12,) N·m (실기: 추정 토크, 시뮬: 적용 토크)
    age: float = 0.0                 # 상태 나이 s (DDS 워치독용)
    motor_mode: np.ndarray | None = None   # (12,) 실기 모터 mode
    extra: dict = field(default_factory=dict)

    def gravity_body(self) -> np.ndarray:
        """중력 방향 단위벡터를 몸체 좌표계로. 똑바로 서 있으면 [0, 0, -1]."""
        w, x, y, z = self.quat_wxyz
        return np.array([-2.0 * (x * z - w * y),
                         -2.0 * (y * z + w * x),
                         -(1.0 - 2.0 * (x * x + y * y))])


@dataclass
class JointCmd:
    """관절 명령 (논리 순서). 실기 모터 드라이버/시뮬 PD 가 계산하는 토크:
        tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff      (SDK low_level 문서 공식)
    """
    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray

    @staticmethod
    def _vec(x) -> np.ndarray:
        return np.broadcast_to(np.asarray(x, dtype=float), (NUM_JOINTS,)).copy()

    @classmethod
    def pd(cls, q, kp, kd, dq=0.0, tau=0.0) -> "JointCmd":
        return cls(cls._vec(q), cls._vec(dq), cls._vec(kp), cls._vec(kd), cls._vec(tau))

    @classmethod
    def damping(cls, kd=KD_DAMP, q=0.0) -> "JointCmd":
        """kp=0, tau=0 인 댐핑 명령. q 는 무의미하지만 기록용으로 현재각을 넣는다."""
        return cls.pd(q, 0.0, kd)

    def torque(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        return self.kp * (self.q - q) + self.kd * (self.dq - dq) + self.tau

    def is_damping(self) -> bool:
        return bool(np.all(self.kp == 0.0) and np.all(self.tau == 0.0))


def cosine_interp(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """sim2real io.interp 와 같은 코사인 보간. s∈[0,1]."""
    s = float(np.clip(s, 0.0, 1.0))
    return a + 0.5 * (1.0 - math.cos(math.pi * s)) * (b - a)


def joint_mask(spec: str) -> np.ndarray:
    """'all' | 'abad,calf' | 'FL,RR' | '0,1,2' → (12,) bool."""
    spec = (spec or "all").strip()
    if spec.lower() == "all":
        return np.ones(NUM_JOINTS, dtype=bool)
    mask = np.zeros(NUM_JOINTS, dtype=bool)
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if token.lower() in PARTS:
            mask[PARTS.index(token.lower())::3] = True
        elif token.upper() in LEGS:
            k = LEGS.index(token.upper())
            mask[3 * k:3 * k + 3] = True
        elif token.isdigit() and 0 <= int(token) < NUM_JOINTS:
            mask[int(token)] = True
        else:
            raise ValueError(f"알 수 없는 관절 지정: {token}")
    return mask


def fmt(v, nd=3) -> str:
    return "[" + " ".join(f"{float(x):+.{nd}f}" for x in np.asarray(v).ravel()) + "]"


XML_REL = Path("dobot_rover_simulation/dobot_sim2real/resources/mujoco/dobot_quad.xml")


def find_default_xml() -> Path | None:
    """dobot_sim2real 의 dobot_quad.xml 후보 경로 중 처음 존재하는 것. 없으면 None.
    후보: $ROVER_VENDOR, <이 저장소>/vendor, /work/vendor(컨테이너), ~/rover-mujoco-poc/vendor(thor)."""
    here = Path(__file__).resolve().parents[1]
    candidates = [here / "vendor", Path("/work/vendor"), Path.home() / "rover-mujoco-poc/vendor"]
    if os.environ.get("ROVER_VENDOR"):
        candidates.insert(0, Path(os.environ["ROVER_VENDOR"]))
    for base in candidates:
        if (base / XML_REL).exists():
            return base / XML_REL
    return None
