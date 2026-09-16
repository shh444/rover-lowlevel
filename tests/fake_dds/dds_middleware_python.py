"""dds_middleware_python 의 가짜 구현(in-process 가상 로봇). 실기 없이 DDS 백엔드 경로를 끝까지 검증한다.

    PYTHONPATH=tests/fake_dds MUJOCO_GL=egl python run.py --backend dds --robot-ip 127.0.0.1 --yes --program standup

실제 SDK 와 같은 표면만 흉내 낸다:
    PyDDSMiddleware(config).subscribeLowerState / createLowerCmdWriter / publishLowerCmd
    LowerCmd()[hw].mode/q/dq/tau/kp/kd
    LowerState.motor_state()[i].q()/dq()/ddq()/tau_est()/mode()/motor_temp(), imu_state().quaternion()/...
로봇 몸체는 lowlevel/virtual_plant.py(MuJoCo, 하드웨어 슬롯 규약·SDK 토크 공식) 이고, 상태는 별도 스레드가
2ms(500Hz)마다 콜백으로 전달한다. 실제 SDK 로 DDS 통신까지 검증하려면 tools/virtual_robot_dds.py 를 쓴다.

고장 주입(환경변수):
    FAKE_DDS_STOP_AT=<s>   그 시각(시작 후 초) 이후 상태 발행 중단          → 워치독 시험
    FAKE_DDS_TILT_AT=<s>   그 시각 이후 IMU 자세를 x축 60도 기울여 보고      → 넘어짐 시험
    FAKE_DDS_XML=<path>    MJCF 경로 (기본: lowlevel.common.find_default_xml())
    FAKE_DDS_REPORT=<path> 종료 시 가상 로봇 통계 JSON 저장
"""
from __future__ import annotations

import atexit
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "disable")   # 렌더링 안 함. EGL 이 없는 컨테이너에서도 import 되게
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from lowlevel.virtual_plant import VirtualPlant   # noqa: E402

PUB_DT = 0.002


# ---------------- SDK 메시지 표면 ----------------
class _MotorCmd:
    def __init__(self):
        self._v = {"mode": 0, "q": 0.0, "dq": 0.0, "tau": 0.0, "kp": 0.0, "kd": 0.0}

    def _acc(self, key, value):
        if value is None:
            return self._v[key]
        self._v[key] = value

    def mode(self, v=None): return self._acc("mode", v)
    def q(self, v=None): return self._acc("q", v)
    def dq(self, v=None): return self._acc("dq", v)
    def tau(self, v=None): return self._acc("tau", v)
    def kp(self, v=None): return self._acc("kp", v)
    def kd(self, v=None): return self._acc("kd", v)


class LowerCmd:
    def __init__(self):
        self._m = [_MotorCmd() for _ in range(16)]

    def __getitem__(self, i):
        return self._m[i]

    def motor_cmd(self):
        return self._m


class _MotorState:
    def __init__(self, q, dq, tau, mode, temp):
        self._q, self._dq, self._tau = float(q), float(dq), float(tau)
        self._mode, self._temp = int(mode), int(temp)

    def q(self): return self._q
    def dq(self): return self._dq
    def ddq(self): return 0.0
    def tau_est(self): return self._tau
    def q_raw(self): return self._q
    def dq_raw(self): return self._dq
    def ddq_raw(self): return 0.0
    def mode(self): return self._mode
    def motor_temp(self): return self._temp


class _ImuState:
    def __init__(self, quat, gyro, acc, rpy, stamp):
        self._quat, self._gyro, self._acc, self._rpy, self._stamp = quat, gyro, acc, rpy, stamp

    def quaternion(self): return list(self._quat)
    def gyroscope(self): return list(self._gyro)
    def accelerometer(self): return list(self._acc)
    def rpy(self): return list(self._rpy)
    def timestamp(self): return int(self._stamp)


class _BmsState:
    def battery_level(self): return 88


class LowerState:
    def __init__(self, motors, imu, bms):
        self._motors, self._imu, self._bms = motors, imu, bms

    def motor_state(self): return self._motors
    def imu_state(self): return self._imu
    def bms_state(self): return self._bms


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


# ---------------- 가상 로봇 (스레드 + 고장 주입) ----------------
class _VirtualRobot:
    def __init__(self):
        self.plant = VirtualPlant(xml=os.environ.get("FAKE_DDS_XML") or None)
        self.lock = threading.Lock()
        self.callbacks = []
        self.stop_at = float(os.environ.get("FAKE_DDS_STOP_AT", "inf"))
        self.tilt_at = float(os.environ.get("FAKE_DDS_TILT_AT", "inf"))
        self.tilt = np.array([math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0, 0.0])
        self.states = 0
        self._stop = threading.Event()
        self.t0 = time.monotonic()
        # 인터프리터 종료 중에도 스레드가 MuJoCo 를 밟으면 segfault 가 나므로, atexit 에서 먼저 멈추고 join 한다.
        atexit.register(self._shutdown)
        self.thread = threading.Thread(target=self._loop, name="fake-dds-robot", daemon=True)
        self.thread.start()

    def set_cmd(self, lc: LowerCmd):
        cols = {k: [lc[i]._v[k] for i in range(16)] for k in ("q", "dq", "tau", "kp", "kd", "mode")}
        with self.lock:
            self.plant.set_cmd(cols["q"], cols["dq"], cols["tau"], cols["kp"], cols["kd"], cols["mode"])

    def _make_state(self, s, elapsed):
        quat = s["quat_wxyz"]
        if elapsed >= self.tilt_at:
            quat = _qmul(quat, self.tilt)
        motors = [_MotorState(s["q_hw"][i], s["dq_hw"][i], s["tau_hw"][i], s["mode_hw"][i], 35) for i in range(16)]
        return LowerState(motors, _ImuState(quat, s["gyro"], s["acc"], _rpy(quat), time.monotonic_ns()), _BmsState())

    def _loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += PUB_DT
            with self.lock:
                self.plant.step(PUB_DT)
                s = self.plant.sample()
            elapsed = time.monotonic() - self.t0
            if elapsed < self.stop_at:
                state = self._make_state(s, elapsed)
                for cb in list(self.callbacks):
                    cb(state)
                self.states += 1
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _shutdown(self):
        self._stop.set()
        self.thread.join(timeout=3.0)
        path = os.environ.get("FAKE_DDS_REPORT")
        if not path:
            return
        with self.lock:
            out = self.plant.report()
        out.update(states=self.states, stop_at=self.stop_at, tilt_at=self.tilt_at, kind="fake_dds_in_process")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(out, indent=2), encoding="utf-8")


_ROBOT = None


def _robot() -> _VirtualRobot:
    global _ROBOT
    if _ROBOT is None:
        _ROBOT = _VirtualRobot()
    return _ROBOT


class PyDDSMiddleware:
    def __init__(self, config):
        self.config = str(config)
        self.writers = {}

    def subscribeLowerState(self, topic, cb, qos=None):
        assert topic == "rt/lower/state", topic
        _robot().callbacks.append(cb)

    def createLowerCmdWriter(self, topic, qos=None):
        assert topic == "rt/lower/cmd", topic
        self.writers[topic] = dict(qos or {})

    def publishLowerCmd(self, cmd):
        assert "rt/lower/cmd" in self.writers, "createLowerCmdWriter 먼저"
        _robot().set_cmd(cmd)
