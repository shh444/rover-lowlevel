"""DDS 백엔드의 슬롯 매핑·오프셋 검증. 실제 SDK 없이 dds_middleware_python 스텁으로 돌린다.

    python tests/test_dds_mapping.py
"""
import math
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# ---- dds_middleware_python 스텁 (SDK E9 가 쓰는 표면만) ----
class _MotorCmd:
    def __init__(self):
        self.v = {}

    def mode(self, x): self.v["mode"] = x
    def q(self, x): self.v["q"] = x
    def dq(self, x): self.v["dq"] = x
    def tau(self, x): self.v["tau"] = x
    def kp(self, x): self.v["kp"] = x
    def kd(self, x): self.v["kd"] = x


class LowerCmd:
    def __init__(self):
        self.slots = [_MotorCmd() for _ in range(16)]

    def __getitem__(self, i):
        return self.slots[i]


class PyDDSMiddleware:
    def __init__(self, cfg):
        self.cfg, self.cb, self.qos, self.published = cfg, None, None, []

    def subscribeLowerState(self, topic, cb):
        self.state_topic, self.cb = topic, cb

    def createLowerCmdWriter(self, topic, qos):
        self.cmd_topic, self.qos = topic, qos

    def publishLowerCmd(self, cmd):
        self.published.append(cmd)


stub = types.ModuleType("dds_middleware_python")
stub.LowerCmd, stub.PyDDSMiddleware = LowerCmd, PyDDSMiddleware
sys.modules["dds_middleware_python"] = stub

from lowlevel.backend_dds import DdsBackend                       # noqa: E402
from lowlevel.common import ABS2HW, MOTOR_OFFSET, JointCmd       # noqa: E402


class _MotorState:
    def __init__(self, q, i):
        self._q, self._i = q, i

    def q(self): return self._q
    def dq(self): return 0.1 * self._i
    def ddq(self): return 0.0
    def tau_est(self): return 0.5
    def mode(self): return 4
    def motor_temp(self): return 35


class _Imu:
    def quaternion(self): return [1.0, 0.0, 0.0, 0.0]
    def gyroscope(self): return [0.0, 0.0, 0.0]
    def accelerometer(self): return [0.0, 0.0, 9.81]


class _LowerState:
    def __init__(self, q_hw):
        self._m = [_MotorState(q, i) for i, q in enumerate(q_hw)]

    def motor_state(self): return self._m
    def imu_state(self): return _Imu()


def main():
    io = DdsBackend("dummy.yaml", discovery_wait=0.0, timeout=1.0)
    assert io.mw.state_topic == "rt/lower/state" and io.mw.cmd_topic == "rt/lower/cmd"
    assert io.mw.qos == {"reliability": "reliable", "history_kind": "keep_last",
                         "history_depth": 1, "durability": "volatile"}

    # 하드웨어 각도 = 오프셋 + 0.01*슬롯번호 → 논리 각도는 0.01*슬롯번호 여야 한다
    q_hw = [MOTOR_OFFSET[i] + 0.01 * i for i in range(16)]
    io.mw.cb(_LowerState(q_hw))
    st = io.wait_ready()
    assert np.allclose(st.q, [0.01 * hw for hw in ABS2HW]), st.q
    assert np.allclose(st.dq, [0.1 * hw for hw in ABS2HW])
    assert st.motor_mode.tolist() == [4] * 12 and st.extra["temp_hw"].tolist() == [35] * 16
    assert np.allclose(st.gravity_body(), [0, 0, -1])
    assert st.age >= 0.0

    # 논리 명령 → 슬롯 명령: q+offset, mode 0, 나머지 슬롯(3,7,11,15)은 건드리지 않음
    cmd = JointCmd.pd(np.arange(12) * 0.1, 30.0, 1.2)
    io.send(cmd)
    lc = io.mw.published[-1]
    for i, hw in enumerate(ABS2HW):
        v = lc[hw].v
        assert v["mode"] == 0
        assert math.isclose(v["q"], 0.1 * i + MOTOR_OFFSET[hw]), (i, hw, v)
        assert v["kp"] == 30.0 and v["kd"] == 1.2 and v["dq"] == 0.0 and v["tau"] == 0.0
    for hw in (3, 7, 11, 15):
        assert lc[hw].v == {}
    # 왕복: 보낸 하드웨어 각도를 다시 읽으면 논리 각도가 돌아와야 한다
    io.mw.cb(_LowerState([lc[i].v.get("q", 0.0) for i in range(16)]))
    assert np.allclose(io.read().q, cmd.q)
    print("test_dds_mapping: OK")


if __name__ == "__main__":
    main()
