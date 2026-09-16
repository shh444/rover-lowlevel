"""실기 DDS 백엔드: SDK low_level E9 와 같은 토픽·메시지·매핑을 쓴다.

    구독 rt/lower/state (LowerState) → State      (16 슬롯 → 논리 12, 오프셋 제거)
    발행 rt/lower/cmd   (LowerCmd)   ← JointCmd   (논리 12 → 16 슬롯, 오프셋 더함, mode 0)

반드시 kill_robot 으로 주 제어기를 끈 뒤에만 쓴다. CYCLONEDDS_URI 로 네트워크 인터페이스가
지정된 cyclonedds.xml 이 필요하다 (SDK setup_cyclonedds_env.sh 또는 cyclonedds.xml).
"""
from __future__ import annotations

import os
import threading
import time

import numpy as np

from .common import (ABS2HW, HW_INDEX, HW_OFFSET, MOTOR_OFFSET, NUM_HW_MOTORS, NUM_JOINTS,
                     JointCmd, State)

CMD_QOS = {"reliability": "reliable", "history_kind": "keep_last",
           "history_depth": 1, "durability": "volatile"}          # E9 와 동일


class DdsBackend:
    name = "dds"

    def __init__(self, config_path, state_topic: str = "rt/lower/state",
                 cmd_topic: str = "rt/lower/cmd", timeout: float = 10.0,
                 discovery_wait: float = 1.0, create_writer: bool = True):
        import dds_middleware_python as dds   # SDK dist/ 의 wheel (CPython 3.10)
        if not os.environ.get("CYCLONEDDS_URI"):
            print("[경고] CYCLONEDDS_URI 가 비어 있습니다. 네트워크 인터페이스가 지정된 cyclonedds.xml 을 설정하세요.")
        self.dds = dds
        self.timeout = float(timeout)
        self.discovery_wait = float(discovery_wait)
        self._lock = threading.Lock()
        self._latest: State | None = None
        self.count = 0
        self.parse_errors = 0
        self.has_writer = bool(create_writer)
        self.mw = dds.PyDDSMiddleware(str(config_path))
        self.mw.subscribeLowerState(state_topic, self._on_state)
        if create_writer:
            self.mw.createLowerCmdWriter(cmd_topic, dict(CMD_QOS))
        self._writer_created = time.monotonic()

    # ---- 수신 스레드 ----
    def _on_state(self, msg) -> None:
        now = time.monotonic()
        try:
            motors = msg.motor_state()
            imu = msg.imu_state()
            q_hw = np.array([float(motors[i].q()) for i in range(NUM_HW_MOTORS)])
            dq_hw = np.array([float(motors[i].dq()) for i in range(NUM_HW_MOTORS)])
            tau_hw = np.array([float(motors[i].tau_est()) for i in range(NUM_HW_MOTORS)])
            mode_hw = np.array([int(motors[i].mode()) for i in range(NUM_HW_MOTORS)])
            temp_hw = np.array([int(motors[i].motor_temp()) for i in range(NUM_HW_MOTORS)])
            st = State(t=now,
                       q=q_hw[HW_INDEX] - HW_OFFSET,
                       dq=dq_hw[HW_INDEX],
                       quat_wxyz=np.array(list(imu.quaternion()), dtype=float),
                       gyro=np.array(list(imu.gyroscope()), dtype=float),
                       acc=np.array(list(imu.accelerometer()), dtype=float),
                       tau_est=tau_hw[HW_INDEX], age=0.0, motor_mode=mode_hw[HW_INDEX],
                       extra={"mode_hw": mode_hw, "temp_hw": temp_hw})
        except Exception as exc:
            self.parse_errors += 1
            if self.parse_errors <= 3:
                print(f"[DDS] 상태 파싱 실패: {exc!r}")
            return
        with self._lock:
            self._latest = st
            self.count += 1

    # ---- 공통 인터페이스 ----
    def wait_ready(self) -> State:
        t0 = time.monotonic()
        while True:
            with self._lock:
                ready = self._latest is not None
            if ready:
                break
            if time.monotonic() - t0 > self.timeout:
                raise RuntimeError("rt/lower/state 수신 없음: 유선 연결(192.168.5.x), CYCLONEDDS_URI, "
                                   "인터페이스 이름, cyclonedds ps 출력을 확인하세요")
            time.sleep(0.01)
        # DDS 엔티티 탐색 대기 (SDK E7 문서: writer 생성 직후 발행하면 유실될 수 있음)
        remain = self.discovery_wait - (time.monotonic() - self._writer_created)
        if remain > 0:
            time.sleep(remain)
        return self.read()

    def read(self) -> State:
        with self._lock:
            st = self._latest
        if st is None:
            raise RuntimeError("아직 상태를 받지 못했습니다 (wait_ready 먼저)")
        return State(t=st.t, q=st.q.copy(), dq=st.dq.copy(), quat_wxyz=st.quat_wxyz.copy(),
                     gyro=st.gyro.copy(), acc=st.acc.copy(), tau_est=st.tau_est.copy(),
                     age=time.monotonic() - st.t,
                     motor_mode=None if st.motor_mode is None else st.motor_mode.copy(),
                     extra=dict(st.extra))

    def send(self, cmd: JointCmd) -> None:
        if not self.has_writer:
            raise RuntimeError("읽기 전용(create_writer=False)으로 열린 백엔드에는 명령을 보낼 수 없습니다")
        lc = self.dds.LowerCmd()
        for i in range(NUM_JOINTS):
            hw = ABS2HW[i]
            m = lc[hw]
            m.mode(0)
            m.q(float(cmd.q[i] + MOTOR_OFFSET[hw]))
            m.dq(float(cmd.dq[i]))
            m.tau(float(cmd.tau[i]))
            m.kp(float(cmd.kp[i]))
            m.kd(float(cmd.kd[i]))
        self.mw.publishLowerCmd(lc)

    def snapshot(self, path) -> bool:
        return False

    def close(self) -> None:
        pass
