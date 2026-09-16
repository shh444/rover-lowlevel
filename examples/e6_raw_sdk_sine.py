#!/usr/bin/env python3
"""예제 6: 패키지 없이 SDK(dds_middleware_python)만으로 E9 식 사인 구동 — 실기 전용, 학습용.

SDK 의 low_level/python/e9_motor_cmd_pub.py 와 같은 API 를 쓰되, 실기에서 중요한 네 가지를 추가했다.
  1. 주 제어기(gRPC 50051)가 살아 있으면 시작하지 않는다 (kill_robot 먼저)
  2. 첫 상태를 받은 뒤, writer 생성 후 1초 기다린다 (DDS 탐색; 바로 보내면 유실)
  3. 진폭을 1초에 걸쳐 키운다 (E9 는 첫 명령에서 목표가 점프한다)
  4. 어떤 경우에도(정상 종료, Ctrl+C, 예외) 마지막에 댐핑 1초를 보낸다

    export CYCLONEDDS_URI=file://.../cyclonedds.xml     # 유선 인터페이스가 지정된 설정
    python examples/e6_raw_sdk_sine.py --joints thigh --amp 0.1 --seconds 5
"""
import argparse
import math
import socket
import sys
import threading
import time

import dds_middleware_python as dds   # SDK dist/ 의 wheel (CPython 3.10)

# SDK E9 와 동일한 매핑
NUM_MOTORS = 12
ABS2HW = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
MOTOR_OFFSET = [-0.05, -0.5, 1.17, 0.0, 0.05, -0.5, 1.17, 0.0,
                -0.05, 0.5, -1.17, 0.0, 0.05, 0.5, -1.17, 0.0]
PART_OF = {"abad": [0, 3, 6, 9], "thigh": [1, 4, 7, 10], "calf": [2, 5, 8, 11]}

latest = {"q_hw": None, "t": 0.0}
lock = threading.Lock()


def on_state(state):
    motors = state.motor_state()
    q_hw = [motors[i].q() for i in range(16)]
    with lock:
        latest["q_hw"], latest["t"] = q_hw, time.monotonic()


def make_cmd(q_logical, kp, kd):
    """논리 관절각(12, 오프셋 없음) → LowerCmd (하드웨어 슬롯에 오프셋을 더해서)."""
    cmd = dds.LowerCmd()
    for i in range(NUM_MOTORS):
        hw = ABS2HW[i]
        cmd[hw].mode(0)
        cmd[hw].q(float(q_logical[i] + MOTOR_OFFSET[hw]))
        cmd[hw].dq(0.0)
        cmd[hw].tau(0.0)
        cmd[hw].kp(float(kp))
        cmd[hw].kd(float(kd))
    return cmd


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="dds_config.yaml", help="SDK 의 dds_config.yaml 경로")
    ap.add_argument("--joints", default="thigh", choices=("abad", "thigh", "calf", "all"))
    ap.add_argument("--amp", type=float, default=0.1)
    ap.add_argument("--freq", type=float, default=0.9)
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--kd", type=float, default=1.2)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--robot-ip", default="192.168.5.2")
    a = ap.parse_args()

    # 1) 주 제어기 검사
    try:
        with socket.create_connection((a.robot_ip, 50051), timeout=1.0):
            print("주 제어기(gRPC 50051)가 아직 살아 있습니다. kill_robot 을 먼저 실행하세요.")
            return 2
    except OSError:
        pass
    if input("로봇이 엎드려 있고 주변이 비어 있으면 yes 입력: ").strip().lower() != "yes":
        return 1

    # 2) DDS 준비: 구독 → writer → 첫 상태 대기 → 탐색 1초
    mw = dds.PyDDSMiddleware(a.config)
    mw.subscribeLowerState("rt/lower/state", on_state)
    mw.createLowerCmdWriter("rt/lower/cmd", {"reliability": "reliable", "history_kind": "keep_last",
                                             "history_depth": 1, "durability": "volatile"})
    t0 = time.monotonic()
    while latest["q_hw"] is None:
        if time.monotonic() - t0 > 10.0:
            print("rt/lower/state 수신 없음: 유선 연결과 CYCLONEDDS_URI 를 확인하세요")
            return 1
        time.sleep(0.01)
    time.sleep(1.0)
    with lock:
        q0 = [latest["q_hw"][ABS2HW[i]] - MOTOR_OFFSET[ABS2HW[i]] for i in range(NUM_MOTORS)]
    print("초기 논리각:", [round(x, 3) for x in q0])
    active = list(range(12)) if a.joints == "all" else PART_OF[a.joints]

    # 3) 200Hz 루프: 댐핑 1초 → 사인 (진폭 램프 1초)
    dt, tick = 0.005, 0
    start = time.monotonic()
    try:
        while True:
            t = tick * dt
            if t >= 1.0 + a.seconds:
                break
            with lock:
                age = time.monotonic() - latest["t"]
            if age > 0.2:
                print(f"상태가 {age * 1e3:.0f} ms 동안 안 옴 → 중단")
                break
            if t < 1.0:
                cmd = make_cmd(q0, 0.0, 3.0)                     # 댐핑
            else:
                tt = t - 1.0
                env = min(1.0, tt)
                q = list(q0)
                for j in active:
                    q[j] = q0[j] + a.amp * env * math.sin(2.0 * math.pi * a.freq * tt)
                cmd = make_cmd(q, a.kp, a.kd)
            mw.publishLowerCmd(cmd)
            tick += 1
            sleep = start + tick * dt - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nCtrl+C → 댐핑으로 종료")
    finally:                                                     # 4) 반드시 댐핑 1초
        with lock:
            q_now = [latest["q_hw"][ABS2HW[i]] - MOTOR_OFFSET[ABS2HW[i]] for i in range(NUM_MOTORS)]
        for _ in range(200):
            mw.publishLowerCmd(make_cmd(q_now, 0.0, 3.0))
            time.sleep(dt)
        print("댐핑 전송 완료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
