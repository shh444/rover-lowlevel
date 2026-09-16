"""실행 보조 도구: 벽시계 페이싱, 신호(Ctrl+C/SIGTERM/SIGHUP) 처리, CSV 기록, 안전 종료, 실기 확인 절차,
그리고 이것들을 한 번에 묶은 Session. run.py 와 examples/ 가 함께 쓴다.

    io = make_backend("mujoco")                     # 또는 make_backend("dds", yes=True)
    state0 = io.wait_ready()
    with Session(io, dt=0.005, realtime=False) as s:
        for t, state in s.run(seconds=3.0):
            s.send(JointCmd.damping(3.0, state.q))
    # with 블록을 빠져나가면(정상 종료, Ctrl+C, 안전 가드 이상 모두) 반드시 안전 종료(엎드림/댐핑)가 실행된다.
"""
from __future__ import annotations

import csv
import os
import signal
import socket
import sys
import time
from pathlib import Path

import numpy as np

from . import programs
from .common import JOINT_SHORT, KD_DAMP, KD_STAND, JointCmd, cosine_interp
from .safety import Guard, SafetyAbort

ROOT = Path(__file__).resolve().parents[1]

CHECKLIST = """
=== 실기(DDS) 저수준 제어 전 확인 (SDK low_level 문서 E9 경고) ===
 1. kill_robot 으로 주 제어기를 종료했다:
      python3 high_level/python/examples/kill_robot.py 192.168.5.2:50051
 2. standup 이면 로봇이 평평한 바닥에 엎드려 있고, hold/sine/damp 이면 다리를 띄운 채 지지되어 있다
 3. 주변에 사람·물건이 없고, Ctrl+C 와 전원 차단으로 바로 멈출 수 있다
 4. 유선 연결(192.168.5.x)이며 CYCLONEDDS_URI 가 올바른 인터페이스의 cyclonedds.xml 을 가리킨다
"""


class Interrupts:
    """Ctrl+C(SIGINT), SIGTERM, SIGHUP(SSH 끊김)을 한 곳에서 받는다.

    제어 루프 중 첫 신호는 KeyboardInterrupt 로 루프를 끝내 안전 종료로 넘긴다.
    안전 종료 중의 추가 신호는 예외를 내지 않고 횟수만 센다(엎드리기를 생략하고 댐핑으로 감).
    안전 종료가 끝나기 전에 프로세스가 죽으면 마지막 명령(예: kp 60 유지)이 로봇에 남기 때문이다.
    """

    def __init__(self):
        self.count = 0
        self.during_exit = False

    def install(self):
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            if hasattr(signal, name):
                signal.signal(getattr(signal, name), self._handler)

    def _handler(self, signum, frame):
        self.count += 1
        if self.during_exit:
            print(f"\n[신호 {signum}] 안전 종료 진행 중입니다. 엎드리기를 생략하고 댐핑으로 갑니다.")
            return
        # 첫 신호: 여기서 바로 during_exit 를 켜야 거의 동시에 오는 두 번째 신호(예: timeout 은
        # 프로세스와 그룹에 한 번씩 보낸다)가 안전 종료 코드 밖에서 다시 예외를 내지 않는다.
        self.during_exit = True
        raise KeyboardInterrupt


class Pacer:
    """벽시계 페이싱. enabled=False 면 최대 속도로 돈다(시뮬 시험)."""

    def __init__(self, dt: float, enabled: bool = True):
        self.dt, self.enabled = float(dt), bool(enabled)
        self.t0 = None
        self.misses = 0
        self.max_late = 0.0

    def wait(self, tick: int) -> None:
        if not self.enabled:
            return
        if self.t0 is None:
            self.t0 = time.monotonic()
        target = self.t0 + tick * self.dt
        now = time.monotonic()
        if now < target:
            time.sleep(target - now)
        else:
            late = now - target
            self.max_late = max(self.max_late, late)
            if late > self.dt:
                self.misses += 1


class TraceLog:
    """틱마다 상태·명령을 CSV 로 남긴다 (tools/trace_stats.py 로 요약)."""

    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "phase", "age_ms", "grav_z", "base_z"]
                        + [f"q_{n}" for n in JOINT_SHORT] + [f"dq_{n}" for n in JOINT_SHORT]
                        + [f"qdes_{n}" for n in JOINT_SHORT] + [f"tau_{n}" for n in JOINT_SHORT]
                        + ["kp", "kd"])
        self.rows = 0

    def row(self, t, phase, state, cmd) -> None:
        base_z = state.extra.get("base_pos", (np.nan, np.nan, np.nan))[2]
        self.w.writerow([f"{t:.4f}", phase, f"{state.age * 1e3:.1f}",
                         f"{state.gravity_body()[2]:.4f}", f"{base_z:.4f}"]
                        + [f"{x:.5f}" for x in state.q] + [f"{x:.4f}" for x in state.dq]
                        + [f"{x:.5f}" for x in cmd.q] + [f"{x:.3f}" for x in state.tau_est]
                        + [f"{cmd.kp[0]:.2f}", f"{cmd.kd[0]:.3f}"])
        self.rows += 1

    def close(self) -> None:
        self.f.close()


def main_controller_alive(robot_ip: str = "192.168.5.2", port: int = 50051, timeout: float = 1.0) -> bool:
    """주 제어기의 gRPC 포트가 열려 있으면 True (열려 있으면 저수준 모터 명령을 보내면 안 된다)."""
    try:
        with socket.create_connection((robot_ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def confirm_real_robot(robot_ip: str = "192.168.5.2", force: bool = False, yes: bool = False) -> None:
    """실기 명령 전 게이트: 주 제어기 응답 검사 + 체크리스트 확인. 거부 시 프로세스를 끝낸다."""
    if not force:
        if main_controller_alive(robot_ip):
            print(f"[중단] {robot_ip}:50051 (주 제어기 gRPC) 가 아직 응답합니다. "
                  "kill_robot 을 먼저 실행하세요. (무시하려면 --force)")
            sys.exit(2)
        print(f"[확인] {robot_ip}:50051 응답 없음 → 주 제어기가 종료된 것으로 간주")
    print(CHECKLIST)
    if not yes:
        try:
            ans = input("모두 확인했으면 yes 입력: ")
        except EOFError:
            ans = ""
        if ans.strip().lower() != "yes":
            print("중단")
            sys.exit(1)


def make_backend(name: str, *, read_only: bool = False, yes: bool = False, force: bool = False,
                 robot_ip: str | None = None, dds_config=None, dt: float = 0.005, **mujoco_kw):
    """예제용 백엔드 생성. "mujoco" 는 바로, "dds" 는 실기 게이트(주 제어기 검사·체크리스트)를 거친다.
    read_only=True 면 DDS writer 를 만들지 않아 명령을 보낼 수 없다(상태 모니터용, 게이트 생략).
    robot_ip 기본값은 환경변수 ROVER_ROBOT_IP, 없으면 192.168.5.2 (가짜 SDK 시험 때 127.0.0.1 로 바꾼다)."""
    robot_ip = robot_ip or os.environ.get("ROVER_ROBOT_IP", "192.168.5.2")
    if name == "mujoco":
        from .backend_mujoco import MujocoBackend
        return MujocoBackend(dt=dt, **mujoco_kw)
    if name == "dds":
        from .backend_dds import DdsBackend
        if not read_only:
            confirm_real_robot(robot_ip, force=force, yes=yes)
        return DdsBackend(dds_config or ROOT / "dds_config.yaml", create_writer=not read_only)
    raise ValueError(f"알 수 없는 백엔드: {name}")


def safe_exit(io, guard, log, dt, mode, realtime, t_offset, interrupts, kd_damp=KD_DAMP) -> int:
    """sim2real safe_exit 와 같다: (crouch) 현재 → 엎드림 1.5s kp30 → 댐핑 1s. 상태가 오래됐으면 댐핑만.
    진행 중 추가 Ctrl+C 가 오면 엎드리기를 생략하고 댐핑으로 간다. 댐핑 1초는 어떤 경우에도 끝까지 보낸다."""
    interrupts.during_exit = True
    signals_before = interrupts.count
    state = io.read()
    if mode == "crouch" and state.age > guard.state_timeout:
        print("[안전 종료] 상태가 오래되어 엎드리기를 생략하고 댐핑만 보냅니다")
        mode = "damp"
    print(f"[안전 종료] 방식={mode}")
    pacer = Pacer(dt, enabled=realtime)
    tick = 0
    if mode == "crouch":
        q_from = state.q.copy()
        steps = max(1, int(round(1.5 / dt)))
        for k in range(steps):
            if interrupts.count > signals_before:
                break
            pacer.wait(tick)
            state = io.read()
            target = cosine_interp(q_from, programs.Q_CROUCH_SAFE, k / steps)
            cmd = guard.limit(JointCmd.pd(target, 30.0, KD_STAND), state)
            io.send(cmd)
            if log:
                log.row(t_offset + tick * dt, "exit_crouch", state, cmd)
            tick += 1
    for _ in range(max(1, int(round(1.0 / dt)))):
        pacer.wait(tick)
        state = io.read()
        cmd = guard.limit(JointCmd.damping(kd_damp, state.q), state)
        io.send(cmd)
        if log:
            log.row(t_offset + tick * dt, "exit_damp", state, cmd)
        tick += 1
    return tick


class Session:
    """예제용 제어 세션. 한 틱 = 페이싱 → 상태 읽기 → 가드 검사 → (사용자 코드) → 명령 제한 → 전송.

        with Session(io, dt=0.005, realtime=True, exit_mode="damp") as s:
            for t, state in s.run(seconds=5.0):
                s.send(JointCmd.pd(q_des, 30.0, 1.2))

    with 블록을 어떻게 빠져나가든(정상 종료, break, Ctrl+C, SafetyAbort, 다른 예외) 안전 종료가 실행된다.
    Ctrl+C 와 SafetyAbort 는 정상 처리로 보고 예외를 삼킨다. exit_mode 는 "damp" 또는 "crouch"(엎드린 뒤 댐핑).
    """

    def __init__(self, io, dt: float = 0.005, realtime: bool = True, exit_mode: str = "damp",
                 guard: Guard | None = None, log_path=None, kd_damp: float = KD_DAMP):
        self.io, self.dt, self.realtime = io, float(dt), bool(realtime)
        self.exit_mode, self.kd_damp = exit_mode, float(kd_damp)
        self.guard = guard or Guard(self.dt)
        self.log = TraceLog(log_path) if log_path else None
        self.interrupts = Interrupts()
        self.pacer = Pacer(self.dt, self.realtime)
        self.tick = 0
        self.state = None
        self.phase = "run"
        self.reason = None

    def __enter__(self):
        self.interrupts.install()
        return self

    def run(self, seconds: float | None = None):
        """(t, state) 를 dt 마다 낸다. seconds 가 None 이면 break 할 때까지."""
        while seconds is None or self.tick * self.dt < seconds:
            self.pacer.wait(self.tick)
            self.state = self.io.read()
            self.guard.check(self.state)          # 이상이면 SafetyAbort → __exit__ 에서 댐핑
            yield self.tick * self.dt, self.state
            self.tick += 1

    def send(self, cmd: JointCmd) -> JointCmd:
        if self.state is None:
            raise RuntimeError("run() 루프 안에서 send() 를 호출하세요")
        cmd = self.guard.limit(cmd, self.state)
        self.io.send(cmd)
        if self.log:
            self.log.row(self.tick * self.dt, self.phase, self.state, cmd)
        return cmd

    def __exit__(self, exc_type, exc, tb):
        exit_mode = self.exit_mode
        if exc_type is None:
            self.reason = "completed"
        elif issubclass(exc_type, KeyboardInterrupt):
            self.reason = "keyboard_interrupt"
        elif issubclass(exc_type, SafetyAbort):
            self.reason, exit_mode = f"safety:{exc}", "damp"
        else:
            self.reason, exit_mode = f"error:{exc_type.__name__}:{exc}", "damp"
        self.interrupts.during_exit = True
        print(f"[세션 종료] 사유={self.reason} ticks={self.tick} t={self.tick * self.dt:.2f}s")
        try:
            safe_exit(self.io, self.guard, self.log, self.dt, exit_mode, self.realtime,
                      self.tick * self.dt, self.interrupts, self.kd_damp)
        finally:
            if self.log:
                self.log.close()
            self.io.close()
        print(f"[세션 종료] 안전 종료 완료. 기한 초과 {self.pacer.misses}회, 최대 지연 {self.pacer.max_late * 1e3:.2f} ms")
        return exc_type is not None and issubclass(exc_type, (KeyboardInterrupt, SafetyAbort))
