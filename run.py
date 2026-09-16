#!/usr/bin/env python3
"""Rover 저수준 관절 제어 실행기 (MuJoCo 시뮬레이션 / 실기 DDS 공용).

같은 프로그램(standup, sine, hold, damp)과 같은 안전 가드를 두 백엔드에 그대로 적용한다.

  시뮬(thor):  MUJOCO_GL=egl python run.py --backend mujoco --program standup --duration 3 --snapshot
  실기(DDS) :  python run.py --backend dds --program hold --duration 5
              (kill_robot 으로 주 제어기를 끈 뒤, 로봇을 지지한 상태에서 먼저 시험)

더 짧은 사용 예는 examples/ 를 본다 (lowlevel.runtime.Session 사용).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from lowlevel import programs                                                    # noqa: E402
from lowlevel.common import KD_SINE, KP_SINE, fmt, joint_mask                    # noqa: E402
from lowlevel.dataset import update_meta, utc_now, write_meta                    # noqa: E402
from lowlevel.runtime import (Interrupts, Pacer, TraceLog, confirm_real_robot,   # noqa: E402
                              safe_exit)
from lowlevel.safety import Guard, SafetyAbort                                   # noqa: E402


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=("mujoco", "dds", "isaac"), required=True,
                   help="mujoco=MuJoCo 시뮬, dds=실기, isaac=Isaac Sim (실험적, Python 3.11 + Isaac Sim 5.x)")
    p.add_argument("--program", choices=("standup", "sine", "hold", "damp"), required=True)
    p.add_argument("--duration", type=float, default=5.0,
                   help="본동작 시간(s). standup 은 선 뒤 유지하는 시간, sine/hold/damp 는 동작 시간")
    p.add_argument("--dt", type=float, default=0.005, help="제어 주기(s). 기본 5ms=200Hz")
    p.add_argument("--out", type=Path, default=None, help="기록 폴더 (기본 runs/<UTC>-<backend>-<program>)")
    p.add_argument("--exit", choices=("crouch", "damp"), default=None,
                   help="종료 방식. 기본: standup 은 crouch(엎드린 뒤 댐핑), 나머지는 damp")
    p.add_argument("--tag", action="append", default=[], help="기록 meta.json 의 태그 (여러 번 가능)")
    p.add_argument("--note", default="", help="기록 meta.json 의 메모")
    g = p.add_argument_group("안전 가드")
    g.add_argument("--slew", type=float, default=2.0, help="목표각 변화율 한계 rad/s")
    g.add_argument("--state-timeout", type=float, default=0.2, help="상태 수신 워치독 s (sim2real 0.2)")
    g.add_argument("--fall-threshold", type=float, default=-0.866,
                   help="몸체 좌표계 중력 z 가 이 값보다 크면 넘어짐 (-0.866 = 30도)")
    g.add_argument("--no-fall-check", action="store_true")
    s = p.add_argument_group("sine / hold")
    s.add_argument("--amp", type=float, default=0.2, help="sine 진폭 rad (E9 기본 0.2)")
    s.add_argument("--freq", type=float, default=0.9, help="sine 주파수 Hz (E9 는 1/(500*2.2ms)=0.91)")
    s.add_argument("--joints", default="all", help="sine 대상: all | abad,thigh,calf | FL,FR,RL,RR | 0,1,2")
    s.add_argument("--kp", type=float, default=KP_SINE, help="sine/hold kp (E9 기본 30)")
    s.add_argument("--kd", type=float, default=KD_SINE, help="sine/hold kd (E9 기본 1.2)")
    m = p.add_argument_group("mujoco")
    m.add_argument("--xml", type=Path, default=None, help="dobot_quad.xml 경로")
    m.add_argument("--urdf", type=Path, default=None, help="(isaac) dobot_quad_ros.urdf 경로")
    m.add_argument("--fixed-base", action="store_true", help="(mujoco/isaac) 몸통을 공중에 고정 (지지된 로봇 시험)")
    m.add_argument("--start", choices=("lying", "standing"), default="lying")
    m.add_argument("--physics-dt", type=float, default=0.001)
    m.add_argument("--seed", type=int, default=0)
    m.add_argument("--realtime", action="store_true", help="시뮬도 벽시계에 맞춰 실행")
    m.add_argument("--viewer", action="store_true", help="MuJoCo 뷰어 창을 열어 지켜본다 (realtime 자동 적용)")
    m.add_argument("--snapshot", action="store_true", help="프로그램 종료·최종 시점 이미지 저장(EGL)")
    m.add_argument("--render-every", type=int, default=0, help="N 틱마다 frames/ 에 이미지 저장")
    d = p.add_argument_group("dds")
    d.add_argument("--dds-config", type=Path, default=ROOT / "dds_config.yaml")
    d.add_argument("--state-wait", type=float, default=10.0, help="첫 상태 수신 대기 s")
    d.add_argument("--robot-ip", default="192.168.5.2")
    d.add_argument("--force", action="store_true", help="주 제어기(gRPC 50051) 응답 검사를 무시")
    d.add_argument("--yes", action="store_true", help="확인 프롬프트 생략")
    return p.parse_args(argv)


def make_backend(args, out: Path):
    if args.backend == "mujoco":
        from lowlevel.backend_mujoco import MujocoBackend
        return MujocoBackend(xml_path=args.xml, dt=args.dt, physics_dt=args.physics_dt,
                             start=args.start, seed=args.seed, fixed_base=args.fixed_base,
                             frames_dir=(out / "frames") if args.render_every else None,
                             render_every=args.render_every, viewer=args.viewer)
    if args.backend == "isaac":
        from lowlevel.backend_isaac import IsaacBackend
        return IsaacBackend(urdf_path=args.urdf, dt=args.dt, physics_dt=args.physics_dt, start=args.start,
                            fixed_base=args.fixed_base, headless=not args.viewer)
    from lowlevel.backend_dds import DdsBackend
    return DdsBackend(args.dds_config, timeout=args.state_wait)


def make_program(args, state0):
    if args.program == "standup":
        return programs.StandUpProgram(state0, hold_s=args.duration)
    if args.program == "sine":
        return programs.SineProgram(state0, duration=args.duration, amp=args.amp, freq=args.freq,
                                    mask=joint_mask(args.joints), kp=args.kp, kd=args.kd)
    if args.program == "hold":
        return programs.HoldProgram(state0, duration=args.duration, kp=args.kp, kd=args.kd)
    return programs.DampProgram(duration=args.duration)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.backend == "dds":
        args.realtime = True
        confirm_real_robot(args.robot_ip, force=args.force, yes=args.yes)
    if args.viewer:
        args.realtime = True
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = args.out or (ROOT / "runs" / f"{stamp}-{args.backend}-{args.program}")
    out.mkdir(parents=True, exist_ok=True)
    write_meta(out, source=args.backend, program=args.program, dt=args.dt, tags=args.tag, note=args.note,
               params={k: getattr(args, k) for k in ("duration", "amp", "freq", "joints", "kp", "kd", "start", "exit")})
    try:
        io = make_backend(args, out)
        print(f"[준비] backend={io.name} program={args.program} dt={args.dt * 1e3:.1f}ms "
              f"realtime={args.realtime} out={out}")
        state = io.wait_ready()
    except Exception as exc:                       # 백엔드 준비 실패: 기록에 남기고 그대로 올린다
        update_meta(out, status=f"error:{type(exc).__name__}:{exc}"[:200], ended_at=utc_now())
        raise
    print(f"[준비] 첫 상태: q={fmt(state.q)} grav_z={state.gravity_body()[2]:+.3f}")
    if state.motor_mode is not None:
        print(f"[준비] 모터 mode(논리 12개)={state.motor_mode.tolist()} 온도={state.extra.get('temp_hw')}")

    guard = Guard(args.dt, slew=args.slew, state_timeout=args.state_timeout,
                  fall_threshold=args.fall_threshold, check_fall=not args.no_fall_check)
    program = make_program(args, state)
    pacer = Pacer(args.dt, enabled=args.realtime)
    log = TraceLog(out / "trace.csv")
    exit_mode = args.exit or ("crouch" if args.program == "standup" else "damp")
    reason = "completed"
    per_sec = max(1, int(round(1.0 / args.dt)))
    hold_err = []
    tick = 0
    wall0 = time.monotonic()
    interrupts = Interrupts()
    interrupts.install()
    try:
        while True:
            pacer.wait(tick)
            t = tick * args.dt
            state = io.read()
            guard.check(state)
            cmd = program.step(t, state)
            if cmd is None:
                break
            cmd = guard.limit(cmd, state)
            io.send(cmd)
            log.row(t, program.phase, state, cmd)
            if program.phase == "hold":
                hold_err.append(float(np.max(np.abs(cmd.q - state.q))))
            if tick % per_sec == 0:
                base_z = state.extra.get("base_pos", (np.nan,) * 3)[2]
                print(f"t={t:6.2f}s phase={program.phase:<7} grav_z={state.gravity_body()[2]:+.3f} "
                      f"base_z={base_z:.3f} |q_des-q|max={float(np.max(np.abs(cmd.q - state.q))):.3f} "
                      f"|tau|max={float(np.max(np.abs(state.tau_est))):.1f} age={state.age * 1e3:.0f}ms")
            tick += 1
    except KeyboardInterrupt:
        reason = "keyboard_interrupt"
    except SafetyAbort as exc:
        reason = f"safety:{exc}"
        exit_mode = "damp"
    interrupts.during_exit = True          # 이 시점부터 추가 신호는 예외가 아니라 플래그
    program_t = tick * args.dt
    print(f"[프로그램 종료] 사유={reason} ticks={tick} t={program_t:.2f}s")
    if args.snapshot:
        io.snapshot(out / "snapshot_program_end.jpg")
    exit_ticks = safe_exit(io, guard, log, args.dt, exit_mode, args.realtime, program_t, interrupts)
    if args.snapshot:
        io.snapshot(out / "snapshot_final.jpg")
    final = io.read()
    log.close()
    io.close()

    summary = {
        "backend": io.name, "program": args.program, "dt_s": args.dt, "realtime": args.realtime,
        "ticks": tick, "exit_ticks": exit_ticks, "program_seconds": program_t,
        "wall_seconds": time.monotonic() - wall0, "exit_reason": reason, "exit_mode": exit_mode,
        "deadline_misses": pacer.misses, "max_late_ms": pacer.max_late * 1e3,
        "signals_received": interrupts.count,
        "hold_tracking_max_err_rad": ({"mean": float(np.mean(hold_err)), "max": float(np.max(hold_err))}
                                      if hold_err else None),
        "final": {"gravity_z": float(final.gravity_body()[2]),
                  "base_z": float(final.extra["base_pos"][2]) if "base_pos" in final.extra else None,
                  "q": [round(float(x), 4) for x in final.q]},
        "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
    }
    if io.name == "dds":
        summary["dds"] = {"states_received": io.count, "parse_errors": io.parse_errors,
                          "last_mode_hw": np.asarray(final.extra.get("mode_hw", [])).tolist(),
                          "last_temp_hw": np.asarray(final.extra.get("temp_hw", [])).tolist()}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    update_meta(out, status=reason, ended_at=utc_now(), ticks=tick, program_seconds=program_t, exit_mode=exit_mode,
                deadline_misses=pacer.misses, max_late_ms=pacer.max_late * 1e3, signals_received=interrupts.count)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if reason == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
