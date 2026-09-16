#!/usr/bin/env python3
"""실기와 MuJoCo 쌍둥이를 같은 명령으로 동시에 움직이며 관절각·토크를 실시간 차트로 비교한다.

실기(또는 --backend mujoco 로 시뮬끼리)를 프로그램(sine/hold/damp/standup)으로 제어하면서, 가드를 거친 **같은
JointCmd** 를 MuJoCo 쌍둥이에도 보낸다. 쌍둥이는 실기의 첫 관절각으로 초기화한다. --twin fixed(기본)는 몸통을
공중에 고정한 모델(거치대에 지지된 로봇), --twin ground 는 바닥에 놓고 --base-z 높이에서 안착시킨 모델이다.
브라우저 차트: http://127.0.0.1:<port>/   (thor 에서 실행하면 PC 에서  ssh -L 8767:127.0.0.1:8090 thor  로 터널)

    docker/sdk.sh bash -c "source docker/dds_env.sh && python3 tools/live_compare.py --backend dds --program sine --joints thigh --amp 0.1 --seconds 10"
    python tools/live_compare.py --backend mujoco --program sine --seconds 10          # 로봇 없이 (시뮬 vs 시뮬)

끝나면 runs/live-<UTC>/ 에 real.csv, sim.csv, summary.json 을 남기고 --linger 초 동안 차트를 계속 서비스한다.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lowlevel import programs                                              # noqa: E402
from lowlevel.backend_mujoco import MujocoBackend                          # noqa: E402
from lowlevel.common import JOINT_SHORT, KD_SINE, KP_SINE, JointCmd, joint_mask   # noqa: E402
from lowlevel.runtime import Session, make_backend                         # noqa: E402

HTML = (ROOT / "tools" / "live_compare.html").read_text(encoding="utf-8")


def lag_ms(reference, signal, dt, max_lag_s=0.5):
    a, b = reference - reference.mean(), signal - signal.mean()
    if a.std() < 1e-6 or b.std() < 1e-6 or len(a) < 20:
        return None
    max_lag = min(int(max_lag_s / dt), len(a) - 10)
    best, best_c = 0, -np.inf
    for lag in range(0, max_lag + 1):
        c = float(np.dot(a[:len(a) - lag], b[lag:]))
        if c > best_c:
            best, best_c = lag, c
    return best * dt * 1e3


class Buffer:
    """링버퍼: 틱마다 실기·쌍둥이 상태와 명령을 쌓고, 차트용 JSON 과 통계를 만든다."""

    def __init__(self, n: int, dt: float):
        self.n, self.dt = n, dt
        self.lock = threading.Lock()
        self.t = np.zeros(n)
        self.q_real, self.q_sim, self.q_des = (np.zeros((n, 12)) for _ in range(3))
        self.tau_real, self.tau_sim = np.zeros((n, 12)), np.zeros((n, 12))
        self.grav_real, self.base_sim = np.zeros(n), np.zeros(n)
        self.count = 0
        self.phase, self.running, self.reason = "init", True, None

    def push(self, t, real, sim, cmd, phase):
        with self.lock:
            i = self.count % self.n
            self.t[i], self.phase = t, phase
            self.q_real[i], self.q_sim[i], self.q_des[i] = real.q, sim.q, cmd.q
            self.tau_real[i], self.tau_sim[i] = real.tau_est, sim.tau_est
            self.grav_real[i], self.base_sim[i] = real.gravity_body()[2], sim.extra.get("base_pos", (0, 0, 0))[2]
            self.count += 1

    def _window(self, start):
        idx = [(k % self.n) for k in range(start, self.count)]
        return idx

    def since(self, seq, joints):
        with self.lock:
            start = max(seq, self.count - self.n, 0)
            idx = self._window(start)
            out = {"seq": self.count, "t": self.t[idx].round(4).tolist(), "phase": self.phase,
                   "running": self.running, "reason": self.reason, "series": {}}
            for j in joints:
                out["series"][JOINT_SHORT[j]] = {
                    "des": self.q_des[idx, j].round(4).tolist(), "real": self.q_real[idx, j].round(4).tolist(),
                    "sim": self.q_sim[idx, j].round(4).tolist(),
                    "tau_real": self.tau_real[idx, j].round(3).tolist(), "tau_sim": self.tau_sim[idx, j].round(3).tolist()}
            return out

    def stats(self, joints, window_s):
        with self.lock:
            w = min(int(window_s / self.dt), self.count, self.n)
            if w < 10:
                return {"window_s": 0, "joints": {}}
            idx = self._window(self.count - w)
            res = {"window_s": w * self.dt, "ticks": self.count, "joints": {}}
            for j in joints:
                d, r, s = self.q_des[idx, j], self.q_real[idx, j], self.q_sim[idx, j]
                p2p = lambda x: float(x.max() - x.min())   # noqa: E731
                res["joints"][JOINT_SHORT[j]] = {
                    "rmse_real_sim": float(np.sqrt(np.mean((r - s) ** 2))),
                    "rmse_real_des": float(np.sqrt(np.mean((r - d) ** 2))),
                    "rmse_sim_des": float(np.sqrt(np.mean((s - d) ** 2))),
                    "amp_ratio_real": p2p(r) / p2p(d) if p2p(d) > 1e-3 else None,
                    "amp_ratio_sim": p2p(s) / p2p(d) if p2p(d) > 1e-3 else None,
                    "lag_real_ms": lag_ms(d, r, self.dt), "lag_sim_ms": lag_ms(d, s, self.dt),
                    "tau_peak_real": float(np.abs(self.tau_real[idx, j]).max()),
                    "tau_peak_sim": float(np.abs(self.tau_sim[idx, j]).max())}
            return res

    def dump(self, out: Path):
        with self.lock:
            idx = self._window(max(0, self.count - self.n))
            with open(out / "real.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"q_{j}" for j in JOINT_SHORT] + [f"qdes_{j}" for j in JOINT_SHORT]
                           + [f"tau_{j}" for j in JOINT_SHORT] + ["grav_z"])
                for i in idx:
                    w.writerow([f"{self.t[i]:.4f}"] + [f"{x:.5f}" for x in self.q_real[i]] + [f"{x:.5f}" for x in self.q_des[i]]
                               + [f"{x:.3f}" for x in self.tau_real[i]] + [f"{self.grav_real[i]:.4f}"])
            with open(out / "sim.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"q_{j}" for j in JOINT_SHORT] + [f"tau_{j}" for j in JOINT_SHORT] + ["base_z"])
                for i in idx:
                    w.writerow([f"{self.t[i]:.4f}"] + [f"{x:.5f}" for x in self.q_sim[i]] + [f"{x:.3f}" for x in self.tau_sim[i]]
                               + [f"{self.base_sim[i]:.4f}"])


def serve(buffer: Buffer, bind: str, port: int, default_joints):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            joints = [int(x) for x in qs.get("joints", [",".join(map(str, default_joints))])[0].split(",") if x != ""]
            joints = [j for j in joints if 0 <= j < 12] or list(default_joints)
            if u.path == "/data":
                self._json(buffer.since(int(qs.get("since", ["0"])[0]), joints))
            elif u.path == "/stats":
                self._json(buffer.stats(joints, float(qs.get("window", ["10"])[0])))
            elif u.path == "/meta":
                self._json({"joints": JOINT_SHORT, "default": list(default_joints), "dt": buffer.dt})
            else:
                body = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

    server = ThreadingHTTPServer((bind, port), Handler)
    threading.Thread(target=server.serve_forever, name="live-compare-http", daemon=True).start()
    return server


def make_program(a, state0):
    if a.program == "sine":
        return programs.SineProgram(state0, duration=a.seconds, amp=a.amp, freq=a.freq, mask=joint_mask(a.joints), kp=a.kp, kd=a.kd)
    if a.program == "hold":
        return programs.HoldProgram(state0, duration=a.seconds, kp=a.kp, kd=a.kd)
    if a.program == "standup":
        return programs.StandUpProgram(state0, hold_s=a.seconds)
    return programs.DampProgram(duration=a.seconds)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("dds", "mujoco"), default="mujoco", help="'실기' 쪽 백엔드 (mujoco 면 시뮬 vs 시뮬)")
    ap.add_argument("--program", choices=("sine", "hold", "damp", "standup"), default="sine")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--joints", default="thigh", help="sine 대상 관절 (차트 기본 관절도 여기서 정한다)")
    ap.add_argument("--amp", type=float, default=0.1)
    ap.add_argument("--freq", type=float, default=0.9)
    ap.add_argument("--kp", type=float, default=KP_SINE)
    ap.add_argument("--kd", type=float, default=KD_SINE)
    ap.add_argument("--twin", choices=("fixed", "ground"), default="fixed",
                    help="쌍둥이 모델: fixed=몸통 공중 고정(거치대에 지지된 로봇), ground=바닥에 놓고 --base-z 에서 안착")
    ap.add_argument("--base-z", type=float, default=0.22, help="ground 모드 초기 몸통 높이 m")
    ap.add_argument("--place", type=float, default=1.0, help="쌍둥이 안착 시간 s")
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--linger", type=float, default=30.0, help="끝난 뒤 차트를 계속 서비스하는 시간 s")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    out = a.out or (ROOT / "runs" / f"live-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{a.backend}-{a.program}")
    out.mkdir(parents=True, exist_ok=True)
    mask = joint_mask(a.joints)
    default_joints = [int(j) for j in np.flatnonzero(mask)][:4] or [1, 4]

    io_real = make_backend(a.backend, yes=a.yes, dt=a.dt)
    state0 = io_real.wait_ready()
    io_sim = MujocoBackend(dt=a.dt, init_q=state0.q, init_base_z=a.base_z, fixed_base=(a.twin == "fixed"))
    for _ in range(int(round(a.place / a.dt))):                      # 쌍둥이 안착 (실기는 아직 명령 없음)
        io_sim.send(JointCmd.pd(state0.q, 30.0, 1.2))
    sim0 = io_sim.read()
    print(f"[쌍둥이] {a.twin}: 안착 후 몸통 {sim0.extra['base_pos'][2]:.3f} m, 관절 변화 최대 {np.abs(sim0.q - state0.q).max():.3f} rad")

    buffer = Buffer(n=int(120 / a.dt), dt=a.dt)
    server = serve(buffer, a.bind, a.port, default_joints)
    print(f"[차트] http://{a.bind}:{a.port}/   (PC 에서: ssh -L 8767:{a.bind}:{a.port} thor → http://127.0.0.1:8767/)")

    prog = make_program(a, state0)
    exit_mode = "crouch" if a.program == "standup" else "damp"
    meta = {"program": a.program, "tags": ["live-compare"], "twin": a.twin,
            "params": {"duration": a.seconds, "amp": a.amp, "freq": a.freq, "joints": a.joints, "kp": a.kp, "kd": a.kd}}
    with Session(io_real, dt=a.dt, realtime=True, exit_mode=exit_mode, log_path=out / "trace.csv", meta=meta) as s:
        for t, state in s.run():
            cmd = prog.step(t, state)
            if cmd is None:
                break
            s.phase = prog.phase
            cmd = s.send(cmd)                       # 실기: 가드 제한 후 전송
            sim_state = io_sim.read()               # 쌍둥이: 실기와 같은 시점의 상태를 읽고
            io_sim.send(cmd)                        #          같은 명령을 보낸다
            buffer.push(t, state, sim_state, cmd, prog.phase)
            if s.tick % 200 == 0:
                j = default_joints[0]
                print(f"t={t:5.1f}s {prog.phase:<7} {JOINT_SHORT[j]} des={cmd.q[j]:+.3f} real={state.q[j]:+.3f} "
                      f"sim={sim_state.q[j]:+.3f} | tau real={state.tau_est[j]:+.2f} sim={sim_state.tau_est[j]:+.2f}")
    buffer.running, buffer.reason = False, s.reason
    io_sim.close()
    buffer.dump(out)
    summary = {"backend": a.backend, "program": a.program, "seconds": a.seconds, "reason": s.reason,
               "ticks": buffer.count, "deadline_misses": s.pacer.misses, "max_late_ms": s.pacer.max_late * 1e3,
               "twin": a.twin,
               "sim_place": {"base_z_init": a.base_z, "base_z_after": float(sim0.extra["base_pos"][2]),
                             "drift_max_rad": float(np.abs(sim0.q - state0.q).max())},
               "stats_full_run": buffer.stats(default_joints, window_s=1e9)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary["stats_full_run"], indent=1, ensure_ascii=False))
    print(f"[기록] {out}  (차트를 {a.linger:.0f}초 더 서비스합니다. Ctrl+C 로 종료)")
    try:
        time.sleep(a.linger)
    except KeyboardInterrupt:
        pass
    server.shutdown()
    return 0 if s.reason == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
