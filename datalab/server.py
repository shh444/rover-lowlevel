#!/usr/bin/env python3
"""데이터 플랫폼 서버: 시뮬·실기 실행을 시작/정지하고, 기록을 나열·열람·비교하는 웹 대시보드 (표준 라이브러리만).

    python datalab/server.py --root runs --port 8095          # 브라우저: http://127.0.0.1:8095/
    thor 에서 실행하고 PC 에서 보려면:  ssh -L 8768:127.0.0.1:8095 thor  →  http://127.0.0.1:8768/

API
  GET  /api/runs                                  기록 목록 (meta.json 기준, 최신순)
  GET  /api/run/<id>                              meta + summary
  GET  /api/run/<id>/series?joints=1,4&max=2500   차트용 시계열 (실행 중이면 지금까지)
  GET  /api/compare?a=<id>&b=<id>&joints=1,4      비교 지표(RMSE·상관·진폭비·지연·토크) + 두 기록의 시계열
  POST /api/start   {backend, program, duration, amp, freq, joints, kp, kd, start, tags, note, confirm}
                    실행 시작. 실기(dds)는 confirm 이 "REAL" 이어야 하고, run.py 의 주 제어기 검사도 그대로 적용된다
  POST /api/stop    {job}                          안전 종료 (SIGINT → 엎드림/댐핑 후 기록 마감)
  GET  /api/jobs                                   실행 중/최근 작업과 로그 꼬리
  POST /api/run/<id>/meta {tags, note}             태그·메모 수정
실행 명령은 datalab/config.json 으로 바꿀 수 있다 (기본: 시뮬은 이 파이썬으로 run.py, 실기는 docker/sdk.sh 안에서 run.py).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lowlevel.common import JOINT_SHORT                                   # noqa: E402
from lowlevel.dataset import (compare_runs, compare_series, driven_joints, list_runs,   # noqa: E402
                              load_run, read_meta, series, update_meta)

HERE = Path(__file__).resolve().parent
UI_PATH = HERE / "ui.html"
DEFAULT_CONFIG = {
    "commands": {
        "mujoco": [sys.executable, "run.py"],
        "dds": ["docker/sdk.sh", "bash", "-c", "source docker/dds_env.sh > /dev/null && exec python3 run.py \"$@\"", "_"],
    },
    "real_confirm": "REAL",
    "limits": {"amp_max": 0.3, "kp_max": 100.0, "kd_max": 5.0, "duration_max": 600.0},
    "programs": ["damp", "hold", "sine", "standup"],
    "sim_realtime": True,      # 플랫폼에서 시작한 시뮬은 벽시계 페이싱 (라이브 차트·정지가 실기와 같은 흐름)
}


def load_config(path: Path | None) -> dict:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    p = path or (HERE / "config.json")
    if p.exists():
        user = json.loads(p.read_text(encoding="utf-8"))
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


class Jobs:
    """run.py 서브프로세스 관리. 정지는 SIGINT(Windows: CTRL_BREAK) 로 보내 run.py 가 안전 종료하게 한다."""

    def __init__(self, root: Path, cfg: dict):
        self.root, self.cfg = root, cfg
        self.lock = threading.Lock()
        self.jobs: dict[str, dict] = {}

    def start(self, req: dict) -> dict:
        backend = req.get("backend")
        if backend not in self.cfg["commands"]:
            raise ValueError(f"backend 는 {list(self.cfg['commands'])} 중 하나")
        program = req.get("program", "sine")
        if program not in self.cfg["programs"]:
            raise ValueError(f"program 은 {self.cfg['programs']} 중 하나")
        if backend == "dds" and req.get("confirm") != self.cfg["real_confirm"]:
            raise ValueError(f"실기 실행은 confirm='{self.cfg['real_confirm']}' 가 필요합니다 (kill_robot·지지 상태·주변 확인 후)")
        lim = self.cfg["limits"]
        duration = float(req.get("duration", 5.0))
        amp, freq = float(req.get("amp", 0.1)), float(req.get("freq", 0.9))
        kp, kd = float(req.get("kp", 30.0)), float(req.get("kd", 1.2))
        if not (0 < duration <= lim["duration_max"]):
            raise ValueError("duration 범위 밖")
        if not (0 <= amp <= lim["amp_max"]) or not (0 <= kp <= lim["kp_max"]) or not (0 <= kd <= lim["kd_max"]):
            raise ValueError(f"amp≤{lim['amp_max']}, kp≤{lim['kp_max']}, kd≤{lim['kd_max']} 이어야 합니다")
        joints = str(req.get("joints", "thigh")).replace(" ", "") or "thigh"
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        label = "real" if backend == "dds" else "sim"
        run_id = f"{stamp}-{label}-{program}"
        out = self.root / run_id
        out.mkdir(parents=True, exist_ok=True)
        args = ["--backend", backend, "--program", program, "--duration", str(duration), "--out", str(out),
                "--amp", str(amp), "--freq", str(freq), "--joints", joints, "--kp", str(kp), "--kd", str(kd)]
        if backend == "mujoco" and req.get("start") in ("lying", "standing"):
            args += ["--start", req["start"]]
        if backend == "mujoco" and self.cfg.get("sim_realtime", True):
            args += ["--realtime"]
        if backend == "dds":
            args += ["--yes"]
        for tag in req.get("tags", []) or []:
            if tag:
                args += ["--tag", str(tag)]
        args += ["--tag", "datalab"]
        if req.get("note"):
            args += ["--note", str(req["note"])]
        cmd = list(self.cfg["commands"][backend]) + args
        log_path = out / "job.log"
        log = open(log_path, "w", encoding="utf-8")
        kwargs = {"cwd": str(ROOT), "stdout": log, "stderr": subprocess.STDOUT, "stdin": subprocess.DEVNULL}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        proc = subprocess.Popen(cmd, **kwargs)
        job = {"id": run_id, "run_id": run_id, "backend": backend, "label": label, "program": program, "pid": proc.pid,
               "cmd": cmd, "started_at": time.time(), "log_path": str(log_path), "returncode": None,
               "_proc": proc, "_log": log}
        with self.lock:
            self.jobs[run_id] = job
        return self.public(job)

    def stop(self, job_id: str) -> dict:
        with self.lock:
            job = self.jobs.get(job_id)
        if not job:
            raise ValueError("그런 작업이 없습니다")
        proc = job["_proc"]
        if proc.poll() is not None:
            return self.public(job)
        if os.name == "nt":
            proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        job["stop_requested_at"] = time.time()
        return self.public(job)

    def poll(self):
        with self.lock:
            for job in self.jobs.values():
                proc = job["_proc"]
                if job["returncode"] is None and proc.poll() is not None:
                    job["returncode"] = proc.returncode
                    job["ended_at"] = time.time()
                    try:
                        job["_log"].close()
                    except OSError:
                        pass

    def public(self, job: dict, tail: int = 12) -> dict:
        self.poll()
        pub = {k: v for k, v in job.items() if not k.startswith("_")}
        pub["running"] = job["returncode"] is None
        try:
            lines = Path(job["log_path"]).read_text(encoding="utf-8", errors="replace").splitlines()
            pub["log_tail"] = lines[-tail:]
        except OSError:
            pub["log_tail"] = []
        return pub

    def list(self) -> list[dict]:
        self.poll()
        with self.lock:
            jobs = list(self.jobs.values())
        return [self.public(j) for j in sorted(jobs, key=lambda j: j["started_at"], reverse=True)[:20]]


def make_handler(root: Path, jobs: Jobs, cfg: dict):
    ui_html = UI_PATH.read_text(encoding="utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200):
            self._send(code, json.dumps(obj, ensure_ascii=False, allow_nan=True).encode("utf-8"), "application/json; charset=utf-8")

        def _error(self, exc, code: int = 400):
            self._json({"error": str(exc)}, code)

        @staticmethod
        def _joints(qs, default=None):
            raw = qs.get("joints", [""])[0]
            if raw.strip() == "":
                return default
            joints = []
            for tok in raw.split(","):
                tok = tok.strip()
                if tok.isdigit() and 0 <= int(tok) < 12:
                    joints.append(int(tok))
                elif tok in JOINT_SHORT:
                    joints.append(JOINT_SHORT.index(tok))
            return joints or default

        def _run_dir(self, run_id: str) -> Path:
            if "/" in run_id or "\\" in run_id or run_id in ("", ".", ".."):
                raise ValueError("잘못된 id")
            d = root / run_id
            if not (d / "trace.csv").exists():
                raise FileNotFoundError(f"기록 없음: {run_id}")
            return d

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            parts = [p for p in u.path.split("/") if p]
            try:
                if not parts:
                    return self._send(200, ui_html.encode("utf-8"), "text/html; charset=utf-8")
                if parts[:2] == ["api", "runs"]:
                    return self._json({"runs": list_runs(root), "joints": JOINT_SHORT})
                if parts[:2] == ["api", "jobs"]:
                    return self._json({"jobs": jobs.list(), "limits": cfg["limits"], "programs": cfg["programs"],
                                       "backends": list(cfg["commands"]), "real_confirm": cfg["real_confirm"]})
                if parts[:2] == ["api", "compare"]:
                    a = load_run(self._run_dir(qs.get("a", [""])[0]))
                    b = load_run(self._run_dir(qs.get("b", [""])[0]))
                    rel = qs.get("rel", ["0"])[0] in ("1", "true", "yes")
                    metrics = compare_runs(a, b, self._joints(qs), relative=rel)
                    joints = [JOINT_SHORT.index(n) for n in metrics["joints"]]
                    return self._json({"metrics": metrics,
                                       "series": compare_series(a, b, joints, int(qs.get("max", ["2500"])[0]), relative=rel)})
                if parts[:2] == ["api", "run"] and len(parts) >= 3:
                    d = self._run_dir(parts[2])
                    if len(parts) == 3:
                        meta = read_meta(d)
                        summary = None
                        if (d / "summary.json").exists():
                            summary = json.loads((d / "summary.json").read_text(encoding="utf-8"))
                        return self._json({"meta": meta, "summary": summary})
                    if parts[3] == "series":
                        run = load_run(d)
                        joints = self._joints(qs, default=driven_joints(run)[:4] or [1, 4, 7, 10])
                        out = series(run, joints, int(qs.get("max", ["2500"])[0]))
                        out["meta"] = run.meta
                        out["rows"] = len(run.t)
                        return self._json(out)
                return self._error("없는 경로", 404)
            except FileNotFoundError as exc:
                return self._error(exc, 404)
            except Exception as exc:                                   # noqa: BLE001
                return self._error(exc, 400)

        def do_POST(self):
            u = urlparse(self.path)
            parts = [p for p in u.path.split("/") if p]
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(length) or b"{}")
                if parts == ["api", "start"]:
                    return self._json(jobs.start(body))
                if parts == ["api", "stop"]:
                    return self._json(jobs.stop(str(body.get("job", ""))))
                if parts[:2] == ["api", "run"] and len(parts) == 4 and parts[3] == "meta":
                    d = self._run_dir(parts[2])
                    fields = {k: body[k] for k in ("tags", "note") if k in body}
                    return self._json(update_meta(d, **fields))
                return self._error("없는 경로", 404)
            except Exception as exc:                                   # noqa: BLE001
                return self._error(exc, 400)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=ROOT / "runs", help="기록 폴더")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8095)
    ap.add_argument("--config", type=Path, default=None, help="datalab/config.json 대신 쓸 설정")
    a = ap.parse_args()
    cfg = load_config(a.config)
    root = a.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    jobs = Jobs(root, cfg)
    server = ThreadingHTTPServer((a.bind, a.port), make_handler(root, jobs, cfg))
    print(f"[datalab] http://{a.bind}:{a.port}/  root={root}  backends={list(cfg['commands'])}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for job in jobs.list():
            if job["running"]:
                try:
                    jobs.stop(job["id"])
                except Exception:      # noqa: BLE001
                    pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
