#!/usr/bin/env python3
"""데이터 플랫폼 서버: 시뮬·실기·Isaac·다른 로봇(MJCF) 실행을 시작/정지하고, 기록을 나열·열람·비교하는 웹 대시보드.
표준 라이브러리만 쓴다.

    python datalab/server.py --root runs --port 8095          # 브라우저: http://127.0.0.1:8095/
    thor 에서 실행하고 PC 에서 보려면:  ssh -L 8768:127.0.0.1:8095 thor  →  http://127.0.0.1:8768/

API
  GET  /api/runs                                   기록 목록(meta.json) + robots/labels/programs 목록
  GET  /api/run/<id>                               meta + summary
  GET  /api/run/<id>/series?joints=a,b&max=2500&rel=0   차트용 시계열 (실행 중이면 지금까지)
  GET  /api/compare?runs=A,B[,C]&joints=&rel=0     기준 A 와 나머지의 쌍별 비교 지표 + 모든 기록의 시계열
  POST /api/start   {backend, program, duration, amp, freq, joints, kp, kd, start, viewer, fixed_base, robot, xml, tags, note, confirm}
                    backend: mujoco | dds(confirm="REAL") | isaac | mjcf(임의 MJCF, xml·robot 필요)
  POST /api/stop    {job}                           안전 종료 (SIGINT → 엎드림/댐핑 후 기록 마감)
  GET  /api/jobs                                    실행 중/최근 작업과 로그 꼬리, 백엔드·프로그램·한계
  POST /api/run/<id>/meta {tags, note, robot}       메타 수정
실행 명령은 datalab/config.json 으로 바꿀 수 있다 (기본: 시뮬·mjcf 는 서버의 파이썬, 실기는 docker/sdk.sh 컨테이너).
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
from lowlevel.dataset import (compare_many, list_runs, load_run, read_meta, series,   # noqa: E402
                              update_meta)

HERE = Path(__file__).resolve().parent
UI_PATH = HERE / "ui.html"
DEFAULT_CONFIG = {
    "commands": {
        "mujoco": [sys.executable, "run.py"],
        "mjcf": [sys.executable, "tools/mujoco_record.py"],
        "dds": ["docker/sdk.sh", "bash", "-c", "source docker/dds_env.sh > /dev/null && exec python3 run.py \"$@\"", "_"],
    },
    "real_confirm": "REAL",
    "limits": {"amp_max": 0.5, "kp_max": 100.0, "kd_max": 5.0, "duration_max": 600.0},
    "programs": ["damp", "hold", "sine", "standup"],
    "mjcf_programs": ["hold", "sine", "damp"],
    "sim_realtime": True,
    "mjcf_models": {},                       # 이름 → xml 경로 (수집 탭의 MJCF 목록)
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
    cfg["commands"] = {k: v for k, v in cfg["commands"].items() if not k.startswith("_")}
    cfg["mjcf_models"] = {k: v for k, v in cfg.get("mjcf_models", {}).items() if not k.startswith("_")}
    return cfg


class Jobs:
    """run.py / mujoco_record.py 서브프로세스 관리. 정지는 SIGINT(Windows: CTRL_BREAK) 로 보내 안전 종료하게 한다."""

    def __init__(self, root: Path, cfg: dict):
        self.root, self.cfg = root, cfg
        self.lock = threading.Lock()
        self.jobs: dict[str, dict] = {}

    def _num(self, req, key, default, lo, hi):
        v = float(req.get(key, default))
        if not (lo <= v <= hi):
            raise ValueError(f"{key} 는 {lo}~{hi} 범위여야 합니다")
        return v

    def start(self, req: dict) -> dict:
        backend = req.get("backend")
        if backend not in self.cfg["commands"]:
            raise ValueError(f"backend 는 {list(self.cfg['commands'])} 중 하나")
        lim = self.cfg["limits"]
        program = req.get("program", "sine")
        allowed = self.cfg["mjcf_programs"] if backend == "mjcf" else self.cfg["programs"]
        if program not in allowed:
            raise ValueError(f"program 은 {allowed} 중 하나")
        if backend == "dds" and req.get("confirm") != self.cfg["real_confirm"]:
            raise ValueError(f"실기 실행은 confirm='{self.cfg['real_confirm']}' 가 필요합니다 (kill_robot·지지 상태·주변 확인 후)")
        duration = self._num(req, "duration", 5.0, 0.1, lim["duration_max"])
        amp = self._num(req, "amp", 0.1, 0.0, lim["amp_max"])
        freq = self._num(req, "freq", 0.9, 0.0, 10.0)
        kp = self._num(req, "kp", 30.0, 0.0, lim["kp_max"])
        kd = self._num(req, "kd", 1.2, 0.0, lim["kd_max"])
        joints = str(req.get("joints", "thigh")).replace(" ", "") or "all"
        robot = str(req.get("robot") or ("rover" if backend != "mjcf" else "")).strip()
        tags = [str(t) for t in (req.get("tags") or []) if t] + ["datalab"]
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        label = {"dds": "real", "mujoco": "sim", "isaac": "isaac", "mjcf": "sim"}.get(backend, backend)

        if backend == "mjcf":
            xml = req.get("xml") or self.cfg["mjcf_models"].get(robot)
            if not xml:
                raise ValueError("mjcf 는 xml 경로(또는 config 의 mjcf_models 이름)가 필요합니다")
            xml_path = Path(xml)
            if not xml_path.is_absolute():
                xml_path = ROOT / xml_path
            if not xml_path.exists():
                raise FileNotFoundError(f"xml 없음: {xml_path}")
            robot = robot or xml_path.stem
            run_id = f"{stamp}-sim-{program}-{robot}"
            out = self.root / run_id
            args = ["--xml", str(xml_path), "--robot", robot, "--program", program, "--seconds", str(duration),
                    "--amp", str(amp), "--freq", str(freq), "--joints", joints, "--kp", str(kp), "--kd", str(kd),
                    "--out", str(out)]
            if self.cfg.get("sim_realtime", True):
                args += ["--realtime"]
            if req.get("viewer"):
                args += ["--viewer"]
        else:
            run_id = f"{stamp}-{label}-{program}" + (f"-{robot}" if robot and robot != "rover" else "")
            out = self.root / run_id
            args = ["--backend", backend, "--program", program, "--duration", str(duration), "--out", str(out),
                    "--amp", str(amp), "--freq", str(freq), "--joints", joints, "--kp", str(kp), "--kd", str(kd),
                    "--robot", robot or "rover"]
            if backend in ("mujoco", "isaac") and req.get("start") in ("lying", "standing"):
                args += ["--start", req["start"]]
            if backend in ("mujoco", "isaac") and self.cfg.get("sim_realtime", True):
                args += ["--realtime"]
            if backend in ("mujoco", "isaac") and req.get("viewer"):
                args += ["--viewer"]
            if backend in ("mujoco", "isaac") and req.get("fixed_base"):
                args += ["--fixed-base"]
            if backend == "dds":
                args += ["--yes"]
        for tag in tags:
            args += ["--tag", tag]
        if req.get("note"):
            args += ["--note", str(req["note"])]
        out.mkdir(parents=True, exist_ok=True)
        cmd = list(self.cfg["commands"][backend]) + args
        log_path = out / "job.log"
        log = open(log_path, "w", encoding="utf-8")
        kwargs = {"cwd": str(ROOT), "stdout": log, "stderr": subprocess.STDOUT, "stdin": subprocess.DEVNULL}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True
        proc = subprocess.Popen(cmd, **kwargs)
        job = {"id": run_id, "run_id": run_id, "backend": backend, "label": label, "program": program, "robot": robot or "rover",
               "pid": proc.pid, "cmd": cmd, "started_at": time.time(), "log_path": str(log_path), "returncode": None,
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

        def _run_dir(self, run_id: str) -> Path:
            if not run_id or "/" in run_id or "\\" in run_id or run_id in (".", ".."):
                raise ValueError("잘못된 id")
            d = root / run_id
            if not (d / "trace.csv").exists():
                raise FileNotFoundError(f"기록 없음: {run_id}")
            return d

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            parts = [p for p in u.path.split("/") if p]
            joints_q = qs.get("joints", [""])[0] or None
            rel = qs.get("rel", ["0"])[0] in ("1", "true", "yes")
            max_pts = int(qs.get("max", ["2500"])[0])
            try:
                if not parts:
                    return self._send(200, ui_html.encode("utf-8"), "text/html; charset=utf-8")
                if parts[:2] == ["api", "runs"]:
                    runs = list_runs(root)
                    return self._json({"runs": runs,
                                       "robots": sorted({r.get("robot", "?") for r in runs}),
                                       "labels": sorted({r.get("label", "?") for r in runs}),
                                       "programs": sorted({str(r.get("program", "?")) for r in runs})})
                if parts[:2] == ["api", "jobs"]:
                    return self._json({"jobs": jobs.list(), "limits": cfg["limits"], "programs": cfg["programs"],
                                       "mjcf_programs": cfg["mjcf_programs"], "mjcf_models": cfg["mjcf_models"],
                                       "backends": list(cfg["commands"]), "real_confirm": cfg["real_confirm"]})
                if parts[:2] == ["api", "compare"]:
                    ids = [s for s in qs.get("runs", [""])[0].split(",") if s] or [qs.get("a", [""])[0], qs.get("b", [""])[0]]
                    runs = [load_run(self._run_dir(i)) for i in ids]
                    return self._json(compare_many(runs, joints_q, rel, max_pts))
                if parts[:2] == ["api", "run"] and len(parts) >= 3:
                    d = self._run_dir(parts[2])
                    if len(parts) == 3:
                        summary = None
                        if (d / "summary.json").exists():
                            summary = json.loads((d / "summary.json").read_text(encoding="utf-8"))
                        return self._json({"meta": read_meta(d), "summary": summary})
                    if parts[3] == "series":
                        run = load_run(d)
                        out = series(run, joints_q, max_pts, rel)
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
                    fields = {k: body[k] for k in ("tags", "note", "robot") if k in body}
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
