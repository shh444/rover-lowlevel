"""실행 기록(runs/<id>/)을 읽고 비교하는 데이터 계층. 로봇 종류·관절 수에 무관하다.

기록 폴더 구성 (시뮬·실기·Isaac·다른 로봇 모두 같은 형식)
  meta.json     robot(rover, atom_upper …), joints(관절 이름 목록), source(mujoco|dds|isaac → label sim|real|isaac),
                program, params, tags, note, 시작/종료 시각, 상태, 틱 수, 기한 초과
  trace.csv     틱마다 t, phase, age_ms, grav_z, base_z, q_<관절>…, dq_<관절>…, qdes_<관절>…, tau_<관절>…, kp, kd,
                gyro_xyz, acc_xyz  (관절 이름과 개수는 헤더에서 읽는다)
  summary.json  run.py 요약 (선택)
  sim.csv       live_compare 의 쌍둥이 기록 (선택)

비교(compare_runs)는 두 기록을 t=0 기준으로 겹쳐 놓고 B 를 A 의 시각에 보간한 뒤, 두 기록에 모두 있는 관절 이름에 대해
RMSE·상관·추종 오차·진폭비·지연·토크 피크와 A 의 단계(phase)별 RMSE 를 계산한다. compare_many 는 기준 A 와 여러 기록을 짝지어 비교한다.
"""
from __future__ import annotations

import csv
import json
import socket
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .common import JOINT_SHORT

TRACE_BASE = ["t", "phase", "age_ms", "grav_z", "base_z"]
TRACE_TAIL = ["kp", "kd", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"]
DEFAULT_ROBOT = "rover"


def trace_columns(joints) -> list[str]:
    joints = list(joints)
    return (TRACE_BASE + [f"q_{j}" for j in joints] + [f"dq_{j}" for j in joints]
            + [f"qdes_{j}" for j in joints] + [f"tau_{j}" for j in joints] + TRACE_TAIL)


TRACE_COLUMNS = trace_columns(JOINT_SHORT)          # rover 기본


def joints_from_header(header) -> list[str]:
    return [c[2:] for c in header if c.startswith("q_")]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def label_of(source: str) -> str:
    """dds → real, mujoco → sim, isaac → isaac (시뮬레이터끼리도 구분한다)."""
    return {"dds": "real", "mujoco": "sim", "isaac": "isaac"}.get(source, "sim" if source else "?")


# ---------------- meta.json ----------------
def write_meta(run_dir: Path, source: str, program: str, params: dict | None = None, dt: float | None = None,
               tags=None, note: str = "", extra: dict | None = None, robot: str = DEFAULT_ROBOT,
               joints=None) -> dict:
    """실행 시작 시 meta.json 을 만든다 (status=running)."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    joints = list(joints or JOINT_SHORT)
    meta = {"id": run_dir.name, "robot": robot, "joints": joints, "source": source, "label": label_of(source),
            "program": program, "params": params or {}, "dt": dt, "tags": list(tags or []), "note": note or "",
            "started_at": utc_now(), "ended_at": None, "status": "running", "ticks": 0,
            "host": socket.gethostname(), "python": sys.version.split()[0], "columns": trace_columns(joints)}
    if extra:
        meta.update(extra)
    (run_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return meta


def update_meta(run_dir: Path, **fields) -> dict:
    """실행 종료 등으로 meta.json 을 갱신한다. 없으면 만든다."""
    run_dir = Path(run_dir)
    p = run_dir / "meta.json"
    meta = json.loads(p.read_text(encoding="utf-8")) if p.exists() else infer_meta(run_dir)
    meta.update(fields)
    p.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return meta


def _header_of(run_dir: Path) -> list[str]:
    try:
        with open(run_dir / "trace.csv", encoding="utf-8", newline="") as f:
            return next(csv.reader(f))
    except (OSError, StopIteration):
        return []


def infer_meta(run_dir: Path) -> dict:
    """meta.json 이 없는 옛 기록: summary.json 의 args 나 폴더 이름, trace 헤더에서 추정한다."""
    run_dir = Path(run_dir)
    meta = {"id": run_dir.name, "robot": DEFAULT_ROBOT, "source": "?", "label": "?", "program": "?", "params": {},
            "tags": [], "note": "", "status": "unknown", "inferred": True, "dt": None, "ticks": None,
            "joints": joints_from_header(_header_of(run_dir)) or list(JOINT_SHORT)}
    s = run_dir / "summary.json"
    if s.exists():
        try:
            summ = json.loads(s.read_text(encoding="utf-8"))
            meta["source"] = summ.get("backend", "?")
            meta["program"] = summ.get("program", "?")
            meta["status"] = summ.get("exit_reason", "unknown")
            meta["ticks"] = summ.get("ticks")
            meta["dt"] = summ.get("dt_s")
            args = summ.get("args", {})
            meta["params"] = {k: args.get(k) for k in ("duration", "amp", "freq", "joints", "kp", "kd") if k in args}
        except (OSError, ValueError):
            pass
    if meta["source"] == "?":
        for src in ("mujoco", "dds", "isaac"):
            if f"-{src}-" in run_dir.name or run_dir.name.endswith(f"-{src}"):
                meta["source"] = src
        if run_dir.name.startswith("real-"):
            meta["source"] = "dds"
        elif run_dir.name.startswith(("fake-", "c-", "v2-", "v3-", "live-smoke", "hold-", "sine-", "standup-")):
            meta["source"] = "mujoco" if not run_dir.name.startswith("fake-") else "dds"
    meta["label"] = label_of(meta["source"]) if meta["source"] in ("mujoco", "dds", "isaac") else "?"
    try:
        meta["started_at"] = datetime.fromtimestamp((run_dir / "trace.csv").stat().st_mtime, timezone.utc).isoformat(timespec="seconds")
    except OSError:
        pass
    return meta


def read_meta(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    p = run_dir / "meta.json"
    if p.exists():
        try:
            meta = json.loads(p.read_text(encoding="utf-8"))
        except ValueError:
            meta = infer_meta(run_dir)
    else:
        meta = infer_meta(run_dir)
    meta.setdefault("id", run_dir.name)
    meta.setdefault("robot", DEFAULT_ROBOT)
    meta.setdefault("label", label_of(meta.get("source", "")))
    meta.setdefault("tags", [])
    meta.setdefault("params", {})
    if not meta.get("joints"):
        meta["joints"] = joints_from_header(_header_of(run_dir)) or list(JOINT_SHORT)
    return meta


def list_runs(root: Path) -> list[dict]:
    """runs/ 아래 trace.csv 가 있는 폴더를 최신순으로 나열한다."""
    root = Path(root)
    out = []
    if not root.exists():
        return out
    for d in sorted((p for p in root.iterdir() if p.is_dir() and (p / "trace.csv").exists()),
                    key=lambda p: p.stat().st_mtime, reverse=True):
        meta = read_meta(d)
        meta["has_sim"] = (d / "sim.csv").exists()
        meta["trace_bytes"] = (d / "trace.csv").stat().st_size
        meta["n_joints"] = len(meta.get("joints", []))
        if meta.get("status") == "running" or not meta.get("ticks"):
            try:                                                  # 실행 중이면 지금까지 기록된 틱 수
                with open(d / "trace.csv", "rb") as f:
                    meta["ticks"] = max(0, sum(1 for _ in f) - 1)
            except OSError:
                pass
        out.append(meta)
    return out


# ---------------- trace.csv ----------------
def load_trace(path: Path, max_rows: int | None = None, start_row: int = 0) -> dict:
    """열 이름 → 배열. 문자열 열(phase)은 리스트. 없는 열은 만들지 않는다."""
    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = []
        for i, r in enumerate(reader):
            if i < start_row:
                continue
            if len(r) != len(header):
                continue                                    # 쓰는 중인 마지막 줄
            rows.append(r)
            if max_rows and len(rows) >= max_rows:
                break
    data = {"__header__": header}
    if not rows:
        for name in header:
            data[name] = [] if name == "phase" else np.zeros(0)
        return data
    cols = list(zip(*rows))
    for name, col in zip(header, cols):
        if name == "phase":
            data[name] = list(col)
        else:
            data[name] = np.array([float(x) if x not in ("", "nan") else np.nan for x in col])
    return data


@dataclass
class Run:
    path: Path
    id: str
    meta: dict
    joints: list
    t: np.ndarray
    phase: list
    q: np.ndarray
    dq: np.ndarray
    qdes: np.ndarray
    tau: np.ndarray
    grav_z: np.ndarray
    base_z: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    gyro: np.ndarray | None = None
    acc: np.ndarray | None = None
    sim: dict | None = None
    extra: dict = field(default_factory=dict)

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t))) if len(self.t) > 1 else (self.meta.get("dt") or 0.005)

    @property
    def label(self) -> str:
        return self.meta.get("label", "?")

    @property
    def robot(self) -> str:
        return self.meta.get("robot", DEFAULT_ROBOT)

    def index(self, name: str) -> int:
        return self.joints.index(name)


def _stack(data: dict, prefix: str, joints) -> np.ndarray:
    n = len(data["t"])
    return np.column_stack([data.get(f"{prefix}_{j}", np.full(n, np.nan)) for j in joints]) if joints else np.zeros((n, 0))


def load_run(run_dir: Path, max_rows: int | None = None) -> Run:
    run_dir = Path(run_dir)
    d = load_trace(run_dir / "trace.csv", max_rows)
    joints = joints_from_header(d["__header__"])
    n = len(d["t"])
    gyro = np.column_stack([d[f"gyro_{a}"] for a in "xyz"]) if "gyro_x" in d else None
    acc = np.column_stack([d[f"acc_{a}"] for a in "xyz"]) if "acc_x" in d else None
    sim = None
    if (run_dir / "sim.csv").exists():
        s = load_trace(run_dir / "sim.csv")
        sj = joints_from_header(s["__header__"])
        sim = {"t": s["t"], "joints": sj, "q": _stack(s, "q", sj), "tau": _stack(s, "tau", sj), "base_z": s.get("base_z")}
    meta = read_meta(run_dir)
    meta["joints"] = joints or meta.get("joints", [])
    return Run(path=run_dir, id=run_dir.name, meta=meta, joints=list(meta["joints"]), t=d["t"],
               phase=d.get("phase", ["?"] * n), q=_stack(d, "q", joints), dq=_stack(d, "dq", joints),
               qdes=_stack(d, "qdes", joints), tau=_stack(d, "tau", joints),
               grav_z=d.get("grav_z", np.full(n, np.nan)), base_z=d.get("base_z", np.full(n, np.nan)),
               kp=d.get("kp", np.full(n, np.nan)), kd=d.get("kd", np.full(n, np.nan)), gyro=gyro, acc=acc, sim=sim)


# ---------------- 관절 선택 ----------------
def resolve_joints(run: Run, spec=None) -> list[int]:
    """None → 구동 관절(없으면 처음 4개). 정수·이름·'a,b,c' 문자열을 run 의 관절 인덱스로 바꾼다."""
    if spec is None or spec == "" or spec == []:
        return driven_joints(run)[:6] or list(range(min(4, len(run.joints))))
    if isinstance(spec, str):
        spec = [s.strip() for s in spec.split(",") if s.strip()]
    out = []
    for s in spec:
        if isinstance(s, int) or (isinstance(s, str) and s.isdigit()):
            i = int(s)
            if 0 <= i < len(run.joints) and i not in out:
                out.append(i)
        elif s in run.joints:
            i = run.joints.index(s)
            if i not in out:
                out.append(i)
    return out


def driven_joints(run: Run, threshold: float = 0.01) -> list[int]:
    """목표각이 실제로 움직인 관절. 댐핑 구간(kp=0)은 목표각이 측정각을 따라가므로 제외한다."""
    active = run.kp > 0 if np.isfinite(run.kp).any() else np.ones(len(run.t), dtype=bool)
    if active.sum() < 2 or run.qdes.shape[1] == 0:
        return []
    return [j for j in range(run.qdes.shape[1]) if np.nanstd(run.qdes[active, j]) > threshold]


def driven_names(run: Run) -> list[str]:
    return [run.joints[j] for j in driven_joints(run)]


# ---------------- 차트용 시계열 ----------------
def decimate_idx(n: int, max_points: int) -> np.ndarray:
    if n <= max_points:
        return np.arange(n)
    return np.unique(np.linspace(0, n - 1, max_points).astype(int))


def phase_segments(t: np.ndarray, phase: list) -> list[dict]:
    segs = []
    if not phase:
        return segs
    start = 0
    for i in range(1, len(phase) + 1):
        if i == len(phase) or phase[i] != phase[start]:
            segs.append({"phase": phase[start], "t0": float(t[start]), "t1": float(t[i - 1])})
            start = i
    return segs


def _clean(a: np.ndarray, nd: int):
    return [None if (isinstance(x, float) and np.isnan(x)) else round(float(x), nd) for x in a]


def series(run: Run, joints=None, max_points: int = 2500, relative: bool = False) -> dict:
    """relative=True 면 각 관절의 첫 표본(시작 자세)을 빼서 '시작 자세 기준 변위' 로 준다. joints 는 이름/인덱스."""
    idx = decimate_idx(len(run.t), max_points)
    jidx = resolve_joints(run, joints)
    out = {"id": run.id, "label": run.label, "robot": run.robot, "joints": run.joints, "relative": relative,
           "t": run.t[idx].round(4).tolist(), "phases": phase_segments(run.t, run.phase), "series": {},
           "grav_z": _clean(run.grav_z[idx], 4), "base_z": _clean(run.base_z[idx], 4),
           "gyro_z": _clean(run.gyro[idx, 2], 4) if run.gyro is not None else None}
    for j in jidx:
        name = run.joints[j]
        q0 = run.q[0, j] if relative else 0.0
        qd0 = run.qdes[0, j] if relative else 0.0
        s = {"q": _clean(run.q[idx, j] - q0, 4), "qdes": _clean(run.qdes[idx, j] - qd0, 4),
             "tau": _clean(run.tau[idx, j], 3), "dq": _clean(run.dq[idx, j], 3)}
        if run.sim is not None and name in run.sim["joints"]:
            sj = run.sim["joints"].index(name)
            sidx = decimate_idx(len(run.sim["t"]), max_points)
            s["sim_t"] = run.sim["t"][sidx].round(4).tolist()
            s["sim_q"] = _clean(run.sim["q"][sidx, sj] - (run.sim["q"][0, sj] if relative else 0.0), 4)
            s["sim_tau"] = _clean(run.sim["tau"][sidx, sj], 3)
        out["series"][name] = s
    return out


# ---------------- 비교 ----------------
def lag_ms(reference: np.ndarray, signal: np.ndarray, dt: float, max_lag_s: float = 0.5):
    a, b = reference - np.nanmean(reference), signal - np.nanmean(signal)
    if len(a) < 20 or np.nanstd(a) < 1e-6 or np.nanstd(b) < 1e-6:
        return None
    a, b = np.nan_to_num(a), np.nan_to_num(b)
    max_lag = min(int(max_lag_s / dt), len(a) - 10)
    best, best_c = 0, -np.inf
    for lag in range(0, max_lag + 1):
        c = float(np.dot(a[:len(a) - lag], b[lag:]))
        if c > best_c:
            best, best_c = lag, c
    return best * dt * 1e3


def common_joint_names(a: Run, b: Run, joints=None) -> list[str]:
    if joints:
        names = [j.strip() for j in joints.split(",")] if isinstance(joints, str) else list(joints)
        names = [a.joints[n] if isinstance(n, int) else n for n in names]
    else:
        names = list(dict.fromkeys(driven_names(a) + driven_names(b)))
        if not names:
            names = [n for n in a.joints[:4]]
    return [n for n in names if n in a.joints and n in b.joints]


def compare_runs(a: Run, b: Run, joints=None, relative: bool = False) -> dict:
    """A 의 시각축에 B 를 보간해 겹쳐 놓고, 두 기록에 모두 있는 관절 이름에 대해 비교한다.
    relative=True 면 각 기록의 시작 자세(첫 표본)를 뺀 변위끼리 비교한다 (시작 자세가 다른 시뮬·실기 비교용)."""
    names = common_joint_names(a, b, joints)
    if not names:
        raise ValueError(f"공통 관절이 없습니다 (A: {a.robot} {len(a.joints)}관절, B: {b.robot} {len(b.joints)}관절)")
    t_end = float(min(a.t[-1], b.t[-1]))
    mask = a.t <= t_end
    ta = a.t[mask]
    dt = a.dt
    per_joint = {}
    diffs = []
    for name in names:
        ja, jb = a.index(name), b.index(name)
        oa = (a.q[0, ja], a.qdes[0, ja]) if relative else (0.0, 0.0)
        ob = (b.q[0, jb], b.qdes[0, jb]) if relative else (0.0, 0.0)
        qa, qda, taua = a.q[mask, ja] - oa[0], a.qdes[mask, ja] - oa[1], a.tau[mask, ja]
        qb = np.interp(ta, b.t, b.q[:, jb]) - ob[0]
        qdb = np.interp(ta, b.t, b.qdes[:, jb]) - ob[1]
        taub = np.interp(ta, b.t, b.tau[:, jb])
        d = qa - qb
        diffs.append(d)
        p2p = lambda x: float(np.nanmax(x) - np.nanmin(x))   # noqa: E731
        corr = float(np.corrcoef(qa, qb)[0, 1]) if np.nanstd(qa) > 1e-6 and np.nanstd(qb) > 1e-6 else None
        per_joint[name] = {
            "rmse_ab": float(np.sqrt(np.nanmean(d ** 2))), "maxabs_ab": float(np.nanmax(np.abs(d))),
            "corr_ab": corr,
            "track_rmse_a": float(np.sqrt(np.nanmean((qa - qda) ** 2))),
            "track_rmse_b": float(np.sqrt(np.nanmean((qb - qdb) ** 2))),
            "p2p_qdes_a": p2p(qda), "p2p_qdes_b": p2p(qdb),
            "amp_ratio_a": p2p(qa) / p2p(qda) if p2p(qda) > 1e-3 else None,
            "amp_ratio_b": p2p(qb) / p2p(qdb) if p2p(qdb) > 1e-3 else None,
            "lag_a_ms": lag_ms(qda, qa, dt), "lag_b_ms": lag_ms(qdb, qb, dt),
            "tau_peak_a": float(np.nanmax(np.abs(taua))) if np.isfinite(taua).any() else None,
            "tau_peak_b": float(np.nanmax(np.abs(taub))) if np.isfinite(taub).any() else None,
            "tau_mean_abs_a": float(np.nanmean(np.abs(taua))) if np.isfinite(taua).any() else None,
            "tau_mean_abs_b": float(np.nanmean(np.abs(taub))) if np.isfinite(taub).any() else None,
        }
    diffs = np.column_stack(diffs)
    per_phase = []
    for seg in phase_segments(ta, [p for p, m in zip(a.phase, mask) if m]):
        m = (ta >= seg["t0"]) & (ta <= seg["t1"])
        if m.sum() >= 2:
            per_phase.append({**seg, "rmse_ab": float(np.sqrt(np.nanmean(diffs[m] ** 2)))})
    return {"a": {"id": a.id, "label": a.label, "robot": a.robot, "program": a.meta.get("program"), "params": a.meta.get("params")},
            "b": {"id": b.id, "label": b.label, "robot": b.robot, "program": b.meta.get("program"), "params": b.meta.get("params")},
            "joints": names, "common_seconds": t_end, "dt": dt, "relative": relative,
            "rmse_ab_all": float(np.sqrt(np.nanmean(diffs ** 2))),
            "per_joint": per_joint, "per_phase": per_phase}


def compare_many(runs: list, joints=None, relative: bool = False, max_points: int = 2500) -> dict:
    """기준 A(runs[0]) 와 나머지 각각을 비교하고, 모든 기록의 차트용 시계열을 함께 준다."""
    if len(runs) < 2:
        raise ValueError("비교하려면 기록이 2개 이상 필요합니다")
    a = runs[0]
    pairs = [compare_runs(a, b, joints, relative) for b in runs[1:]]
    names = list(dict.fromkeys(n for p in pairs for n in p["joints"]))
    return {"reference": a.id, "joints": names, "relative": relative, "pairs": pairs,
            "series": {r.id: series(r, [n for n in names if n in r.joints], max_points, relative) for r in runs}}


def compare_series(a: Run, b: Run, joints, max_points: int = 2500, relative: bool = False) -> dict:
    """비교 차트용: A, B 각각의 (t, q, qdes, tau) 를 감축해 준다 (시각축이 달라도 그대로 겹쳐 그린다)."""
    return {"a": series(a, joints, max_points, relative), "b": series(b, joints, max_points, relative)}
