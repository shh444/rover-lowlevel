#!/usr/bin/env python3
"""외부 CSV(다른 로봇·다른 도구의 기록)를 플랫폼 기록 형식(meta.json + trace.csv)으로 가져온다.

    python tools/import_run.py data.csv --robot humanoid --source dds --program walk \
        --time-col t --q-prefix q_ --qdes-prefix qdes_ --tau-prefix tau_ --dq-prefix dq_ --phase-col phase --tag imported

관절 이름은 --q-prefix 로 시작하는 열 이름에서 얻는다(--joints 로 제한 가능). 없는 항목(qdes, dq, tau)은 NaN 으로 채운다.
시간 열이 없으면 --dt 로 만든다. 가져온 기록은 datalab 목록에 robot 이름으로 나타나고, 관절 이름이 겹치는 기록과 비교할 수 있다.
"""
from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lowlevel.dataset import trace_columns, update_meta, utc_now, write_meta   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--source", default="dds", help="mujoco | dds | isaac | 기타")
    ap.add_argument("--program", default="import")
    ap.add_argument("--time-col", default="t")
    ap.add_argument("--dt", type=float, default=0.005, help="시간 열이 없을 때의 주기")
    ap.add_argument("--q-prefix", default="q_")
    ap.add_argument("--dq-prefix", default="dq_")
    ap.add_argument("--qdes-prefix", default="qdes_")
    ap.add_argument("--tau-prefix", default="tau_")
    ap.add_argument("--phase-col", default=None)
    ap.add_argument("--joints", default=None, help="가져올 관절 이름(쉼표). 기본: q-prefix 열 전부")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tag", action="append", default=[])
    ap.add_argument("--note", default="")
    a = ap.parse_args()

    with open(a.csv, encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        header = reader.fieldnames or []
    joints = [c[len(a.q_prefix):] for c in header if c.startswith(a.q_prefix)]
    if a.joints:
        wanted = [j.strip() for j in a.joints.split(",") if j.strip()]
        joints = [j for j in wanted if j in joints]
    if not joints:
        print(f"'{a.q_prefix}' 로 시작하는 열이 없습니다. 헤더: {header}")
        return 1
    n = len(rows)
    t = (np.array([float(r[a.time_col]) for r in rows]) if a.time_col in header else np.arange(n) * a.dt)
    t = t - t[0]

    def col(prefix, j):
        name = prefix + j
        return [float(r[name]) if name in header and r[name] not in ("", "nan") else float("nan") for r in rows]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = a.out or (ROOT / "runs" / f"{stamp}-{'real' if a.source == 'dds' else 'sim'}-{a.program}-{a.robot}")
    out.mkdir(parents=True, exist_ok=True)
    q = np.column_stack([col(a.q_prefix, j) for j in joints])
    dq = np.column_stack([col(a.dq_prefix, j) for j in joints])
    qdes = np.column_stack([col(a.qdes_prefix, j) for j in joints])
    tau = np.column_stack([col(a.tau_prefix, j) for j in joints])
    phases = [r[a.phase_col] for r in rows] if a.phase_col and a.phase_col in header else ["import"] * n
    with open(out / "trace.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(trace_columns(joints))
        for i in range(n):
            w.writerow([f"{t[i]:.4f}", phases[i], "0.0", "nan", "nan"]
                       + [f"{x:.5f}" for x in q[i]] + [f"{x:.4f}" for x in dq[i]]
                       + [f"{x:.5f}" for x in qdes[i]] + [f"{x:.3f}" for x in tau[i]]
                       + ["nan", "nan", "nan", "nan", "nan", "nan", "nan", "nan"])
    dt = float(np.median(np.diff(t))) if n > 1 else a.dt
    write_meta(out, source=a.source, program=a.program, dt=dt, tags=a.tag + ["imported"], note=a.note,
               robot=a.robot, joints=joints, params={"csv": str(a.csv)})
    update_meta(out, status="imported", ended_at=utc_now(), ticks=n, program_seconds=float(t[-1]) if n else 0.0)
    print(f"가져오기 완료: {out}  ({n}행, 관절 {len(joints)}개: {joints})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
