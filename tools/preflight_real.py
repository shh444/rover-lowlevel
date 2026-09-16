#!/usr/bin/env python3
"""실기 연결 전 읽기 전용 점검. 모터 명령을 보내지 않는다 (LowerCmd writer 도 만들지 않는다).

    python tools/preflight_real.py --seconds 5 [--robot-ip 192.168.5.2] [--out runs/preflight.json]
    PYTHONPATH=tests/fake_dds python tools/preflight_real.py --seconds 3 --robot-ip 127.0.0.1   # 가상 로봇 자체 시험

점검: 192.168.5.x 인터페이스 → ping → 주 제어기 gRPC(50051) 상태 → CYCLONEDDS_URI 와 인터페이스 이름 일치
→ SDK import → rt/lower/state 수신률·모터 mode·온도·관절각(논리 순서, 오프셋 제거)·IMU 중력 방향.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lowlevel.common import JOINT_SHORT, LEGS, Q_LOWER, Q_UPPER   # noqa: E402

MODE_NAMES = {0: "disabled", 1: "error", 2: "offline", 3: "enabled", 4: "controlled", 5: "homing"}
RESULTS: dict[str, dict] = {}


def report(name, status, detail=""):
    RESULTS[name] = {"status": status, "detail": detail}
    print(f"[{status}] {name}: {detail}")
    return status == "PASS"


def find_iface(prefix):
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None, None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3].startswith(prefix):
            return parts[1], parts[3]
    return None, None


def ping(ip):
    try:
        return subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True, timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def port_open(ip, port):
    try:
        with socket.create_connection((ip, port), timeout=1.0):
            return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default="192.168.5.2")
    ap.add_argument("--seconds", type=float, default=5.0, help="상태 수신 관찰 시간")
    ap.add_argument("--dds-config", type=Path, default=ROOT / "dds_config.yaml")
    ap.add_argument("--stale-ms", type=float, default=100.0, help="수신 간격이 이보다 크면 끊김으로 셈")
    ap.add_argument("--out", type=Path, default=None, help="JSON 보고서 경로 (기본 runs/preflight-<UTC>.json)")
    a = ap.parse_args()
    out = a.out or ROOT / "runs" / f"preflight-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    summary = {"checked_at_utc": datetime.now(timezone.utc).isoformat(), "robot_ip": a.robot_ip, "results": RESULTS}

    def finish(code):
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
        print(f"보고서: {out}")
        return code

    # 1) 네트워크
    prefix = ".".join(a.robot_ip.split(".")[:3]) + "."
    iface, addr = find_iface(prefix)
    report("유선 인터페이스", "PASS" if iface else "FAIL",
           f"{iface} {addr}" if iface else f"{prefix}x 주소를 가진 인터페이스 없음 (PC 를 192.168.5.xxx/24 로 설정)")
    report(f"ping {a.robot_ip}", "PASS" if ping(a.robot_ip) else "FAIL", "")
    grpc = port_open(a.robot_ip, 50051)
    report("주 제어기 gRPC 50051", "INFO", "열림 → 모터 명령 전 kill_robot 필요" if grpc else "닫힘 (kill_robot 실행됨 또는 로봇 미연결)")
    summary["main_controller_grpc_open"] = grpc

    # 2) CycloneDDS 설정
    uri = os.environ.get("CYCLONEDDS_URI", "")
    xml_path = uri[7:] if uri.startswith("file://") else uri
    xml_iface = None
    if xml_path and Path(xml_path).exists():
        text = Path(xml_path).read_text(encoding="utf-8", errors="replace")
        m = re.search(r'NetworkInterface\s+name="([^"]+)"', text) or re.search(r"<NetworkInterfaces>\"?([^\"<]+)", text)
        xml_iface = m.group(1) if m else None
        report("CYCLONEDDS_URI", "PASS", f"{xml_path} (인터페이스 {xml_iface})")
        if iface:
            report("cyclonedds.xml 인터페이스 = 유선 인터페이스", "PASS" if xml_iface == iface else "FAIL",
                   f"{xml_iface} vs {iface}")
    else:
        report("CYCLONEDDS_URI", "FAIL", f"{uri or '(비어 있음)'} → source docker/dds_env.sh 또는 SDK setup_cyclonedds_env.sh")

    # 3) SDK 와 상태 수신 (읽기 전용)
    try:
        import dds_middleware_python
        report("SDK import", "PASS", getattr(dds_middleware_python, "__file__", "?"))
    except Exception as exc:
        report("SDK import", "FAIL", repr(exc))
        return finish(1)
    from lowlevel.backend_dds import DdsBackend
    io = DdsBackend(a.dds_config, timeout=a.seconds + 5.0, discovery_wait=0.0, create_writer=False)
    try:
        io.wait_ready()
    except RuntimeError as exc:
        report("rt/lower/state 수신", "FAIL", str(exc))
        return finish(1)

    arrivals, last_t = [], None
    t_end = time.monotonic() + a.seconds
    count0 = io.count
    while time.monotonic() < t_end:
        st = io.read()
        if st.t != last_t:
            arrivals.append(st.t)
            last_t = st.t
        time.sleep(0.0005)
    n_total = io.count - count0
    gaps = np.diff(arrivals) * 1e3 if len(arrivals) > 1 else np.array([])
    rate = n_total / a.seconds
    report("rt/lower/state 수신률", "PASS" if rate > 50 else "FAIL",
           f"{rate:.0f} Hz ({n_total}개/{a.seconds:.0f}s), 폴링 기준 간격 max {gaps.max() if len(gaps) else 0:.1f} ms, "
           f"{a.stale_ms:.0f} ms 초과 {int((gaps > a.stale_ms).sum()) if len(gaps) else 0}회, 파싱 오류 {io.parse_errors}")
    summary["state_rx"] = {"count": n_total, "rate_hz": rate, "gap_max_ms": float(gaps.max()) if len(gaps) else None}

    st = io.read()
    modes = st.extra.get("mode_hw", np.zeros(16, int))
    temps = st.extra.get("temp_hw", np.zeros(16, int))
    mode_set = sorted({int(x) for x in np.asarray(modes)[list(range(16))]})
    report("모터 mode(16 슬롯)", "INFO", " ".join(f"{m}={MODE_NAMES.get(m, '?')}" for m in mode_set) + f"  raw={np.asarray(modes).tolist()}")
    tmax = int(np.max(temps)) if len(temps) else 0
    report("모터 온도", "PASS" if tmax < 70 else "FAIL", f"max {tmax} °C  raw={np.asarray(temps).tolist()}")
    gz = float(st.gravity_body()[2])
    qn = float(np.linalg.norm(st.quat_wxyz))
    report("IMU 중력 방향", "PASS" if gz < -0.95 else "WARN", f"gravity_z={gz:+.3f} (평평하면 -1), |quat|={qn:.3f}")
    inside = (st.q >= Q_LOWER - 0.3) & (st.q <= Q_UPPER + 0.3)
    report("관절각 범위(URDF ±0.3)", "PASS" if inside.all() else "WARN",
           "모두 범위 안" if inside.all() else "범위 밖: " + ", ".join(JOINT_SHORT[i] for i in np.flatnonzero(~inside)))
    print("\n논리 관절각 (오프셋 제거, rad)      abad    thigh    calf")
    for k, leg in enumerate(LEGS):
        print(f"  {leg}: {st.q[3 * k]:+8.3f} {st.q[3 * k + 1]:+8.3f} {st.q[3 * k + 2]:+8.3f}")
    summary["state_sample"] = {"q_logical": st.q.round(4).tolist(), "dq": st.dq.round(4).tolist(),
                               "mode_hw": np.asarray(modes).tolist(), "temp_hw": np.asarray(temps).tolist(),
                               "quat_wxyz": st.quat_wxyz.round(5).tolist(), "gravity_z": gz}
    failed = [k for k, v in RESULTS.items() if v["status"] == "FAIL"]
    print("\n결과:", "모두 통과" if not failed else "실패 항목 → " + ", ".join(failed))
    return finish(0 if not failed else 1)


if __name__ == "__main__":
    sys.exit(main())
