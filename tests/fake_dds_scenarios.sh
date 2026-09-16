#!/usr/bin/env bash
# 가짜 DDS(가상 로봇)로 run.py --backend dds 경로를 끝까지 검증한다. 실기·네트워크 불필요.
#   PY=~/rover-mujoco-poc/.venv/bin/python bash tests/fake_dds_scenarios.sh
set -u
cd "$(dirname "$0")/.."
PY=${PY:-python3}
export PYTHONPATH=tests/fake_dds        # 가짜 SDK 가 실제 dds_middleware_python 보다 먼저 잡힌다 (렌더링 없음)
mkdir -p runs

summarize() {
  "$PY" - "$1" <<'EOF'
import json, sys
d = "runs/fake-" + sys.argv[1]
s = json.load(open(d + "/summary.json")); r = json.load(open(d + "/robot.json"))
print(f"  reason={s['exit_reason']} exit_mode={s['exit_mode']} ticks={s['ticks']} "
      f"misses={s['deadline_misses']} max_late={s['max_late_ms']:.2f}ms")
print(f"  robot: cmds={r['cmds']} states={r['states']} cmd_gap_max={r['cmd_gap_max_ms']:.1f}ms "
      f"kp_max={r['kp_max_seen']:.0f} tau_max={r['tau_max_seen']:.1f} base_z={r['base_z_min']:.3f}~{r['base_z_max']:.3f}")
print(f"  final_q={[round(x, 2) for x in r['final_q']]}  modes(hw)={r['last_cmd_modes']}")
print(f"  dds={s.get('dds')}")
EOF
}

run() {   # run <name> [run.py 옵션...]
  local name=$1; shift
  echo "=== $name ==="
  rm -rf "runs/fake-$name"
  FAKE_DDS_REPORT="runs/fake-$name/robot.json" "$PY" run.py --backend dds --robot-ip 127.0.0.1 --yes \
      --out "runs/fake-$name" "$@" > "runs/fake-$name.log" 2>&1
  echo "  exit=$?"
  summarize "$name"
}

run standup --program standup --duration 2
FAKE_DDS_STOP_AT=4 run watchdog --program standup --duration 2
FAKE_DDS_TILT_AT=9 run fall --program standup --duration 3
run sine --program sine --duration 3 --joints thigh

echo "=== sigint (6초 뒤 Ctrl+C. timeout 은 SIGINT 를 프로세스와 그룹에 두 번 보내므로 '두 번 누름' 시험이 된다) ==="
rm -rf runs/fake-sigint
FAKE_DDS_REPORT=runs/fake-sigint/robot.json timeout -s INT 6 "$PY" run.py --backend dds --robot-ip 127.0.0.1 --yes \
    --out runs/fake-sigint --program standup --duration 30 > runs/fake-sigint.log 2>&1
echo "  exit=$? (timeout 은 124)"
summarize sigint
grep -c "exit_damp" runs/fake-sigint/trace.csv | sed "s/^/  exit_damp rows=/"

echo "=== sigterm (8초 뒤 kill -TERM: SSH 끊김·kill 도 안전 종료로) ==="
rm -rf runs/fake-sigterm
FAKE_DDS_REPORT=runs/fake-sigterm/robot.json "$PY" run.py --backend dds --robot-ip 127.0.0.1 --yes \
    --out runs/fake-sigterm --program standup --duration 30 > runs/fake-sigterm.log 2>&1 &
pid=$!
sleep 8; kill -TERM $pid; wait $pid
echo "  exit=$?"
summarize sigterm

echo "=== preflight 자체 시험 (가상 로봇, lo 인터페이스) ==="
source docker/dds_env.sh lo
"$PY" tools/preflight_real.py --seconds 2 --robot-ip 127.0.0.1 --out runs/fake-preflight.json
echo "  exit=$?"
unset CYCLONEDDS_URI

echo "=== port check (주 제어기 gRPC 가 살아있으면 거부) ==="
"$PY" -c 'import socket, time; s = socket.socket(); s.bind(("127.0.0.1", 50051)); s.listen(1); time.sleep(6)' &
sleep 0.5
"$PY" run.py --backend dds --robot-ip 127.0.0.1 --program damp --duration 1 --out runs/fake-port 2>&1 | tail -1
echo "  exit=${PIPESTATUS[0]} (2 여야 함)"
wait
