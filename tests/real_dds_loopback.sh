#!/usr/bin/env bash
# 실제 SDK(CycloneDDS)로 통신 검증: C++ 가상 로봇(tools/virtual_robot_dds) ↔ run.py --backend dds.
# thor 호스트에서 실행하며, 두 프로세스 모두 rover-sdk 컨테이너(호스트 네트워크) 안에서 돈다. 로봇·192.168.5.x 불필요.
#   bash tests/real_dds_loopback.sh              (IFACE=enP2p1s0 bash tests/real_dds_loopback.sh 로 인터페이스 지정)
set -u
cd "$(dirname "$0")/.."
IFACE=${IFACE:-$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'dev \K\S+')}
PROGRAM=${PROGRAM:-standup}
DURATION=${DURATION:-2}
echo "=== real DDS loopback: iface=$IFACE program=$PROGRAM ==="
mkdir -p runs
[ -x tools/virtual_robot_dds/build/virtual_robot ] || docker/sdk.sh bash tools/virtual_robot_dds/build.sh
rm -rf "runs/realdds-$PROGRAM" runs/realdds-robot.json

docker/sdk.sh bash -c "source docker/dds_env.sh $IFACE > /dev/null && exec tools/virtual_robot_dds/build/virtual_robot 120 runs/realdds-robot.json" \
    > runs/realdds-robot.log 2>&1 &
ROBOT=$!
sleep 3
docker/sdk.sh bash -c "source docker/dds_env.sh $IFACE > /dev/null && python3 run.py --backend dds --robot-ip 127.0.0.1 --yes --program $PROGRAM --duration $DURATION --out runs/realdds-$PROGRAM" \
    > "runs/realdds-$PROGRAM.log" 2>&1
echo "client exit=$?"
# 컨테이너 안 프로세스에 TERM 을 전달하려면 docker 컨테이너를 찾아 kill 한다 (docker/sdk.sh 는 exec 이므로 $ROBOT 은 docker CLI)
CID=$(docker ps -q --filter ancestor=rover-sdk:0.23.3 | head -1)
[ -n "$CID" ] && docker kill --signal=TERM "$CID" > /dev/null
wait $ROBOT 2>/dev/null
echo "robot exit=$?"
grep -E "^t=|준비\] 첫 상태|모터 mode|종료|Traceback|Error" "runs/realdds-$PROGRAM.log" | head -30
tail -3 runs/realdds-robot.log
python3 - "$PROGRAM" <<'EOF'
import json, sys
d = "runs/realdds-" + sys.argv[1]
s = json.load(open(d + "/summary.json"))
r = json.load(open("runs/realdds-robot.json"))
print(f"client: reason={s['exit_reason']} exit_mode={s['exit_mode']} ticks={s['ticks']} misses={s['deadline_misses']} "
      f"max_late={s['max_late_ms']:.2f}ms states_rx={s['dds']['states_received']} parse_errors={s['dds']['parse_errors']}")
print(f"client final q (logical): {s['final']['q']}")
print(f"robot : states_tx={r['states']} cmds_rx={r['cmds']} cmd_gap_max={r['cmd_gap_max_ms']:.1f}ms kp_max={r['kp_max_seen']:.0f} tau_max={r['tau_max_seen']:.1f}")
print(f"robot final q (logical): {[round(x, 3) for x in r['final_q']]}")
EOF
