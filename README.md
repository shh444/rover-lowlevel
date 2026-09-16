# Rover 저수준(DDS) 관절 제어 실습

<!-- doc:overview:start -->
Dobot Rover(Quad) SDK 의 저수준 제어(E9: `rt/lower/cmd` 발행 + `rt/lower/state` 구독)를
**MuJoCo 시뮬레이션과 실기(DDS)에서 같은 코드로** 돌리는 실습 패키지다.
Dobot 공식 dobot_sim2real 의 모델·기립 절차와 SDK E9 예제를 합쳤고, 2026-09-16 실기(miniQuad)에서
댐핑 → 자세 유지 → 허벅지 사인 구동까지 확인했다.

```bash
git clone https://github.com/shh444/rover-lowlevel.git && cd rover-lowlevel
pip install -r requirements.txt
# MuJoCo 모델은 Dobot 공식 저장소에서 받는다 (vendor/ 에 두면 자동 인식, 다른 곳이면 ROVER_VENDOR=경로)
git clone --filter=blob:none --sparse https://github.com/Dobot-Team/dobot_rover_simulation.git vendor/dobot_rover_simulation
git -C vendor/dobot_rover_simulation sparse-checkout set dobot_sim2real/resources
python examples/e4_sine_joint.py --backend mujoco      # 로봇 없이 바로
```

| 출처 | 그대로 가져온 것 |
|---|---|
| SDK `low_level/python/e9_motor_cmd_pub.py` | 토픽, QoS, `ABS2HW`(12→16 슬롯), `MOTOR_OFFSET`(영점), `mode(0)`, 사인 구동 기본값(0.2 rad, kp 30, kd 1.2) |
| SDK `docs/api/low_level.md` | 토크 공식 `tau = kp(q_des-q) + kd(dq_des-dq) + tau_ff`, 모터 mode 의미, kill_robot 필수 |
| `dobot_sim2real/config/robot.yaml`, `deploy.py`, `io.py` | 서기/엎드림 자세, 3단계 기립(댐핑 2s → 엎드림 3s → 기립 3s), 게인(kp 60/kd 1.8, 댐핑 kd 3), 안전 종료(엎드림 1.5s → 댐핑 1s), 워치독 0.2s, 넘어짐 감지 |
| `resources/dobot_quad/urdf` | 관절 범위(abad ±0.663, thigh ±2.618, calf ±2.53), 토크 한계(23/23/55 N·m) |

## 파일

```
rover_lowlevel/
├── run.py                   실행기(CLI). 프로그램 선택, 기록(meta.json + trace.csv), 요약 JSON
├── datalab/                 데이터 플랫폼: 시뮬·실기 실행 시작/정지, 기록 목록, 비교 대시보드 (server.py, ui.html)
├── examples/                e1~e10 짧은 예제 (아래 "예제" 절)
├── docs/USAGE.md            라이브러리로 쓰는 법 (API 튜토리얼)
├── docs/platform.md         데이터 플랫폼 설명 (기록 형식, 화면, 지표, API)
├── dds_config.yaml          SDK 의 DDS QoS 설정 사본
├── lowlevel/
│   ├── common.py            관절 순서·매핑·한계·State/JointCmd
│   ├── safety.py            Guard: 워치독·넘어짐·과속·NaN → SafetyAbort, 목표각 범위/변화율/게인 제한
│   ├── programs.py          standup / sine / hold / damp 목표 생성
│   ├── runtime.py           페이싱·신호 처리·기록·안전 종료·실기 게이트, 이를 묶은 Session
│   ├── dataset.py           기록 형식(meta.json, trace.csv) 읽기·목록·비교 지표 (시뮬·실기 공통)
│   ├── backend_mujoco.py    MuJoCo 백엔드 (1ms substep 에서 PD 토크 계산 → mt00~mt11)
│   ├── backend_dds.py       실기 백엔드 (LowerState → State, JointCmd → LowerCmd)
│   └── virtual_plant.py     시험용 가상 로봇 몸체 (MuJoCo, 하드웨어 슬롯 규약) — 제어 코드는 쓰지 않음
├── tools/
│   ├── trace_stats.py       trace.csv 단계별 요약 (오차·토크·몸체 높이)
│   ├── preflight_real.py    실기 연결 전 읽기 전용 점검 (네트워크·DDS 설정·상태 수신·mode·IMU)
│   ├── compare_sim_real.py  실기 기록의 명령을 MuJoCo 에 재생해 관절각·토크·지연·진폭 비교 (오프라인)
│   ├── live_compare.py/.html  실기와 MuJoCo 쌍둥이를 같은 명령으로 동시에 움직이며 브라우저 실시간 차트 비교
│   └── virtual_robot_dds/   C++ 가상 로봇: 실제 SDK 로 rt/lower/state 발행·rt/lower/cmd 구독 (컨테이너에서 build.sh)
├── docker/
│   ├── Dockerfile           Ubuntu 22.04 + Python 3.10 + SDK 0.23.3 (thor 에서 DDS 를 쓰기 위한 컨테이너)
│   ├── sdk.sh               컨테이너 안에서 명령 실행 (호스트 네트워크, 저장소 마운트)
│   ├── dds_env.sh           로봇으로 가는 인터페이스를 찾아 cyclonedds.xml 생성 + CYCLONEDDS_URI 설정
│   └── sdk/                 SDK dist 의 arm64 deb / aarch64 whl (thor 용)
└── tests/
    ├── test_dds_mapping.py  SDK 스텁으로 슬롯/오프셋 왕복 검증
    ├── test_safety.py       가드·프로그램 단계 검증
    ├── fake_dds/            dds_middleware_python 가짜 구현 (in-process, 가상 로봇 몸체는 virtual_plant.py)
    ├── fake_dds_scenarios.sh  가상 로봇으로 DDS 경로 종단 시험 (기립·워치독·넘어짐·Ctrl+C·SIGTERM·포트 검사·preflight)
    └── real_dds_loopback.sh   실제 SDK·CycloneDDS 로 C++ 가상 로봇 ↔ run.py 왕복 시험 (컨테이너)
```

제어 루프는 백엔드와 무관하게 한 가지다.

```
state = io.read()              # 논리 12관절 q/dq, IMU (실기: 오프셋 제거·나이 계산)
guard.check(state)             # 오래됨/넘어짐/과속/NaN → SafetyAbort → 댐핑으로 안전 종료
cmd = program.step(t, state)   # q_des, dq_des, kp, kd, tau_ff (None 이면 종료)
cmd = guard.limit(cmd, state)  # URDF 범위, 변화율 2 rad/s, kp≤100, kd≤5, tau_ff≤10
io.send(cmd)                   # 실기: LowerCmd 발행 / 시뮬: PD 토크 계산 후 물리 진행
```

## thor 에서 MuJoCo 로 실행

thor 에는 `~/rover-lowlevel` 로 복사해 두었고, 기존 venv(`~/rover-mujoco-poc/.venv`, mujoco 3.13)를 그대로 쓴다.

```bash
ssh thor
cd ~/rover-lowlevel
PY=~/rover-mujoco-poc/.venv/bin/python

$PY tests/test_safety.py && $PY tests/test_dds_mapping.py

# 엎드린 상태에서 3단계 기립 → 3초 유지 → 엎드림 → 댐핑 (기록: runs/<UTC>-mujoco-standup/)
MUJOCO_GL=egl $PY run.py --backend mujoco --program standup --duration 3 --snapshot

# E9 와 같은 사인 구동 (엎드린 채 12관절 ±0.2 rad), calf 만 하려면 --joints calf
MUJOCO_GL=egl $PY run.py --backend mujoco --program sine --duration 5 --snapshot

# 서 있는 상태에서 시작해 자세 유지 (kp/kd 실험용)
MUJOCO_GL=egl $PY run.py --backend mujoco --program hold --start standing --kp 60 --kd 1.8 --duration 3
```

`--realtime` 을 붙이면 벽시계 5ms 주기로 돌고 요약에 기한 초과 횟수가 남는다.
`--render-every 40` 이면 0.2초마다 `frames/` 에 이미지를 저장한다.

기록: `trace.csv`(틱마다 q, dq, q_des, tau, 중력 z, 몸체 높이), `summary.json`, `snapshot_*.jpg`.

<!-- doc:overview:end -->
<!-- doc:realrobot:start -->
## 실기(DDS) 로 실행

SDK 문서(E9)의 절차를 코드가 강제한다. 실행기는 `192.168.5.2:50051`(주 제어기 gRPC)이 아직 응답하면
중단하고, 체크리스트에 `yes` 를 입력해야 시작한다.

```bash
# 1) SDK 가 설치된 Ubuntu 22.04 / Python 3.10 환경 (또는 SDK Dockerfile 컨테이너, --network host)
cd dobot_quad_sdk
source setup_cyclonedds_env.sh          # 192.168.5.2 로 가는 인터페이스를 찾아 CYCLONEDDS_URI 설정
cyclonedds ps                           # rt/lower/state 가 보여야 한다

# 2) 주 제어기 종료 (PASSIVE → 5초 대기 → 프로세스 종료)
python3 high_level/python/examples/kill_robot.py 192.168.5.2:50051

# 3) 단계적으로
cd ~/rover-lowlevel
python3 run.py --backend dds --program damp --duration 3                 # 통신·모터 mode 확인만
python3 run.py --backend dds --program hold --duration 5                 # 로봇을 지지한 채 힘이 들어가는지
python3 run.py --backend dds --program sine --joints calf --amp 0.1 --duration 5
python3 run.py --backend dds --program standup --duration 5              # 평평한 바닥에 엎드린 상태에서
```

Ctrl+C 는 언제나 안전 종료(엎드림 → 댐핑)로 이어진다. 상태가 0.2초 이상 끊기거나 몸이 30도 이상
기울면 즉시 댐핑으로 넘어간다. 마지막으로 보내는 명령은 항상 댐핑(kp 0, kd 3)이다.

### thor 에서 직접 DDS 를 쓰려면 (Docker, 준비 완료)

thor 는 Python 3.12 인데 SDK 의 wheel 은 CPython 3.10 전용이라 venv 에 바로 설치되지 않는다.
그래서 `docker/Dockerfile`(Ubuntu 22.04 + SDK arm64 deb + aarch64 wheel + numpy/mujoco)로 `rover-sdk:0.23.3`
이미지를 만들어 두었다. SDK 의 deb 는 `/dobot_algs/middleware/{lib,include,bin}` 에 설치되고, Python 모듈의
RPATH 가 그 경로를 가리키므로 별도 설정 없이 import 된다 (`libyaml-cpp0.7` 만 추가로 필요).

로봇을 thor 의 남는 유선 포트(`mgbe*`)에 연결하고 그 포트에 `192.168.5.x/24` 주소를 준 뒤:

```bash
cd ~/rover-lowlevel
docker build -t rover-sdk:0.23.3 docker/          # 최초 1회 (이미 빌드됨)
docker/sdk.sh bash                                 # 컨테이너 셸 (호스트 네트워크, 저장소는 /work)
source docker/dds_env.sh                           # 192.168.5.2 로 가는 인터페이스 → cyclonedds.generated.xml
python3 tools/preflight_real.py --seconds 5        # 읽기 전용 점검 (명령 없음)
python3 run.py --backend dds --program damp --duration 3     # 이후 hold → sine → standup 순서
```

`preflight_real.py` 는 인터페이스·ping·주 제어기 gRPC 상태·CYCLONEDDS_URI 인터페이스 일치·SDK import·
`rt/lower/state` 수신률·모터 mode/온도·논리 관절각·IMU 중력 방향을 점검하고 JSON 보고서를 남긴다.
같은 도구를 `PYTHONPATH=tests/fake_dds` 로 돌리면 가상 로봇으로 자체 시험이 된다.

<!-- doc:realrobot:end -->
## 데이터 플랫폼 (datalab/)

시뮬과 실기를 같은 형식(`runs/<id>/meta.json` + `trace.csv`)으로 기록하고, 한 화면에서 실행·열람·비교한다.
표준 라이브러리만 쓰므로 thor(실기+시뮬)에서도 노트북(시뮬·열람)에서도 그대로 돈다.

```bash
python datalab/server.py --root runs --port 8095        # http://127.0.0.1:8095/  (thor 면 ssh -L 8768:127.0.0.1:8095 thor)
```

- **수집** 탭: 대상(시뮬/실기)·프로그램·파라미터·태그를 정해 시작, 라이브 차트, 정지(SIGINT → 안전 종료). 실기는 `REAL` 확인 + 주 제어기 검사.
- **기록** 탭: 모든 실행의 목록·차트·태그, 비교 대상 A/B 지정.
- **비교** 탭: A(시뮬) vs B(실기)를 겹쳐 그리고 관절별 RMSE·상관·추종 오차·진폭비·지연·토크 피크, 단계별 RMSE. 시작 자세가 달라도 "시작 자세 기준(상대)" 로 비교.

자세한 내용은 [docs/platform.md](https://github.com/shh444/rover-lowlevel/blob/main/docs/platform.md).

## 예제 (examples/)

SDK 의 e1~e9 처럼 짧은 스크립트다. `lowlevel.runtime.Session` 이 200 Hz 페이싱·가드·Ctrl+C·안전 종료를 맡아
본문이 몇 줄이다. 모두 `--backend mujoco`(기본)로 로봇 없이 돌고, `--backend dds` 면 실기 게이트(주 제어기 검사·체크리스트)를
거친다.

```python
io = make_backend("dds")                 # 또는 "mujoco"
q0 = io.wait_ready().q.copy()
with Session(io, dt=0.005, realtime=True, exit_mode="damp") as s:   # 블록을 어떻게 나가든 댐핑으로 끝난다
    for t, state in s.run(seconds=5.0):
        s.send(JointCmd.pd(q0, kp=30.0, kd=1.2))
```

| 예제 | 내용 | 실기 |
|---|---|---|
| `e1_state_monitor.py` | 상태 읽기만. 실기에서는 writer 를 만들지 않아 kill_robot 없이 안전 | ✅ (preflight 와 같은 경로) |
| `e2_damping.py` | 댐핑 명령만 (kp 0, kd 3). 실기 첫 명령 | ✅ 2026-09-16 |
| `e3_hold_pose.py` | 현재 자세 유지, kp 1초 램프 | ✅ 2026-09-16 |
| `e4_sine_joint.py` | E9 식 사인 (기본 thigh ±0.1 rad, 0.9 Hz) | ✅ 2026-09-16 (thigh ±0.1) |
| `e5_standup.py` | sim2real 3단계 기립 → 유지 → 엎드림 | 시뮬만 |
| `e6_raw_sdk_sine.py` | 패키지 없이 SDK API 만으로 쓴 E9 개선판 (실기 전용, 학습용) | 문법·import 만 확인 |
| `e7_custom_program.py` | 나만의 Program 클래스 작성법 (앞발 들기) | 시뮬·가짜 DDS |
| `e8_squat.py` | 서 있는 자세에서 자세 보간 (스쿼트) | 시뮬만 |
| `e9_gravity_feedforward.py` | `tau` 피드포워드(중력 보상)와 몸통 고정 시험 모드 | 시뮬만 |
| `e10_plot_trace.py` | trace.csv 그래프 (matplotlib) | 실기 기록으로 확인 |

패키지를 라이브러리처럼 쓰는 방법(백엔드·명령·가드·Program 작성·실기 절차·기록 분석)은 [docs/USAGE.md](docs/USAGE.md) 에 정리했다.

<!-- doc:pitfalls:start -->
## 흔히 막히는 지점 (다른 환경에서 저수준 제어가 안 될 때)

이번에 실기까지 된 이유는 로봇이 특별해서가 아니라 아래 함정을 하나씩 피했기 때문이다. 순서대로 확인한다.

| # | 함정 | 증상 | 이 저장소의 대응 |
|---|---|---|---|
| 1 | SDK Python wheel 이 **CPython 3.10 전용**(`cp310`) | 3.11/3.12 에서 pip 설치 거부 또는 import 실패 | Ubuntu 22.04 컨테이너(`docker/`) |
| 2 | deb 가 `/dobot_algs/middleware` 에 설치되는데 문서는 `/usr/local` | C++ 빌드 실패, `CYCLONEDDS_HOME` 무효 | Dockerfile 의 ENV, C++ 도구의 `SDK_ROOT` |
| 3 | `libdds_middleware.so` 가 `libyaml-cpp.so.0.7` 을 요구하지만 deb 에 의존성 선언 없음 | Python import 시 공유 라이브러리 오류 | `libyaml-cpp0.7` 설치 |
| 4 | `cyclonedds.xml` 의 인터페이스 이름 불일치 또는 `CYCLONEDDS_URI` 미설정 | 참여자는 뜨는데 `rt/lower/state` 가 전혀 안 옴 | `docker/dds_env.sh` 가 `ip route get 192.168.5.2` 로 자동 지정, `preflight_real.py` 가 일치 검사 |
| 5 | WiFi(192.168.1.x)로 시도 | DDS 멀티캐스트가 안 감 | 유선 192.168.5.x 만 사용 |
| 6 | **주 제어기를 끄지 않음** | 저수준 명령이 먹지 않거나 두 제어기가 충돌 | 50051 이 열려 있으면 실행 거부, `kill_robot` 후 진행 |
| 7 | writer 생성 직후 발행 | 탐색 전이라 첫 명령들이 유실 | 첫 상태 수신 후 1초 대기 |
| 8 | reader 를 reliable 로 설정 | 로봇 상태는 best_effort 로 오므로 매칭 안 됨 | SDK `dds_config.yaml` 기본(reader best_effort) 유지 |
| 9 | 12관절을 16슬롯 `ABS2HW` 에 넣지 않거나 `MOTOR_OFFSET` 누락 | 목표가 엉뚱한 곳 → 큰 토크·이상 자세 | E9 표 그대로, 고수준 `jpos_leg` 와 대조해 thigh/calf 일치 확인 |
| 10 | 한 번만 보내거나 kp 를 켠 채 프로세스 종료 | 반응 없음 또는 마지막 명령이 남음 | 200 Hz 연속 발행, 어떤 종료든 댐핑 1초로 마무리 |

<!-- doc:pitfalls:end -->
<!-- doc:options:start -->
## 옵션 요약

| 옵션 | 기본 | 설명 |
|---|---|---|
| `--program` | | `standup` 3단계 기립 후 `--duration` 초 유지 / `sine` E9 사인 / `hold` 현재각 유지 / `damp` 댐핑만 |
| `--dt` | 0.005 | 제어 주기(s). 실기는 벽시계에 맞춰 발행 |
| `--amp --freq --joints` | 0.2, 0.9, all | sine 진폭(rad)·주파수(Hz)·대상 관절 (`calf`, `FL,RR`, `0,1,2`) |
| `--kp --kd` | 30, 1.2 | sine/hold 게인 (E9 값) |
| `--slew` | 2.0 | 목표각 변화율 한계 rad/s |
| `--state-timeout` | 0.2 | 워치독(s) |
| `--fall-threshold` | -0.866 | 넘어짐 판정(몸체 중력 z), `--no-fall-check` 로 끔 |
| `--exit` | standup=crouch, 그 외 damp | 종료 방식 |
| `--start` | lying | MuJoCo 초기 자세 (`standing` 이면 서 있는 상태) |
| `--force --yes` | | 실기 gRPC 응답 검사·체크리스트 생략 (권장하지 않음) |

<!-- doc:options:end -->
<!-- doc:verification:start -->
## 확인된 것 / 확인되지 않은 것 (2026-09-16, thor)

기록 요약은 `python tools/trace_stats.py runs/<폴더> [--joint FL_thigh]` 로 본다.

| 실행 (thor, `runs/`) | 결과 |
|---|---|
| `standup-01` (엎드림 → 기립 → 3s 유지 → 엎드림) | 몸체 높이 0.11 m → 0.38 m, 중력 z ≥ −0.999 유지, 유지 구간 추종 오차 평균 0.05 / 최대 0.10 rad, 토크 최대 11 N·m (한계 23/55), 종료 후 다시 0.11 m |
| `sine-01` (엎드린 채 12관절 ±0.2 rad, 0.9 Hz) | 완주. thigh 는 사인을 따라가고, calf 는 엎드린 자세가 URDF 한계(−2.53)에 걸려 가드가 목표를 자른다 (`--joints thigh` 또는 `--start standing` 권장) |
| `hold-01` (서서 kp 60 / kd 1.8) | 중력에 의한 정상 처짐 0.10 rad (sim2real 과 같은 게인·같은 현상. 필요하면 `tau_ff` 에 중력 보상 추가 가능) |
| `standup-rt-01` (`--realtime`, 5 ms 벽시계 페이싱) | 2300 틱 기한 초과 0회, 최대 시작 지연 1 µs (RT 커널) |

- 단위 테스트 `tests/test_safety.py`, `tests/test_dds_mapping.py` 통과 (thor 의 Python 3.12 와 컨테이너의 Python 3.10 둘 다).

### DDS 경로 종단 시험 (가상 로봇, `bash tests/fake_dds_scenarios.sh`)

`tests/fake_dds` 는 `dds_middleware_python` 과 같은 표면을 가진 가짜 SDK 로, MuJoCo 가상 로봇(`lowlevel/virtual_plant.py`)이
500 Hz 로 `rt/lower/state` 를 콜백하고 `rt/lower/cmd` 를 하드웨어 슬롯 단위 PD 로 적용한다. `run.py --backend dds` 를
코드 수정 없이 그대로 돌린다 (5 ms 벽시계 페이싱 포함).

| 시나리오 | 결과 (thor, 2026-09-16) |
|---|---|
| standup (기립 2 s 유지 → 엎드림) | 완주. 명령 2500개(200 Hz, 최대 간격 5.4 ms), 상태 6749개(500 Hz), 몸체 0.10 → 0.38 m, 명령 mode 0 / 상태 mode 4 |
| watchdog (4 s 뒤 상태 발행 중단) | 203 ms 만에 `state_stale` 감지 → 엎드리기 생략, 댐핑만 전송 |
| fall (9 s 뒤 IMU 60도 기울임) | `fallen gravity_z=-0.50` 감지 → 댐핑 |
| sine (thigh 3관절, ±0.2 rad) | 완주 |
| sigint (`timeout -s INT`, 신호 2회) | 첫 신호로 루프 종료, 두 번째는 무시 → 엎드림 1.5 s + 댐핑 1 s(200틱) 완료 후 요약 저장 |
| sigterm (`kill -TERM`) | SIGINT 와 같은 안전 종료 |
| port check (127.0.0.1:50051 열어 둠) | "주 제어기가 아직 응답" 으로 거부, 종료 코드 2 |
| preflight 자체 시험 | 인터페이스·URI·SDK import·수신률 500 Hz·mode·온도·IMU·관절 범위 모두 PASS |

컨테이너(Ubuntu 22.04, Python 3.10, pip mujoco)에서도 같은 standup 이 완주했고, 실제 SDK 로 만든 CycloneDDS 참여자는
로봇이 없을 때 `preflight_real.py` 가 "수신 없음" 으로 정상 실패한다.

### 실제 SDK·CycloneDDS 왕복 시험 (`bash tests/real_dds_loopback.sh`)

SDK 의 Python 바인딩은 메시지 필드를 읽을 수만 있어서(setter 없음) 상태를 발행하는 쪽은 C++ 로 만들었다.
`tools/virtual_robot_dds/virtual_robot.cc` 는 SDK 의 `dds_middleware` + 생성된 `LowerState_`/`LowerCmd_` 타입으로
`rt/lower/state` 를 500 Hz 로 내고 `rt/lower/cmd` 를 받아 관절별 1차 모델(중력 없음)에 적용한다. 컨테이너 두 개
(호스트 네트워크, 실제 CycloneDDS 멀티캐스트) 사이에서 `run.py --backend dds --program standup` 을 돌린 결과:

| 항목 | 결과 (thor, 2026-09-16) |
|---|---|
| 토픽·QoS 매칭 | 로봇 best_effort 상태 ↔ 클라이언트 best_effort 구독, 클라이언트 reliable 명령 ↔ 로봇 best_effort 구독 모두 연결 |
| 상태 수신 | 로봇 발행 8536개 중 클라이언트 실행 구간 6748개 수신 (500 Hz), 파싱 오류 0, 상태 나이 1 ms |
| 명령 수신 | 200 Hz 명령 2500개 전부 수신, 최대 간격 5.7 ms, kp 최대 60 |
| 슬롯·오프셋 | 슬롯 3/7/11/15 는 건드리지 않음(온도 0 으로 확인), 오프셋을 거친 논리 관절각이 양쪽에서 일치 (calf −2.524) |
| 동작 | 엎드림 → 기립(default 자세 오차 0.000) → 2 s 유지 → 엎드림 → 댐핑, 기한 초과 0 |

### 실기 1차 확인 (2026-09-16, thor enP2p1s0 = 192.168.5.154, 로봇 192.168.5.2, 읽기 전용)

| 항목 | 관측 |
|---|---|
| 링크 | ping 0.6 ms, 주 제어기 gRPC 50051 열림, FSM 상태 `READY`(엎드린 안전 정지), 기종 `miniQuad` |
| `rt/lower/state` | **994 Hz** (5 s 에 4968개), 간격 max 2.0 ms, 파싱 오류 0 |
| 모터 mode(16 슬롯) | 다리 12개 = 4(controlled), 슬롯 3/7/11/15 = 2(offline, 바퀴 없는 다리형) → 온도도 0 |
| 온도 | 27~31 °C |
| IMU | gravity_z = −1.000, |quat| = 1.000 (SDK 문서대로 wxyz) |
| 관절각 교차 확인 | 고수준 `jpos_leg` 와 저수준 논리각(E9 오프셋 제거)이 **thigh/calf 는 완전히 일치**, abad 만 정확히 ∓0.05 rad 차이 (고수준은 abad 의 ±0.05 영점 오프셋을 적용하지 않음) |

abad 0.05 rad(2.9°)는 현재 자세 기준 상대 동작(`hold`, `sine`)에는 영향이 없고, 절대 자세를 쓰는 `standup`(abad 목표 0)에서도
미미하지만 어느 쪽 영점이 맞는지는 아직 모른다.

### 실기 저수준 명령 (2026-09-16, `kill_robot` 이후, 로봇 엎드린 상태)

| 순서 | 명령 | 결과 |
|---|---|---|
| 1 | `kill_robot` | READY → PASSIVE → 제어기 종료 (gRPC 50051 닫힘). 저수준 상태는 993 Hz 로 계속 수신, 관절각·자세 변화 없음 |
| 2 | `damp --duration 3` (kp 0, kd 3) | 600틱 기한 초과 0(최대 지연 2 µs), 관절각 변화 0, 추정 토크 ≤ 0.3 N·m, mode 4 유지 |
| 3 | `hold --duration 5` (kp 5→30, kd 1.2) | 1000틱, 오차 0.000, 토크 ≤ 0.3 N·m (현재 자세라 힘이 거의 필요 없음) |
| 4 | `sine --joints thigh --amp 0.1 --freq 0.9 --duration 5` | **첫 실제 움직임.** 허벅지 4관절이 사인을 추종: 오차 평균 0.011 / 최대 0.066 rad(바닥에 닿은 다리를 올릴 때 지연), 토크 최대 2.0 N·m, 댐핑으로 정상 종료 |

증적: `evidence/20260916/real-*.json`, `real-sine-01.trace.csv`. `standup` 과 12관절 사인(E9 진폭 0.2)은 아직 실기에서 돌리지 않았다.
kill_robot 이후 고수준 앱/제어기를 다시 쓰려면 로봇을 재부팅해야 한다.

### MuJoCo vs 실기 (2026-09-16, 몸통을 박스에 올려 다리를 띄운 상태)

실기 사인 구동의 명령열을 몸통 고정 MuJoCo 모델에 그대로 재생(`tools/compare_sim_real.py --fixed-base`)하고, 같은 동작을
40 s 동안 실시간으로 쌍둥이와 나란히 돌려(`tools/live_compare.py`) 비교했다. 두 방법의 수치는 같다.

| 항목 | MuJoCo | 실기 |
|---|---|---|
| 허벅지 진폭비 (움직인 폭 / 명령 폭) | 0.97~1.01 | 0.62~0.78 (앞다리가 더 작음) |
| 명령 대비 지연 | 45 ms | 60~70 ms |
| 댐핑(kp 0) 중 다리 | 중력 평형점으로 흔들림 | 전혀 움직이지 않음 (정지 마찰) |
| 관절각 차이 RMSE (실기−시뮬, 구동 관절) | 0.02~0.05 rad | |

실기 관절에는 모델(`frictionloss 0.02`)보다 훨씬 큰 마찰이 있어 0.9 Hz 에서 명령의 ~30% 를 덜 움직인다. 자세한 표·그래프·해석은
[docs/comparison.md](https://github.com/shh444/rover-lowlevel/blob/main/docs/comparison.md) 에 있다.

- 가드는 실습용 최소 방어선이다. 전원 차단 등 독립된 비상정지를 항상 준비한다.

<!-- doc:verification:end -->
## 라이선스와 출처

이 저장소의 코드는 MIT 라이선스다 (`LICENSE`). 매핑 표·QoS 설정은 Dobot Quad SDK(MIT), 기립 절차·게인·자세와
MuJoCo 모델은 Dobot Rover Simulation(BSD 3-Clause, 모델은 별도로 받음), 물리 엔진은 MuJoCo(Apache 2.0)를 쓴다.
SDK 배포물(deb/whl)은 저장소에 넣지 않으며 `docker/sdk/` 에 직접 복사한다.
