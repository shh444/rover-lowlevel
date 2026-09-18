# 데이터 플랫폼 (datalab)

시뮬레이터(MuJoCo, Isaac Sim)와 실기, 그리고 **다른 로봇(휴머노이드 등)** 의 실행을 같은 형식으로 기록하고,
한 화면에서 **실행·열람·비교**하는 웹 대시보드다. 표준 라이브러리만 쓰므로 thor(실기+시뮬)에서도, 노트북(시뮬·열람)에서도 그대로 돈다.

```bash
python datalab/server.py --root runs --port 8095            # 브라우저: http://127.0.0.1:8095/
# thor 에서 띄우고 PC 에서 보기:  ssh -L 8768:127.0.0.1:8095 thor  →  http://127.0.0.1:8768/
```

## 기록 형식 (모든 로봇·출처 공통)

모든 실행(`run.py`, `examples/*`, `tools/live_compare.py`, `tools/mujoco_record.py`, `tools/import_run.py`)은 `runs/<id>/` 에 같은 형식을 남긴다.

| 파일 | 내용 |
|---|---|
| `meta.json` | `robot`(rover, atom_upper …), `joints`(관절 이름 목록), `source`(mujoco/dds/isaac → `label` sim/real/isaac), 프로그램, 파라미터, 태그·메모, 시작/종료 시각, 상태, 틱 수, 기한 초과, 호스트 |
| `trace.csv` | 틱마다 `t, phase, age_ms, grav_z, base_z, q_<관절>…, dq_<관절>…, qdes_<관절>…, tau_<관절>…, kp, kd, gyro_xyz, acc_xyz`. 관절 이름·개수는 헤더에서 읽는다. 0.1 s 마다 flush → 실행 중 열람 |
| `summary.json` | run.py 요약 (선택) |
| `sim.csv` | live_compare 쌍둥이 기록 (선택) |

`lowlevel/dataset.py` 가 이 형식을 읽고(`load_run`, `list_runs`) 비교한다(`compare_runs`, `compare_many`). 관절 수가 달라도 되고,
비교는 두 기록에 **모두 있는 관절 이름** 에 대해서만 한다 (공통 관절이 없으면 오류). meta.json 이 없는 옛 기록은 추정한다.

## 기록을 만드는 방법

| 방법 | 대상 | 명령 |
|---|---|---|
| Rover 실행기 | Rover 시뮬·실기·Isaac | `python run.py --backend mujoco|dds|isaac --program sine … --robot rover` |
| 범용 MJCF 기록기 | 임의 MuJoCo 로봇 (휴머노이드 상체 등) | `python tools/mujoco_record.py --xml model.xml --robot atom_upper --program sine --joints left_shoulder_pitch --amp 0.3` |
| 실시간 쌍둥이 | 실기 + MuJoCo 동시 | `python tools/live_compare.py --backend dds …` |
| 외부 CSV 가져오기 | 다른 도구·로봇의 기록 | `python tools/import_run.py data.csv --robot humanoid --source dds --q-prefix q_ …` |

범용 기록기는 모델의 액추에이터가 붙은 관절을 모델 이름 그대로 관절 목록으로 쓰고, Rover 와 같은 PD(`tau = kp(q_des−q) + kd(dq_des−dq)`)를
액추에이터 한계로 잘라 적용한다. 예: thor 백업의 Atom 상체 모델(`atom_upper.xml`, 관절 17개)을 `robot=atom_upper` 로 기록했다.

## 화면

- **수집**: 대상(시뮬/실기/Isaac/MJCF 로봇), 프로그램, 시간·진폭·주파수·관절·kp·kd, 로봇 이름, (MJCF) xml 경로, 뷰어 창, 몸통 고정, 태그·메모.
  실행 중 라이브 차트(명령 vs 측정)와 작업 로그. **정지** 는 SIGINT(Windows: CTRL_BREAK)를 보내 안전 종료(엎드림/댐핑)하게 한다.
  실기는 확인란에 `REAL` 을 입력해야 하고, run.py 의 주 제어기(gRPC 50051) 검사도 그대로 적용된다.
- **기록**: 로봇·출처·검색 필터, 로봇별 기록 수 개요(출처별 sim/real/isaac 수, 클릭하면 그 로봇으로 필터), 목록(로봇·출처·프로그램·파라미터·관절 수·틱·상태·태그), 차트 보기(관절 선택, 시작 자세 기준), 태그·메모·로봇 이름 수정, 비교 대상 A/B/C 지정.
- **비교**: 기준 A 와 B(, C)를 겹쳐 그린다. 관절 칩으로 골라 보고, 차트 종류(관절각, 차이 A−X, 토크, IMU 중력 z·자이로 z)를 켜고 끈다.
  요약 카드(RMSE, 진폭비, 지연 — 진폭비·지연은 명령 폭 0.05 rad 이상으로 실제 구동한 관절만 평균)와 쌍별 관절 지표 표, A 의 단계별 RMSE 표, 지표 JSON 내려받기. 시작 자세가 달라도 "시작 자세 기준" 으로 비교.

## 비교 지표의 뜻

| 지표 | 정의 |
|---|---|
| RMSE A−X | 같은 시각의 측정 관절각 차이의 RMS (X 를 A 의 시각축에 보간) |
| 상관 | 두 측정 관절각의 피어슨 상관 (모양이 같은지) |
| 추종 RMSE | 각자 자기 명령(q_des) 대비 오차 |
| 진폭비 | 움직인 폭 ÷ 명령 폭 (1 이면 명령대로) |
| 지연 | 명령 대비 상호상관 최대 지점 (ms) |
| 토크 피크 | 실기는 `tau_est`, 시뮬은 적용 토크의 최대 절대값 |

같은 프로그램을 시뮬과 실기에서 각각 돌린 기록을 비교하면 "제어기 + 로봇" 전체의 차이를, `tools/compare_sim_real.py` 로 실기 명령을
시뮬에 재생한 기록을 비교하면 "로봇(물리)" 만의 차이를 본다. MuJoCo 와 Isaac Sim 처럼 시뮬레이터끼리도 같은 방법으로 비교한다.

## 실행 명령 설정 (`datalab/config.json`)

| 키 | 내용 |
|---|---|
| `commands` | 백엔드별 실행 명령. 기본: `mujoco`/`mjcf` 는 서버의 파이썬, `dds` 는 `docker/sdk.sh` 컨테이너(thor). `isaac` 은 Isaac Sim 환경의 python 경로를 넣으면 나타난다 |
| `mjcf_models` | 수집 탭 MJCF 목록: 이름 → xml 경로 (예: `"atom_upper": "../thor-backup-20260916/extracted/atom-max-lab/atom_upper_available_meshes.xml"`) |
| `limits` | `amp_max` 0.5, `kp_max` 100, `kd_max` 5, `duration_max` 600 |
| `sim_realtime` | 플랫폼에서 시작한 시뮬을 벽시계 페이싱으로 (기본 true) |
| `realtime_backends` | 페이싱을 거는 백엔드 목록 (기본 `["mujoco"]`). Isaac Sim 은 실시간보다 느려 제외한다 |

`_` 로 시작하는 키는 주석·예시로 무시된다.

## API

| 경로 | 내용 |
|---|---|
| `GET /api/runs` | 기록 목록 + `robots`, `labels`, `programs` |
| `GET /api/run/<id>` | meta + summary |
| `GET /api/run/<id>/series?joints=a,b&max=2500&rel=0` | 차트용 시계열 (실행 중이면 지금까지) |
| `GET /api/compare?runs=A,B[,C]&joints=&rel=0` | 기준 A 와 나머지의 쌍별 지표 + 모든 기록의 시계열 |
| `POST /api/start` | `{backend, program, duration, amp, freq, joints, kp, kd, start, viewer, fixed_base, robot, xml, tags, note, confirm}` |
| `POST /api/stop` | `{job}` |
| `GET /api/jobs` | 작업 목록·로그 꼬리·백엔드·프로그램·한계·MJCF 목록 |
| `POST /api/run/<id>/meta` | `{tags, note, robot}` |
