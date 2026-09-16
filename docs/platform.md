# 데이터 플랫폼 (datalab)

시뮬레이터와 실기를 **같은 형식으로 기록**하고, 한 화면에서 **실행·열람·비교**하는 웹 대시보드다.
표준 라이브러리만 쓰므로 thor(실기+시뮬)에서도, 노트북(시뮬·열람)에서도 그대로 돈다.

```bash
python datalab/server.py --root runs --port 8095            # 브라우저: http://127.0.0.1:8095/
# thor 에서 띄우고 PC 에서 보기:  ssh -L 8768:127.0.0.1:8095 thor  →  http://127.0.0.1:8768/
```

## 기록 형식 (시뮬·실기 공통)

모든 실행(`run.py`, `examples/*`, `tools/live_compare.py`)은 `runs/<id>/` 에 같은 형식을 남긴다.

| 파일 | 내용 |
|---|---|
| `meta.json` | 출처(`source`: mujoco/dds → `label`: sim/real), 프로그램, 파라미터, 태그·메모, 시작/종료 시각, 상태(running/completed/keyboard_interrupt/safety:…), 틱 수, 기한 초과, 호스트 |
| `trace.csv` | 틱마다 `t, phase, age_ms, grav_z, base_z, q×12, dq×12, qdes×12, tau×12, kp, kd, gyro_xyz, acc_xyz` (0.1 s 마다 flush → 실행 중 열람 가능) |
| `summary.json` | run.py 요약 (선택) |
| `sim.csv` | live_compare 쌍둥이 기록 (선택) |

`lowlevel/dataset.py` 가 이 형식을 읽고(`load_run`, `list_runs`) 비교한다(`compare_runs`). meta.json 이 없는 옛 기록은
summary.json 과 폴더 이름에서 추정한다.

## 화면

- **수집**: 대상(시뮬/실기), 프로그램(sine/hold/damp/standup), 시간·진폭·주파수·관절·kp·kd, 태그·메모를 정해 시작한다.
  실행 중 라이브 차트(명령 vs 측정)와 작업 로그가 보이고, **정지** 는 SIGINT 를 보내 run.py 가 엎드림/댐핑으로 안전 종료한다.
  실기는 확인란에 `REAL` 을 입력해야 하고, run.py 의 주 제어기(gRPC 50051) 검사도 그대로 적용된다.
- **기록**: 모든 실행의 목록(출처·프로그램·파라미터·틱·상태·태그). 차트 보기, 태그·메모 수정, 비교 대상 A/B 지정.
- **비교**: A(예: 시뮬)와 B(예: 실기)를 겹쳐 그리고 지표를 낸다. B 를 A 의 시각축에 보간해 공통 구간에서 관절별
  RMSE·상관·추종 RMSE·진폭비·지연·토크 피크, 그리고 A 의 단계(phase)별 RMSE 를 계산한다.

## 비교 지표의 뜻

| 지표 | 정의 |
|---|---|
| RMSE A−B | 같은 시각의 측정 관절각 차이의 RMS |
| 상관 | 두 측정 관절각의 피어슨 상관 (모양이 같은지) |
| 추종 RMSE | 각자 자기 명령(q_des) 대비 오차 |
| 진폭비 | 움직인 폭 / 명령 폭 (1 이면 명령대로) |
| 지연 | 명령 대비 상호상관 최대 지점 (ms) |
| 토크 피크 | 실기는 `tau_est`, 시뮬은 적용 토크의 최대 절대값 |

같은 프로그램을 시뮬과 실기에서 각각 돌린 기록을 비교하면 "제어기 + 로봇" 전체의 차이를, `tools/compare_sim_real.py` 로
실기 명령을 시뮬에 재생한 기록을 비교하면 "로봇(물리)" 만의 차이를 본다.

## 실행 명령 설정

`datalab/config.json` 의 `commands` 가 백엔드별 실행 명령이다. 기본값은 시뮬은 서버를 띄운 파이썬으로 `run.py`,
실기는 `docker/sdk.sh` 컨테이너 안에서 `run.py` (thor). 한계(`amp_max` 0.3, `kp_max` 100, `duration_max` 600)도 여기서 바꾼다.
노트북(Windows)에서는 시뮬만 시작할 수 있고, 실기 기록은 thor 에서 복사해 와 열람·비교한다.

## API

| 경로 | 내용 |
|---|---|
| `GET /api/runs` | 기록 목록 |
| `GET /api/run/<id>` | meta + summary |
| `GET /api/run/<id>/series?joints=1,4&max=2500` | 차트용 시계열 (실행 중이면 지금까지) |
| `GET /api/compare?a=<id>&b=<id>&joints=` | 지표 + 두 시계열 |
| `POST /api/start` | `{backend, program, duration, amp, freq, joints, kp, kd, start, tags, note, confirm}` |
| `POST /api/stop` | `{job}` |
| `GET /api/jobs` | 작업 목록·로그 꼬리·한계·프로그램 목록 |
| `POST /api/run/<id>/meta` | `{tags, note}` |
