# 사용법 튜토리얼

이 문서는 `lowlevel` 패키지를 **내 스크립트에서 라이브러리처럼 쓰는 법**을 설명한다. 실행기(`run.py`)와
예제(`examples/`)가 하는 일을 직접 조립할 수 있게 되는 것이 목표다. 단위: 각도 rad, 속도 rad/s, 토크 N·m, 시간 s.

## 1. 구성 요소

| 이름 | 역할 | 어디에 |
|---|---|---|
| `State` | 로봇 상태. `q`, `dq`, `tau_est`(12, 논리 순서), `quat_wxyz`, `gyro`, `acc`, `age`(상태 나이), `motor_mode`, `extra` | `lowlevel/common.py` |
| `JointCmd` | 관절 명령 `q, dq, kp, kd, tau`(각 12). 토크 = `kp(q_des−q) + kd(dq_des−dq) + tau` | `lowlevel/common.py` |
| 백엔드 | `MujocoBackend`, `DdsBackend`. 같은 메서드: `wait_ready()`, `read()`, `send(cmd)`, `close()` | `lowlevel/backend_*.py` |
| `Guard` | `check(state)` 이상 감지 → `SafetyAbort`, `limit(cmd, state)` 목표각 범위·변화율·게인 제한 | `lowlevel/safety.py` |
| `Program` | `step(t, state)` → `JointCmd` 또는 `None`(끝). `DampProgram`, `HoldProgram`, `SineProgram`, `StandUpProgram` | `lowlevel/programs.py` |
| `Session` | 페이싱 + 가드 + Ctrl+C + 안전 종료를 묶은 컨텍스트. `run(seconds)` 로 (t, state) 를 내고 `send(cmd)` | `lowlevel/runtime.py` |
| `make_backend` | 이름으로 백엔드 생성. `"dds"` 면 주 제어기 검사와 체크리스트를 먼저 거친다 | `lowlevel/runtime.py` |

논리 관절 순서는 항상 `FL(abad, thigh, calf), FR, RL, RR` 이다 (`JOINT_SHORT` 참고). 하드웨어 16 슬롯과
영점 오프셋(`ABS2HW`, `MOTOR_OFFSET`)은 `DdsBackend` 안에서만 다루므로 사용자 코드는 신경 쓰지 않는다.

## 2. 최소 코드

```python
import sys; sys.path.insert(0, "/path/to/rover-lowlevel")        # 저장소 루트에서 실행하면 불필요
from lowlevel.common import JointCmd
from lowlevel.runtime import Session, make_backend

io = make_backend("mujoco")                                        # 또는 make_backend("dds")
q0 = io.wait_ready().q.copy()                                      # 첫 상태 (실기: 첫 수신까지 최대 10초 대기)
with Session(io, dt=0.005, realtime=(io.name == "dds"), exit_mode="damp") as s:
    for t, state in s.run(seconds=3.0):                            # 200 Hz 로 (t, state) 를 준다
        s.send(JointCmd.pd(q0, kp=30.0, kd=1.2))                   # 가드가 제한한 뒤 전송
print(s.reason)                                                    # completed / keyboard_interrupt / safety:...
```

`with` 블록을 어떻게 빠져나가든(정상 종료, `break`, Ctrl+C, SIGTERM, 가드 이상, 다른 예외) **안전 종료**가 실행된다.
`exit_mode="damp"` 는 댐핑 1초, `"crouch"` 는 엎드림 1.5초 후 댐핑 1초(서 있을 때). 마지막 명령은 항상 댐핑이다.

## 3. 백엔드

```python
from lowlevel.backend_mujoco import MujocoBackend
io = MujocoBackend(start="standing")            # "lying"(기본, 엎드린 채 낙하) | "standing"
io = MujocoBackend(fixed_base=True)             # 몸통을 공중에 고정 = '지지된 상태에서 다리 시험'
io = MujocoBackend(viewer=True)                 # 뷰어 창 (노트북). run.py 는 --viewer (realtime 자동)
io = MujocoBackend(xml_path=..., physics_dt=0.001, dt=0.005, frames_dir="frames", render_every=40)
io.snapshot("shot.jpg")                          # EGL 렌더 (MUJOCO_GL=egl)
tau_g = io.nominal_gravity(state.q, state.quat_wxyz)   # 명목 모델 중력 토크 g(q) → tau 피드포워드용

from lowlevel.backend_dds import DdsBackend
io = DdsBackend("dds_config.yaml")               # CYCLONEDDS_URI 필요, 첫 상태 + 탐색 1초 대기
io = DdsBackend("dds_config.yaml", create_writer=False)   # 읽기 전용 (모니터·preflight)
```

MuJoCo 모델 경로는 `find_default_xml()` 이 `vendor/`, `/work/vendor`, `~/rover-mujoco-poc/vendor`, `$ROVER_VENDOR`
순으로 찾는다. 시뮬은 `send()` 를 불러야 물리가 진행된다(댐핑 명령이라도 보내야 시간이 흐른다).

`make_backend("dds", yes=True, force=False, robot_ip="192.168.5.2", read_only=False)`:
`force` 는 주 제어기 검사 무시(권장하지 않음), `yes` 는 체크리스트 프롬프트 생략, `ROVER_ROBOT_IP` 환경변수로 IP 를 바꿀 수 있다.

## 4. 명령 만들기

```python
JointCmd.pd(q_des, kp, kd)                      # 위치 제어 (dq_des=0, tau=0)
JointCmd.pd(q_des, kp, kd, dq=dq_des, tau=tau_ff)
JointCmd.damping(kd=3.0, q=state.q)             # kp 0: 힘을 내지 않고 속도만 감쇠
cmd.torque(state.q, state.dq)                   # 이 명령이 만들 토크 (시뮬 백엔드가 쓰는 식)
```

`q_des`, `kp`, `kd` 는 스칼라 또는 (12,) 배열. 관절별로 다른 게인을 주려면 `np.tile([kp_abad, kp_thigh, kp_calf], 4)`.

| 상황 | kp | kd | 출처 |
|---|---|---|---|
| 댐핑(보호) | 0 | 3.0 | sim2real `damp_kd` |
| E9 사인 시험 | 30 | 1.2 | SDK E9 |
| 기립·서 있기 | 60 | 1.8 | sim2real `kp_stand` |
| 보행 정책 | 10 | 1.0 | sim2real `kp` |

가드 상한은 kp 100, kd 5, tau 10 N·m, 목표각 변화율 2 rad/s, URDF 범위(abad ±0.663, thigh ±2.618, calf ±2.53).

## 5. 안전 가드 튜닝

```python
from lowlevel.safety import Guard
guard = Guard(dt=0.005, slew=2.0, state_timeout=0.2, fall_threshold=-0.866, check_fall=True)
with Session(io, guard=guard) as s: ...
```

- `state_timeout`: 상태가 이 시간 이상 안 오면 `state_stale` → 댐핑. 실기 상태는 약 1 kHz 로 오므로 0.2 s 는 넉넉하다.
- `fall_threshold`: 몸체 좌표계 중력 z 가 이보다 크면 넘어짐. −0.866 = 30°. 로봇을 옆으로 눕혀 시험할 땐 `check_fall=False`.
- `slew`: 큰 목표 점프를 막는다. 프로그램이 목표를 갑자기 바꿔도 2 rad/s 로 따라간다. 댐핑 중엔 기준을 실제각으로 맞춘다.

## 6. 나만의 Program

`step(t, state)` 가 `JointCmd` 를 돌려주고, 끝나면 `None`. 기록용 `phase` 문자열을 두면 trace.csv 에 남는다.
`examples/e7_custom_program.py` 의 `PawLift` 가 전체 예시다.

```python
import math
from lowlevel.programs import Program
from lowlevel.common import JointCmd, KD_DAMP

class Nod(Program):                              # 앞다리 두 개의 허벅지를 함께 0.2 rad 굽혔다 펴기
    def __init__(self, state0, period=2.0, reps=3):
        self.q0, self.period, self.reps = state0.q.copy(), period, reps
    def step(self, t, state):
        if t < 1.0:                              # 1초 댐핑으로 초기각 안정
            self.phase = "settle"; self.q0 = state.q.copy()
            return JointCmd.damping(KD_DAMP, state.q)
        tt = t - 1.0
        if tt >= self.reps * self.period:
            return None
        s = 0.5 - 0.5 * math.cos(2 * math.pi * tt / self.period)     # 0→1→0
        q = self.q0.copy(); q[[1, 4]] += 0.2 * s                       # FL, FR thigh
        self.phase = f"nod{int(tt // self.period) + 1}"
        return JointCmd.pd(q, 30.0, 1.2)

prog = Nod(io.wait_ready())
with Session(io, exit_mode="damp") as s:
    for t, state in s.run():
        cmd = prog.step(t, state)
        if cmd is None: break
        s.phase = prog.phase
        s.send(cmd)
```

시뮬로 먼저 돌려 `runs/.../trace.csv` 를 `tools/trace_stats.py` 나 `examples/e10_plot_trace.py` 로 본 뒤 실기로 간다.

## 7. 실기 절차 (요약)

1. 로봇을 유선(192.168.5.x)으로 연결, `source docker/dds_env.sh` (인터페이스 자동 지정)
2. `python tools/preflight_real.py --seconds 5` — 읽기 전용. 수신률, mode, IMU, 관절 범위 확인
3. `kill_robot` (고수준 클라이언트) — 로봇이 엎드린(READY/PASSIVE) 상태에서만
4. `examples/e2_damping.py --backend dds` → `e3_hold_pose.py` → `e4_sine_joint.py --joints thigh --amp 0.1`
5. 끝나면 로봇은 댐핑 상태로 남는다. 고수준 앱을 다시 쓰려면 로봇 재부팅

thor 처럼 Python 3.12 인 PC 에서는 SDK wheel(cp310) 이 설치되지 않으므로 `docker/sdk.sh` 로 컨테이너 안에서 실행한다.

## 8. 기록 분석

`trace.csv` 열: `t, phase, age_ms, grav_z, base_z, q_<관절>×12, dq_×12, qdes_×12, tau_×12, kp, kd`.
`base_z` 는 시뮬에서만 값이 있다(실기는 nan). `tau_` 는 시뮬에서는 적용 토크, 실기에서는 SDK 의 `tau_est`.

```bash
python tools/trace_stats.py runs/<폴더> --joint FL_thigh --every 0.25     # 단계별 오차·토크 요약 + 표본
python examples/e10_plot_trace.py runs/<폴더> --joints FL_thigh,FL_calf   # PNG 그래프 (matplotlib)
```

## 9. MuJoCo 와 실기 비교

```bash
# 오프라인: 실기 기록의 명령을 시뮬에 재생 (박스 위 지지 상태면 --fixed-base)
python tools/compare_sim_real.py runs/real-sine-01 --joints FL_thigh,FR_thigh --fixed-base --out runs/compare-sine-01-fixed

# 온라인: 실기와 쌍둥이를 같은 명령으로 동시에 움직이며 브라우저 차트 (thor 에서 실행, PC 에서 터널)
docker/sdk.sh bash -c "source docker/dds_env.sh && python3 tools/live_compare.py --backend dds --program sine --joints thigh --amp 0.1 --seconds 40"
ssh -L 8767:127.0.0.1:8090 thor     # 다른 창에서 → http://127.0.0.1:8767/

# 로봇 없이 시험 (시뮬 vs 시뮬)
python tools/live_compare.py --backend mujoco --program sine --seconds 10
```

차트 페이지는 관절 체크박스, 시간 창(5~60 s), 최근 창의 RMSE·진폭비·지연·토크 피크 표를 보여준다. 끝나면
`runs/live-*/real.csv`, `sim.csv`, `summary.json` 이 남고 `examples/e10_plot_trace.py <run> --sim <run>/sim.csv` 로 정적 그래프를 만든다.
쌍둥이 모델은 `--twin fixed`(몸통 공중 고정, 기본) 또는 `--twin ground --base-z 0.22`(바닥) 로 고른다.

## 10. 데이터 플랫폼으로 수집·비교

```bash
python datalab/server.py --root runs --port 8095      # http://127.0.0.1:8095/
```

수집 탭에서 시뮬/실기/Isaac/임의 MJCF 로봇 실행을 시작·정지하고(라이브 차트), 기록 탭에서 로봇·출처별로 목록·태그를 관리하고,
비교 탭에서 A vs B(, C)를 겹쳐 본다. 모든 기록은 `runs/<id>/meta.json` + `trace.csv` 형식이며 관절 수·이름에 무관하다
(휴머노이드 등 다른 로봇도 같은 형식). `lowlevel.dataset` 으로 스크립트에서도 읽을 수 있다.

```bash
python tools/mujoco_record.py --xml path/to/humanoid.xml --robot atom_upper --program sine --joints left_shoulder_pitch --amp 0.3
python tools/import_run.py external.csv --robot humanoid --source dds --program walk --q-prefix q_
```

```python
from lowlevel.dataset import load_run, compare_runs
a, b = load_run("runs/<sim-id>"), load_run("runs/<real-id>")
m = compare_runs(a, b, relative=True)          # 시작 자세가 다르면 relative=True
print(m["rmse_ab_all"], m["per_joint"]["FL_thigh"]["amp_ratio_b"])
```

## 11. 자주 하는 실수

- `send()` 없이 `read()` 만 반복 → 시뮬은 시간이 흐르지 않는다.
- 목표각에 오프셋을 더해서 보냄 → 논리각만 쓴다. 오프셋은 `DdsBackend` 가 처리한다.
- 실기에서 `--force` 로 주 제어기 검사를 건너뜀 → 두 제어기가 충돌한다.
- kp 를 켠 채 프로세스를 죽임 → `Session`/`run.py` 를 통해 끝내면 항상 댐핑으로 마무리된다.
- `--backend dds` 인데 `realtime=False` → 최대 속도로 발행해 명령 주기가 깨진다. 실기는 항상 `realtime=True`.
