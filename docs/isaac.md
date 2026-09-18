# Isaac Sim 백엔드

NVIDIA Isaac Sim **6.1** 을 MuJoCo 와 같은 인터페이스(`wait_ready / read / send / close`)의 백엔드로 추가했다
(`lowlevel/backend_isaac.py`). 프로그램·가드·기록·플랫폼은 그대로 쓰고 `--backend isaac` 만 바꾼다.

**상태 (2026-09-17, 실행 검증 완료)**: 노트북(RTX 5070 Laptop 8 GB, 드라이버 610, Python 3.12)에 Isaac Sim 6.1.0.0 을 `C:\isaac-venv` 에 설치하고,
6.1 API(`URDFImporter`, `isaacsim.core.experimental.prims.Articulation`, `SimulationManager`)로 작성한 백엔드를 **PhysX 와 Newton(MuJoCo-Warp) 두 엔진**에서
연기 시험(`tools/isaac_smoke.py`)과 허벅지 sine 실행으로 검증했다. 결과는 아래 [검증 결과](#검증-결과) 에 있다.

## 요구 사항

| 항목 | 내용 |
|---|---|
| 하드웨어 | x86_64, NVIDIA RTX GPU (VRAM 8 GB 이상 권장), 드라이버 570 이상. Jetson(thor, aarch64)에서는 돌지 않는다 |
| 소프트웨어 | `isaacsim[all,extscache]==6.1.0.0` (pypi.nvidia.com). **Python 3.12 지원** (5.x 는 3.11 이 필요했다). 휠 다운로드 약 5.4 GB + 설치 후 수 GB |
| Windows | 경로 길이 제한(MAX_PATH 260)에 걸리므로 **짧은 경로** 에 venv 를 만든다 (예: `C:\isaac-venv`). 저장소 폴더 안 `.venv-isaac` 에 설치하면 `…\omni.sensors.nv.camera-…\…png.azure` 에서 `OSError [Errno 2]` 로 실패한다 (긴 경로 지원이 꺼진 PC) |
| EULA | 첫 실행 때 NVIDIA Omniverse Kit EULA 동의 프롬프트. 동의하면 패키지 안에 `isaacsim/kit/EULA_ACCEPTED` 가 남아 다시 묻지 않는다 (또는 환경변수 `OMNI_KIT_ACCEPT_EULA=YES`). 코드는 대신 동의하지 않고 안내만 한다 |

```powershell
py -3.12 -m venv C:\isaac-venv
C:\isaac-venv\Scripts\python.exe -m pip install "isaacsim[all,extscache]==6.1.0.0" --extra-index-url https://pypi.nvidia.com
# 첫 실행: EULA 를 읽고 동의하면 Yes 입력 (한 번만)
C:\isaac-venv\Scripts\python.exe -c "import isaacsim"
# 연기 시험 (창 표시)
C:\isaac-venv\Scripts\python.exe tools\isaac_smoke.py --viewer
```

URDF 는 `vendor/dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf` 를 쓴다 (`--urdf` 로 지정 가능).
관절 이름(`joint_front_left_abad` …)이 MuJoCo 모델·`JOINT_NAMES` 와 같아서 이름으로 순서를 맞춘다.

## 동작 방식 (6.1 API)

1. `SimulationApp` 을 띄우고(헤드리스는 뷰포트 갱신 끔, 시작 약 12 s) `SimulationManager.setup_simulation(dt=1 ms, device=cpu)` 로 물리 장면을 만든다.
   물리 엔진은 `physx`(기본) 또는 `newton`(MuJoCo-Warp 솔버)을 `--isaac-engine` 으로 고른다. Newton 은 기본 python 앱에 실려 있지 않아
   `isaacsim.physics.newton` + `isaacsim.physics.newton.tensors` 확장을 켠 뒤 엔진을 전환한다.
2. `URDFImporter` 로 URDF 를 USD 로 변환한다: 드라이브 없음(강성·감쇠 0, 목표 없음), `fix_base` 로 고정/자유 베이스, `robot_type=Quadruped`.
   결과는 `.isaac_cache/<urdf>-<fixed|free>/` 에 두고 다음 실행부터 재사용한다 (첫 변환 약 2 s).
3. 변환 결과는 **다중 물리 패키지** 다: variant set `Physics` 에 `physx / mujoco / physics / none` 이 있고 기본 선택이 없어 그대로 참조하면
   관절도 ArticulationRootAPI 도 없다. 참조할 때 엔진에 맞는 variant 를 고른다 (physx → `physx`, newton → `mujoco`; `physics_variant=` 로 바꿀 수 있다).
4. 로봇 높이는 참조 프림(`/World/rover`) 자체를 옮겨서 정한다. 고정 베이스의 `root_joint`(PhysicsFixedJoint) 는 body0 이 이 프림이라 앵커가 같이 따라온다.
   articulation root 프림(`…/Geometry/link_trunk`)을 직접 옮기면 안 된다. 바닥면은 항상 넣는다 (고정 베이스에서는 발이 닿지 않아 기준면 역할만 한다).
   자유 베이스는 물리 초기화 뒤에 몸통 자세·속도를 다시 맞춘다: 초기화의 워밍업 스텝이 관절 0(다리를 편 자세)으로 돌면서 발이 바닥을 뚫고,
   그 반발 속도가 남아 시작하자마자 솟구치며 기울어 넘어짐 판정(`safety:fallen`)이 났었다.
5. 타임라인을 재생하지 않고 `SimulationManager.initialize_physics()` 로 초기화한 뒤 `SimulationManager.step()` 으로 substep(1 ms)마다 직접 진행한다.
   `send(cmd)` 는 substep 마다 SDK 공식 `tau = kp(q_des−q) + kd(dq_des−dq) + tau_ff` 를 계산해 URDF effort 한계(23/23/55 N·m)로 자르고
   `set_dof_efforts` 로 넣는다 → MuJoCo 백엔드와 같은 방식이라 두 시뮬레이터의 차이는 물리 엔진 차이만 남는다.
   수동 초기화가 안 되는 환경이면 타임라인 재생 방식(틱 5 ms 마다 토크 1회, `app.update()`)으로 자동 전환한다 (`io.mode`).
6. IMU: 몸통 프림의 자세(wxyz)와 각속도(몸체 좌표계로 변환). 가속도는 정지 상태 근사(중력 반작용)다.
7. `--fixed-base` 면 몸통을 공중(0.85 m)에 고정한다 (지지된 로봇 시험). 기본 자세는 MuJoCo 와 같다. `nominal_gravity()` 는 Isaac 의 gravity compensation 토크를 돌려준다 (e9 예제).
8. 종료: Kit 의 fast shutdown 은 `app.close()` 에서 프로세스를 바로 끝내므로 `close()` 는 닫기를 인터프리터 종료 시점(atexit)으로 미룬다.
   그래야 호출자의 요약 파일·메타 갱신·출력이 남는다.
9. 속도: PhysX 결과를 substep(1 ms)마다 USD 에 쓰면 물리 한 스텝이 3.5 ms 걸린다. `SimulationManager.enable_fabric(True)` 로 결과를 Fabric 에만 쓰게 하면
   0.9 ms 로 줄어 틱(5 ms)당 19.0 ms → 6.5 ms 가 된다 (`fabric=True` 가 기본).
10. 뷰어(`--viewer`): Isaac Sim 앱 창("Isaac Sim Python 6.1.0")이 뜬다. 빈 스테이지에는 조명이 없어 돔·키 조명과 바닥면을 넣고 카메라를 로봇 쪽으로 맞춘다.
    화면은 시뮬 틱과 무관하게 **벽시계 30 fps** 로만 갱신한다 (`RenderingManager.render()`: 갱신 중 물리가 진행되지 않게 해 준다. 직전에
    `update_articulations_kinematic()` + `physx fabric update(0, 0)` 로 최신 자세를 반영). 창을 닫으면 안전 종료한다.
    `--snapshot` 은 뷰포트 이미지를 저장한다 (헤드리스여도 렌더링을 켠다).

## 사용

```bash
C:\isaac-venv\Scripts\python.exe run.py --backend isaac --program sine --joints thigh --amp 0.1 --fixed-base --duration 5 --out runs/isaac-sine-01
C:\isaac-venv\Scripts\python.exe run.py --backend isaac --program sine --joints all --amp 0.3 --freq 0.4 --fixed-base --duration 25 --viewer   # 창으로 보기
C:\isaac-venv\Scripts\python.exe run.py --backend isaac --program standup --duration 3 --viewer
C:\isaac-venv\Scripts\python.exe run.py --backend isaac --program sine --isaac-engine newton …      # Newton 엔진
C:\isaac-venv\Scripts\python.exe examples\e4_sine_joint.py --backend isaac                          # make_backend("isaac") 도 지원
```

플랫폼(`datalab/`)은 `datalab/config.json` 의 `commands.isaac` 에 Isaac Sim 환경의 python 경로가 있으면 수집 탭에 **Isaac Sim** 이 나타난다
(이 저장소의 config 에는 `C:/isaac-venv/Scripts/python.exe` 가 들어 있다. 없는 PC 에서는 그 항목을 지운다).
기록의 출처는 `isaac`(초록) 으로 표시되어 MuJoCo(`sim`)·실기(`real`)와 나란히 비교할 수 있다. EULA 에 동의하지 않은 상태로 시작하면
작업 로그에 안내 문구와 함께 오류로 끝난다.

## 검증 결과

2026-09-17, 노트북, 헤드리스, 몸통 고정, physics_dt 1 ms × 5 substep, 제어 200 Hz, 기한 초과 0회.

**연기 시험** (`tools/isaac_smoke.py`: 댐핑 2 s → 기본 자세 유지 3 s, kp 30 / kd 1.2)

| 엔진 | 시작 | 유지 오차 (마지막 0.5 s) | 토크 피크 | 기록 |
|---|---|---|---|---|
| PhysX | 12 s | 평균 0.007 rad, 최대 0.035 rad (abad, 중력 처짐) → PASS | 1.8 N·m | `runs/isaac-smoke` |
| Newton (MuJoCo-Warp) | 12 s + 커널 컴파일 ~35 s (첫 회) | 평균 0.021 rad, 최대 0.035 rad → PASS | 1.3 N·m | `runs/isaac-smoke-newton` |

**허벅지 sine ±0.1 rad 0.9 Hz, kp 30 / kd 1.2, 5 s** — 플랫폼 비교 탭(기준 A = MuJoCo `mj-sine-fixed-01`, 시작 자세 기준):

| 비교 | RMSE A−X | 상관 | 진폭비 A / X | 지연 A / X | 토크 피크 A / X |
|---|---|---|---|---|---|
| MuJoCo vs Isaac PhysX (`isaac-sine-fixed-01`) | 0.006–0.007 rad | 1.00 | 1.00 / 1.00 | 10–15 / 10–15 ms | 1.26 / 1.27 N·m |
| MuJoCo vs Isaac Newton (`isaac-newton-sine-fixed-01`) | 0.009 rad | 0.999 | 1.00 / 1.00 | 10–15 / 15–20 ms | 1.26 / 1.67 N·m |
| MuJoCo vs 실기 (`real-sine-01`) | 0.18 rad | 0.70–0.75 | 1.00 / 0.62–0.79 | 10–15 / 60–70 ms | 1.26 / 1.1–2.0 N·m |

**자유 베이스 standup** (엎드림에서 낙하 → 댐핑 → 웅크림 → 일어서기, `mj-standup-01` vs `isaac-standup-01`): 두 시뮬 모두 넘어지지 않고 일어선다.
일어서기·유지 구간의 관절각 RMSE 는 0.03–0.04 rad, 최종 몸통 높이 0.378 m(MuJoCo) / 0.385 m(Isaac), 토크 피크 11.7 / 11.8 N·m.
낙하 직후의 수동 구간(댐핑·웅크림 초반)은 접촉·마찰 모델과 시작 잡음 차이로 0.43 rad 까지 벌어진다.

같은 URDF 유래 모델과 같은 1 ms PD 를 쓰면 MuJoCo 와 Isaac(PhysX 0.007 rad, Newton 0.009 rad) 은 사실상 같은 답을 낸다 (Newton 은 토크 피크가 조금 높다). 실기와의 차이(진폭 −30 %, 지연 +50 ms)는
시뮬레이터 종류가 아니라 실기 관절 마찰·지연 때문이라는 뜻이다 ([comparison.md](comparison.md) 의 결론과 같다).

**속도** (노트북, 12 관절 1대, physics 1 ms × 5, CPU 물리)

| 구성 | 틱(5 ms)당 | 실시간 대비 |
|---|---|---|
| USD 갱신 (fabric 끔) | 19.0 ms | 3.8배 느림 |
| Fabric (기본), 헤드리스 | 6.5 ms | 1.3배 느림 |
| Fabric + 뷰어 창 (30 fps 렌더) | 9.5 ms | 1.9배 느림 |

뷰어 창은 창 자체를 캡처해 두 시점의 자세가 다른 것으로 확인했다 (`omni.kit.viewport.utility.capture_viewport_to_file` 은 수십 프레임 늦은
그림을 돌려줘서 검증용으로 부적합하다).

## 다음 단계

1. 실기 차이를 줄이려면 URDF 의 `<dynamics damping/friction>`, Isaac 의 solver 설정(iteration, contact offset), 관절 armature 를 MuJoCo 모델(`damping 0.02, armature 7.4e-5, frictionloss 0.02`)과 함께 실측에 맞춘다.
2. 자유 베이스 `standup` 을 Newton 엔진에서도 확인하고, 낙하 직후 수동 구간의 차이(접촉 강성·마찰)를 맞춘다.

## 알려진 한계

- Isaac Sim API 는 버전마다 바뀐다. 6.x 에서 `isaacsim.core.api`(World)·`isaacsim.core.prims`(SingleArticulation)·`isaacsim.core.utils` 는
  **deprecated**(`extsDeprecated/`)이고, 이 백엔드는 대체 API(`isaacsim.core.experimental.*`, `SimulationManager`, `URDFImporter`)만 쓴다.
- 렌더링 없이(headless) 돌려도 Isaac Sim 은 시작에 약 12 s(첫 실행은 1–2 분: 셰이더·확장 캐시), Newton 은 첫 회 커널 컴파일이 더 걸린다. 짧은 실험은 MuJoCo 가 훨씬 빠르다.
- 속도: Fabric 을 켜도 실시간보다 1.3배(뷰어 1.9배) 느리다. 그래서 `--realtime` 페이싱은 의미가 없고(모든 틱이 기한 초과로 찍힌다),
  플랫폼도 Isaac 에는 페이싱을 걸지 않는다 (`realtime_backends`). 시뮬 시간축은 틱 기준이라 결과에는 영향이 없다. 뷰어에서는 약간 느린 동작으로 보인다.
- 렌더러 설정을 함부로 바꾸지 말 것: `--/rtx/hydra/supportMultiTickRate=false` 나 `/rtx/ecoMode/enabled=false` 를 주면 첫 프레임 이후 장면 갱신이 멈춘다.
- RL 학습(Isaac Lab / legged_gym) 은 이 백엔드의 범위가 아니다. Dobot 의 `dobot_rl_gym` 은 Isaac Gym Preview(구버전) 기반이다.
- 패키지에 NVIDIA 가 넣은 에이전트용 문서(`isaacsim/AGENTS.md`, `isaacsim/skills/*/SKILL.md`)가 6.1 API 참고 자료로 유용하다.
