# Isaac Sim 백엔드 (실험적)

NVIDIA Isaac Sim 5.x 를 MuJoCo 와 같은 인터페이스(`wait_ready / read / send / close`)의 백엔드로 추가했다
(`lowlevel/backend_isaac.py`). 프로그램·가드·기록·플랫폼은 그대로 쓰고 `--backend isaac` 만 바꾼다.

**상태: 코드는 작성했지만 아직 실행 검증을 하지 않았다.** Isaac Sim 이 설치된 RTX PC 에서 `tools/isaac_smoke.py` 로 먼저 확인해야 한다.

## 요구 사항

| 항목 | 내용 |
|---|---|
| 하드웨어 | x86_64, NVIDIA RTX GPU (VRAM 8 GB 이상 권장), 드라이버 570 이상. 이 노트북(RTX 5070 Laptop 8 GB, 드라이버 610)은 조건을 만족한다. Jetson(thor, aarch64)에서는 돌지 않는다 |
| 소프트웨어 | Isaac Sim 5.x pip 패키지는 **Python 3.11** 이 필요하다 (이 저장소의 기본 venv 는 3.12). 다운로드 약 10 GB, 첫 실행 시 EULA 동의 |

```powershell
# Python 3.11 설치 후 (python.org 또는 winget install Python.Python.3.11)
py -3.11 -m venv .venv-isaac
.venv-isaac\Scripts\pip install "isaacsim[all,extscache]==5.*" --extra-index-url https://pypi.nvidia.com
.venv-isaac\Scripts\pip install numpy
$env:OMNI_KIT_ACCEPT_EULA = "YES"
.venv-isaac\Scripts\python tools\isaac_smoke.py --viewer
```

URDF 는 `vendor/dobot_rover_simulation/dobot_rl_gym/resources/robots/dobot/urdf/dobot_quad_ros.urdf` 를 쓴다 (`--urdf` 로 지정 가능).

## 동작 방식

1. `SimulationApp` 을 띄우고 `World(physics_dt=1 ms)` 를 만든 뒤 URDF 를 import 한다. 관절 드라이브는 끈다(게인 0).
2. 관절 순서는 이름으로 맞춘다 (`JOINT_NAMES`). MuJoCo 모델과 같은 URDF 이름이다.
3. `send(cmd)` 는 substep 마다 SDK 공식 `tau = kp(q_des−q) + kd(dq_des−dq) + tau_ff` 를 계산해 URDF effort 한계(23/23/55 N·m)로 자르고
   `set_joint_efforts` 로 넣은 뒤 `world.step()` 한다. MuJoCo 백엔드와 같은 방식이라 두 시뮬레이터의 차이는 물리 엔진 차이만 남는다.
4. IMU: 몸통 프림의 자세(wxyz)와 각속도(몸체 좌표계로 변환). 가속도는 정지 상태 근사(중력 반작용)다.
5. `--fixed-base` 면 몸통을 공중에 고정한다 (지지된 로봇 시험). 기본 자세는 MuJoCo 와 같다.

## 사용

```bash
.venv-isaac\Scripts\python run.py --backend isaac --program standup --duration 3 --viewer
.venv-isaac\Scripts\python examples\e4_sine_joint.py --backend isaac     # make_backend("isaac") 도 지원
```

플랫폼(`datalab/`)에서 쓰려면 `datalab/config.json` 의 `commands` 에 `"isaac": ["<.venv-isaac 의 python 경로>", "run.py"]` 를 넣는다.
그러면 수집 탭의 대상에 Isaac Sim 이 나타나고, 기록의 출처는 `isaac`(초록) 으로 표시되어 MuJoCo(`sim`)·실기(`real`)와 나란히 비교할 수 있다.

## 검증 계획

1. `tools/isaac_smoke.py`: URDF import, 관절 매핑, 댐핑 2 s, 기본 자세 유지 3 s (몸통 고정). 유지 오차 0.1 rad 이하면 통과.
2. `run.py --backend isaac --program sine --joints thigh --amp 0.1 --fixed-base` 를 돌리고, 같은 조건의 MuJoCo 기록·실기 기록과 datalab 비교 탭에서 비교.
3. 차이가 크면 URDF 의 `<dynamics damping/friction>`, Isaac 의 solver 설정(iteration, contact offset), 관절 armature 를 MuJoCo 모델(`damping 0.02, armature 7.4e-5, frictionloss 0.02`)에 맞춘다.

## 알려진 한계

- Isaac Sim API 는 버전마다 이름이 바뀐다 (`isaacsim.core.prims.SingleArticulation`, `isaacsim.asset.importer.urdf`, `URDFParseAndImportFile`).
  코드는 5.0/5.1 기준이며 import 실패 시 메시지에 힌트를 남긴다.
- 렌더링 없이(headless) 돌려도 Isaac Sim 은 시작에 수십 초가 걸린다. 짧은 실험은 MuJoCo 가 훨씬 빠르다.
- RL 학습(Isaac Lab / legged_gym) 은 이 백엔드의 범위가 아니다. Dobot 의 `dobot_rl_gym` 은 Isaac Gym Preview(구버전) 기반이다.
