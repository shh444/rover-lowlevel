# 예제 전문

`examples/` 의 스크립트 전체다. 모두 저장소 루트에서 실행하며, `--backend mujoco`(기본)는 로봇 없이 돌고
`--backend dds` 는 실기 게이트(주 제어기 검사·체크리스트)를 거친다. 사용법은 {doc}`USAGE` 를 본다.

| 예제 | 내용 | 실기 |
|---|---|---|
| e1_state_monitor | 상태 읽기만 (writer 없음) | ✅ |
| e2_damping | 댐핑 명령만 (kp 0, kd 3) | ✅ 2026-09-16 |
| e3_hold_pose | 현재 자세 유지, kp 램프 | ✅ 2026-09-16 |
| e4_sine_joint | E9 식 사인 (thigh ±0.1 rad) | ✅ 2026-09-16 |
| e5_standup | sim2real 3단계 기립 → 엎드림 | 시뮬만 |
| e6_raw_sdk_sine | 패키지 없이 SDK API 만으로 (실기 전용) | 문법·import |
| e7_custom_program | 나만의 Program 클래스 (앞발 들기) | 시뮬·가짜 DDS |
| e8_squat | 서 있는 자세에서 자세 보간 | 시뮬만 |
| e9_gravity_feedforward | tau 피드포워드, 몸통 고정 모드 | 시뮬만 |
| e10_plot_trace | trace.csv 그래프 | 실기 기록 |

## e1_state_monitor.py

```{literalinclude} ../examples/e1_state_monitor.py
:language: python
```

## e2_damping.py

```{literalinclude} ../examples/e2_damping.py
:language: python
```

## e3_hold_pose.py

```{literalinclude} ../examples/e3_hold_pose.py
:language: python
```

## e4_sine_joint.py

```{literalinclude} ../examples/e4_sine_joint.py
:language: python
```

## e5_standup.py

```{literalinclude} ../examples/e5_standup.py
:language: python
```

## e6_raw_sdk_sine.py

```{literalinclude} ../examples/e6_raw_sdk_sine.py
:language: python
```

## e7_custom_program.py

```{literalinclude} ../examples/e7_custom_program.py
:language: python
```

## e8_squat.py

```{literalinclude} ../examples/e8_squat.py
:language: python
```

## e9_gravity_feedforward.py

```{literalinclude} ../examples/e9_gravity_feedforward.py
:language: python
```

## e10_plot_trace.py

```{literalinclude} ../examples/e10_plot_trace.py
:language: python
```
