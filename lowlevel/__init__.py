"""Rover(Dobot Quad) 저수준 관절 제어 실습 패키지.

- common.py          관절 순서·하드웨어 매핑·한계값·State/JointCmd
- safety.py          워치독·넘어짐·범위·변화율 가드
- programs.py        standup / sine / hold / damp 목표 생성
- backend_mujoco.py  MuJoCo(dobot_sim2real 모델) 백엔드
- backend_dds.py     실기 DDS(rt/lower/cmd, rt/lower/state) 백엔드
"""
