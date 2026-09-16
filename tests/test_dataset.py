"""데이터 계층(lowlevel.dataset) 검증: 로봇·관절 수가 다른 기록의 로드, 목록, 시계열, 비교(공통 관절), 상대 모드, 다중 비교.

    python tests/test_dataset.py
"""
import math
import sys
import tempfile
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lowlevel.common import State                                          # noqa: E402
from lowlevel.dataset import (compare_many, compare_runs, driven_names, list_runs,   # noqa: E402
                              load_run, read_meta, resolve_joints, series, update_meta, write_meta)
from lowlevel.runtime import TraceLog                                      # noqa: E402


def make_run(root: Path, name: str, joints, source, robot, driven, n=400, dt=0.005, offset=0.0, amp_scale=1.0):
    out = root / name
    write_meta(out, source=source, program="sine", robot=robot, joints=joints, dt=dt, params={"amp": 0.2})
    log = TraceLog(out / "trace.csv", joints=joints)
    nj = len(joints)
    for i in range(n):
        t = i * dt
        settle = t < 0.5
        qdes = np.full(nj, offset)
        q = np.full(nj, offset)
        for j, jn in enumerate(joints):
            if jn in driven and not settle:
                qdes[j] = offset + 0.2 * math.sin(2 * math.pi * 1.0 * (t - 0.5))
                q[j] = offset + 0.2 * amp_scale * math.sin(2 * math.pi * 1.0 * (t - 0.5 - 0.02))
        st = State(t=t, q=q, dq=np.zeros(nj), quat_wxyz=np.array([1.0, 0, 0, 0]), gyro=np.zeros(3), acc=np.zeros(3),
                   tau_est=np.full(nj, 0.5), age=0.0)
        cmd = types.SimpleNamespace(q=qdes, kp=np.full(nj, 0.0 if settle else 30.0), kd=np.full(nj, 1.0))
        log.row(t, "settle" if settle else "sine", st, cmd)
    log.close()
    update_meta(out, status="completed", ticks=n)
    return out


def main():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        a = make_run(root, "a-sim", ["j1", "j2", "j3"], "mujoco", "rover", driven={"j2"})
        b = make_run(root, "b-real", ["j2", "j3", "j4"], "dds", "rover", driven={"j2"}, offset=0.5, amp_scale=0.6)
        c = make_run(root, "c-humanoid", ["arm1", "arm2"], "mujoco", "atom_upper", driven={"arm1"})

        runs = list_runs(root)
        assert {r["id"] for r in runs} == {"a-sim", "b-real", "c-humanoid"}
        by = {r["id"]: r for r in runs}
        assert by["a-sim"]["label"] == "sim" and by["b-real"]["label"] == "real"
        assert by["c-humanoid"]["robot"] == "atom_upper" and by["c-humanoid"]["n_joints"] == 2
        assert by["a-sim"]["ticks"] == 400

        ra, rb, rc = load_run(a), load_run(b), load_run(c)
        assert ra.joints == ["j1", "j2", "j3"] and rc.joints == ["arm1", "arm2"]
        assert ra.q.shape == (400, 3) and ra.gyro.shape == (400, 3)
        assert driven_names(ra) == ["j2"] and driven_names(rc) == ["arm1"]
        assert resolve_joints(ra, "j3,j1") == [2, 0] and resolve_joints(ra, [1]) == [1]
        assert read_meta(a)["joints"] == ["j1", "j2", "j3"]

        s = series(ra, ["j2"], max_points=100)
        assert list(s["series"]) == ["j2"] and len(s["t"]) <= 100 and s["phases"][0]["phase"] == "settle"

        m = compare_runs(ra, rb)                                   # 공통 관절 j2, j3 (구동은 j2)
        assert m["joints"] == ["j2"], m["joints"]
        pj = m["per_joint"]["j2"]
        assert 0.4 < pj["rmse_ab"] < 0.6, pj["rmse_ab"]           # 오프셋 0.5 차이가 지배
        assert pj["corr_ab"] > 0.9
        assert 0.55 < pj["amp_ratio_b"] < 0.65 and 0.95 < pj["amp_ratio_a"] < 1.05
        assert pj["lag_a_ms"] is not None and 10 <= pj["lag_a_ms"] <= 30
        mr = compare_runs(ra, rb, relative=True)
        assert mr["per_joint"]["j2"]["rmse_ab"] < 0.1              # 시작 자세를 빼면 작아진다
        m2 = compare_runs(ra, rb, joints="j3,j2")
        assert m2["joints"] == ["j3", "j2"] and [p["phase"] for p in m2["per_phase"]] == ["settle", "sine"]
        try:
            compare_runs(ra, rc)
            raise AssertionError("공통 관절이 없으면 ValueError")
        except ValueError:
            pass
        many = compare_many([ra, rb, ra], relative=True)
        assert len(many["pairs"]) == 2 and set(many["series"]) == {"a-sim", "b-real"}
    print("test_dataset: OK")


if __name__ == "__main__":
    main()
