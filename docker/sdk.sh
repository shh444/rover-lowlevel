#!/usr/bin/env bash
# rover-sdk 컨테이너 안에서 명령 실행 (호스트 네트워크, 저장소를 /work 에 마운트, 호스트 사용자 권한).
#   docker/sdk.sh python3 tests/test_dds_mapping.py
#   docker/sdk.sh bash -c "source docker/dds_env.sh && python3 tools/preflight_real.py --seconds 5"
#   docker/sdk.sh python3 run.py --backend dds --program damp --duration 3
# CYCLONEDDS_URI 는 컨테이너 안 경로(/work/...)여야 한다. 컨테이너 안에서 source docker/dds_env.sh 로 만든다.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=${IMAGE:-rover-sdk:0.23.3}
VENDOR=${VENDOR:-$HOME/rover-mujoco-poc/vendor}     # dobot_rover_simulation 이 있는 폴더 → /work/vendor (읽기 전용)
TTY_FLAG="-i"
[ -t 0 ] && TTY_FLAG="-it"
VENDOR_MOUNT=()
[ -d "$VENDOR" ] && VENDOR_MOUNT=(-v "$VENDOR:/work/vendor:ro")
exec docker run --rm $TTY_FLAG --network host --user "$(id -u):$(id -g)" -e HOME=/work \
    -v "$ROOT:/work" "${VENDOR_MOUNT[@]}" -w /work \
    -e CYCLONEDDS_URI -e PYTHONPATH -e FAKE_DDS_STOP_AT -e FAKE_DDS_TILT_AT -e FAKE_DDS_REPORT -e FAKE_DDS_XML \
    "$IMAGE" "$@"
