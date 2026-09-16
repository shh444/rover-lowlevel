#!/usr/bin/env bash
# rover-sdk 컨테이너 안에서 C++ 가상 로봇 빌드:  docker/sdk.sh bash tools/virtual_robot_dds/build.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HERE/build" -j
ls -la "$HERE/build/virtual_robot"
