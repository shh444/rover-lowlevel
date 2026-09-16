# Dobot Quad SDK 저수준(DDS) 실행 환경: Ubuntu 22.04 + CPython 3.10 + dds_middleware 0.23.3
# docker/sdk/ 에는 실행할 CPU 아키텍처에 맞는 deb 와 whl 한 쌍만 둔다 (thor: arm64 / aarch64).
# 빌드: docker build -t rover-sdk:0.23.3 docker/
# 실행: docker/sdk.sh python3 run.py --backend dds --program damp --duration 3
# C++ 도구(가상 로봇 tools/virtual_robot_dds) 컴파일을 위해 build-essential, cmake, libyaml-cpp-dev, libboost-dev 도
# 포함한다 (SDK 의 CycloneDDS-CXX cmake 설정이 Boost 헤더를 요구한다).
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip libyaml-cpp0.7 libyaml-cpp-dev libboost-dev libstdc++6 build-essential cmake \
        iproute2 iputils-ping ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY sdk/ /tmp/sdk/
RUN dpkg -i /tmp/sdk/dds-middleware-with-thirdparty_0.23.3_*.deb \
    && pip3 install --no-cache-dir /tmp/sdk/dds_middleware_python-0.23.3-cp310-cp310-linux_*.whl \
       "numpy<2.3" pyyaml mujoco \
    && rm -rf /tmp/sdk
ENV CYCLONEDDS_HOME=/dobot_algs/middleware \
    LD_LIBRARY_PATH=/dobot_algs/middleware/lib \
    CMAKE_PREFIX_PATH=/dobot_algs/middleware \
    PATH=/dobot_algs/middleware/bin:/usr/local/bin:/usr/bin:/bin \
    MUJOCO_GL=disable \
    PYTHONUNBUFFERED=1
WORKDIR /work
CMD ["/bin/bash"]
