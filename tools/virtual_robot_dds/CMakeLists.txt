cmake_minimum_required(VERSION 3.16)
project(virtual_robot_dds LANGUAGES CXX)
# SDK low_level/cpp/CMakeLists.txt 와 같은 방식. deb 는 /dobot_algs/middleware 에 설치된다.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SDK_ROOT /dobot_algs/middleware CACHE PATH "dds_middleware 설치 경로")
list(APPEND CMAKE_PREFIX_PATH ${SDK_ROOT})

find_library(DDS_MIDDLEWARE_LIB dds_middleware PATHS ${SDK_ROOT}/lib REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

include_directories(${SDK_ROOT}/include ${SDK_ROOT}/include/ddscxx)
if(EXISTS ${SDK_ROOT}/include/iceoryx/v2.0.0)
    include_directories(${SDK_ROOT}/include/iceoryx/v2.0.0)
endif()

add_executable(virtual_robot virtual_robot.cc)
target_link_libraries(virtual_robot PRIVATE ${DDS_MIDDLEWARE_LIB} CycloneDDS-CXX::ddscxx yaml-cpp Threads::Threads)
message(STATUS "dds_middleware: ${DDS_MIDDLEWARE_LIB}")
