cmake_minimum_required(VERSION 3.10)
project(orbbec)


set(CMAKE_PROJECT_VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
project(tflite_cpu
    DESCRIPTION "Viam module for tflite inference  on the CPU"
    HOMEPAGE_URL https://github.com/viam-modules/orbbec
    LANGUAGES CXX
)

set(BUILD_SHARED_LIBS OFF)

# add_executable(orbbec-cli src/cli/main.cpp)
add_executable(orbbec-module src/module/main.cpp)

set(OrbbecSDK_DIR lib/orbbecsdk/lib)
include_directories(include)
find_package(OrbbecSDK REQUIRED
    PATHS lib/
    NO_DEFAULT_PATH)
find_package(viam-cpp-sdk REQUIRED)
find_package(Threads REQUIRED)

# target_link_libraries(orbbec-cli
# PRIVATE ob::OrbbecSDK
# )

target_link_libraries(orbbec-module
PRIVATE ob::OrbbecSDK
PRIVATE Threads::Threads
PRIVATE viam-cpp-sdk::viamsdk
)

