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

add_executable(orbbec src/cli/main.cpp)
# add_executable(orbbec src/module/main.cpp)

set(OrbbecSDK_DIR lib/orbbecsdk/lib)
include_directories(include)
find_package(OrbbecSDK REQUIRED
    PATHS lib/
    NO_DEFAULT_PATH)
find_package(viam-cpp-sdk REQUIRED)

# find_package(OpenCV REQUIRED)
# target_link_libraries(orbbec ob::OrbbecSDK ${OpenCV_LIBS})
target_link_libraries(orbbec 
PRIVATE ob::OrbbecSDK
PRIVATE viam-cpp-sdk::viamsdk
)
