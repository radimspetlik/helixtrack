cmake_minimum_required(VERSION 3.20)

# FindHDF5 probes both the C and C++ toolchains during configuration.
project(helixtrack LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
# OpenEB installs the MetavisionSDK CMake packages and targets used below.
find_package(MetavisionSDK REQUIRED COMPONENTS base core stream ui)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(helixtrack
    main.cpp
    metrics_writer.cpp
    playback_plan.cpp
    recording_setup.cpp
)

# The OpenEB image used for HelixTrack does not populate CMake's default
# release optimization flags, so add explicit per-config optimization levels
# to keep runtime behavior aligned with the original ptbd build.
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(helixtrack PRIVATE
        $<$<CONFIG:Debug>:-g>
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
        $<$<CONFIG:MinSizeRel>:-Os -DNDEBUG>
    )
endif()

target_include_directories(helixtrack PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${HDF5_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(helixtrack PRIVATE
    Eigen3::Eigen
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    MetavisionSDK::ui
    Threads::Threads
    yaml-cpp
    ${HDF5_LIBRARIES}
    ${OpenCV_LIBS}
)
