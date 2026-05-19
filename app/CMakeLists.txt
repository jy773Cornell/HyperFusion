cmake_minimum_required(VERSION 3.16)

project(app VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

# Lumo Sensor SDK. Override with -DLUMO_SDK_ROOT=... or env LUMO_SDK_ROOT.
set(HF_DEFAULT_LUMO_SDK_ROOT "C:/Program Files (x86)/Specim/SDKs/SpecSensor/2020_519")

if(DEFINED ENV{LUMO_SDK_ROOT} AND NOT "$ENV{LUMO_SDK_ROOT}" STREQUAL "")
    set(LUMO_SDK_ROOT "$ENV{LUMO_SDK_ROOT}")
elseif(NOT LUMO_SDK_ROOT)
    set(LUMO_SDK_ROOT "${HF_DEFAULT_LUMO_SDK_ROOT}")
endif()

set(LUMO_SDK_ROOT "${LUMO_SDK_ROOT}" CACHE PATH "Specim Lumo Sensor SDK root (include/, bin/x64/)")

set(HF_HAVE_LUMO_SDK OFF)
if(LUMO_SDK_ROOT AND EXISTS "${LUMO_SDK_ROOT}/include/SI_sensor.h")
    set(HF_HAVE_LUMO_SDK ON)
    message(STATUS "Lumo SDK found at ${LUMO_SDK_ROOT}")
else()
    message(STATUS "Lumo SDK not found (LUMO_SDK_ROOT). Lumo adapter will build as stub.")
endif()

qt_add_executable(app
    src/main.cpp
    src/MainWindow.cpp
    src/core/CameraWorker.cpp
    src/adapters/lumo/LumoCamera.cpp
    src/orchestrator/CameraCoordinator.cpp
    include/MainWindow.hpp
    include/core/CameraTypes.hpp
    include/core/ICameraController.hpp
    include/core/CameraWorker.hpp
    include/adapters/lumo/LumoCamera.hpp
    include/adapters/lumo/LumoDeviceTypes.hpp
    include/orchestrator/CameraCoordinator.hpp
)

target_include_directories(app PRIVATE include)

if(HF_HAVE_LUMO_SDK)
    target_compile_definitions(app PRIVATE HF_HAVE_LUMO_SDK)
    target_include_directories(app PRIVATE "${LUMO_SDK_ROOT}/include")
    find_library(LUMO_SPEC_SENSOR_LIB SpecSensor
        PATHS
            "${LUMO_SDK_ROOT}/bin/x64"
            "${LUMO_SDK_ROOT}/bin/Win32"
            "${LUMO_SDK_ROOT}/lib"
            "${LUMO_SDK_ROOT}/Lib"
        NO_DEFAULT_PATH)
    if(LUMO_SPEC_SENSOR_LIB)
        target_link_libraries(app PRIVATE "${LUMO_SPEC_SENSOR_LIB}")
        message(STATUS "Linked Lumo library: ${LUMO_SPEC_SENSOR_LIB}")
    else()
        message(FATAL_ERROR "SpecSensor.lib not found under ${LUMO_SDK_ROOT} (checked bin/x64, bin/Win32, lib)")
    endif()
endif()

target_link_libraries(app PRIVATE Qt6::Widgets)
