cmake_minimum_required(VERSION 3.16)

project(app VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)

# Lumo Sensor SDK. Override with -DLUMO_SDK_ROOT=... or env LUMO_SDK_ROOT.
set(HF_DEFAULT_LUMO_SDK_ROOT "C:/Program Files (x86)/Specim/SDKs/SpecSensor/2020_519")

if(DEFINED ENV{LUMO_SDK_ROOT} AND NOT "$ENV{LUMO_SDK_ROOT}" STREQUAL "")
    set(LUMO_SDK_ROOT "$ENV{LUMO_SDK_ROOT}")
elseif(NOT LUMO_SDK_ROOT)
    set(LUMO_SDK_ROOT "${HF_DEFAULT_LUMO_SDK_ROOT}")
endif()

set(LUMO_SDK_ROOT "${LUMO_SDK_ROOT}" CACHE PATH "Specim Lumo Sensor SDK root (include/, bin/x64/)")

# Zaber Motion Library (stage backend — optional until adapters/zaber is added).
if(DEFINED ENV{ZML_ROOT} AND NOT "$ENV{ZML_ROOT}" STREQUAL "")
    set(ZML_ROOT "$ENV{ZML_ROOT}")
endif()
if(NOT ZML_ROOT)
    set(ZML_ROOT "C:/Program Files/Zaber Motion Library")
endif()
set(ZML_ROOT "${ZML_ROOT}" CACHE PATH "Zaber Motion Library root (include/, lib/Release/, bin/Release/)")
if(EXISTS "${ZML_ROOT}/include/zaber/motion/ascii.h")
    set(HF_HAVE_ZML ON)
    message(STATUS "Zaber Motion Library found at ${ZML_ROOT}")
else()
    set(HF_HAVE_ZML OFF)
    message(STATUS "Zaber Motion Library not found at ${ZML_ROOT}")
endif()

set(HF_HAVE_LUMO_SDK OFF)
if(LUMO_SDK_ROOT AND EXISTS "${LUMO_SDK_ROOT}/include/SI_sensor.h")
    set(HF_HAVE_LUMO_SDK ON)
    message(STATUS "Lumo SDK found at ${LUMO_SDK_ROOT}")
else()
    message(STATUS "Lumo SDK not found (LUMO_SDK_ROOT). Lumo adapter will build as stub.")
endif()

qt_add_executable(app
    src/main.cpp
    # frontend/widgets — QWidget subclasses only
    src/frontend/widgets/MainWindow.cpp
    src/frontend/widgets/DetectorCrosshairWidget.cpp
    src/frontend/widgets/ProfilePlotWidget.cpp
    src/frontend/widgets/StageAxisWidget.cpp
    src/frontend/widgets/IntensityBarWidget.cpp
    src/frontend/widgets/OperationWaitDialog.cpp
    src/frontend/widgets/WaterfallDisplayWidget.cpp
    src/frontend/widgets/StreamPaneHelpers.cpp
    src/frontend/widgets/CameraStreamTabBuilder.cpp
    # frontend/controllers — tab orchestration
    src/frontend/controllers/StagePanelController.cpp
    src/frontend/controllers/LightPanelController.cpp
    src/frontend/controllers/CameraPanelController.cpp
    src/frontend/controllers/UiSettingsController.cpp
    src/frontend/controllers/CapturePanelController.cpp
    src/frontend/widgets/MainWindowTabHelpers.cpp
    src/frontend/widgets/SettingsTabPanelBuilder.cpp
    src/frontend/widgets/CameraSettingsTab.cpp
    src/frontend/widgets/StageSettingsTab.cpp
    src/frontend/widgets/LightSettingsTab.cpp
    src/frontend/widgets/Ur3eSettingsTab.cpp
    src/frontend/widgets/CaptureSettingsTab.cpp
    src/frontend/streaming/FrameCoalescer.cpp
    src/frontend/streaming/CameraStreamPipeline.cpp
    src/frontend/processing/DetectorFrameProcessor.cpp
    src/frontend/processing/WavelengthLookup.cpp
    include/frontend/widgets/MainWindow.hpp
    include/frontend/widgets/OperationWaitDialog.hpp
    include/frontend/widgets/DetectorCrosshairWidget.hpp
    include/frontend/widgets/ProfilePlotWidget.hpp
    include/frontend/widgets/StageAxisWidget.hpp
    include/frontend/widgets/IntensityBarWidget.hpp
    include/frontend/widgets/LumoCameraUi.hpp
    include/frontend/widgets/WaterfallDisplayWidget.hpp
    include/frontend/widgets/StreamPaneHelpers.hpp
    include/frontend/widgets/CameraStreamTabBuilder.hpp
    include/frontend/controllers/StagePanelController.hpp
    include/frontend/controllers/LightPanelController.hpp
    include/frontend/controllers/CameraPanelController.hpp
    include/frontend/controllers/UiSettingsController.hpp
    include/frontend/controllers/CapturePanelController.hpp
    include/frontend/widgets/MainWindowTabHelpers.hpp
    include/frontend/widgets/SettingsTabPanelBuilder.hpp
    include/frontend/streaming/FrameCoalescer.hpp
    include/frontend/streaming/CameraStreamPipeline.hpp
    include/frontend/streaming/StreamFpsTracker.hpp
    include/frontend/processing/DetectorFrameProcessor.hpp
    include/frontend/processing/WavelengthLookup.hpp
    # frontend/processing — frame pipelines (worker threads, extractors)
    src/frontend/processing/BilProfileExtractor.cpp
    src/frontend/processing/DetectorFrameConverter.cpp
    src/frontend/processing/Overexposure.cpp
    src/frontend/processing/ProfileProcessor.cpp
    src/frontend/processing/RgbBandExtractor.cpp
    src/frontend/processing/WaterfallProcessor.cpp
    # frontend/utils — small UI helpers
    src/frontend/utils/SerialPortEnumerator.cpp
    # frontend/settings — persistence
    src/frontend/settings/AppSettingsStore.cpp
    include/frontend/settings/AppSettingsStore.hpp
    # backend — device-agnostic workers, orchestration, I/O
    src/backend/CameraWorker.cpp
    src/backend/SdkLifecycleRunner.cpp
    src/backend/StageWorker.cpp
    src/backend/LighthouseWorker.cpp
    src/backend/LumoDatasetWriter.cpp
    src/backend/CaptureWriterWorker.cpp
    src/backend/HyperFusionConfig.cpp
    src/backend/DualCameraScanOrchestrator.cpp
    src/backend/ICameraController.cpp
    src/backend/CameraCoordinator.cpp
    src/backend/processing/EnviBilReader.cpp
    src/backend/processing/ReferenceSpectrumStats.cpp
    src/backend/processing/ReferenceSpectrumPlot.cpp
    src/backend/processing/CapturePostProcessor.cpp
    src/backend/processing/CapturePostProcessorWorker.cpp
    src/backend/processing/Gsam2SegmentationClient.cpp
    src/backend/processing/Gsam2ServerManager.cpp
    src/backend/processing/Gsam2RoiAnalysis.cpp
    src/backend/processing/Gsam2SpectrumPlot.cpp
    src/backend/processing/GsamWslPathUtil.cpp
    src/backend/processing/IlluminantTables.cpp
    src/backend/processing/ReferenceBuilder.cpp
    src/backend/processing/EnviBilWriter.cpp
    src/backend/processing/FlatFieldCorrector.cpp
    src/backend/processing/HsiToColor.cpp
    include/backend/CameraTypes.hpp
    include/backend/ICameraController.hpp
    include/backend/CameraWorker.hpp
    include/backend/SdkLifecycleRunner.hpp
    include/backend/StageTypes.hpp
    include/backend/IStageController.hpp
    include/backend/StageWorker.hpp
    include/backend/LighthouseTypes.hpp
    include/backend/ILighthouseController.hpp
    include/backend/LighthouseWorker.hpp
    include/backend/CaptureWriterTypes.hpp
    include/backend/LumoDatasetWriter.hpp
    include/backend/CaptureWriterWorker.hpp
    include/backend/HyperFusionConfig.hpp
    include/backend/DualCameraScanOrchestrator.hpp
    include/backend/CameraCoordinator.hpp
    include/backend/processing/Gsam2ServerManager.hpp
    include/backend/processing/Gsam2SegmentationClient.hpp
    include/backend/processing/Gsam2RoiAnalysis.hpp
    include/backend/processing/Gsam2SpectrumPlot.hpp
    include/backend/processing/GsamWslPathUtil.hpp
    include/backend/processing/CapturePostProcessor.hpp
    # adapters — vendor SDK / hardware
    src/adapters/zaber/ZaberStageController.cpp
    src/adapters/zaber/ZaberStageConfigurator.cpp
    src/adapters/zaber/ZaberStageHoming.cpp
    src/adapters/zaber/ZaberStageMotion.cpp
    src/adapters/lumo/CalpackBandCatalog.cpp
    src/adapters/lumo/CalibrationPackPaths.cpp
    src/adapters/lumo/LumoCamera.cpp
    src/adapters/lumo/Swir3NiCamera.cpp
    src/adapters/mcc/Mcc1208LighthouseController.cpp
    src/adapters/mcc/MccUniversalLibrary.cpp
    include/adapters/zaber/ZaberStageController.hpp
    include/adapters/zaber/ZaberStageConfigurator.hpp
    include/adapters/zaber/ZaberStageHoming.hpp
    include/adapters/zaber/ZaberStageMotion.hpp
    include/adapters/zaber/ZaberStageProfile.hpp
    include/adapters/lumo/CalpackBandCatalog.hpp
    include/adapters/lumo/CalibrationPackPaths.hpp
    include/adapters/lumo/LumoCamera.hpp
    include/adapters/lumo/Swir3NiCamera.hpp
    include/adapters/lumo/LumoDeviceTypes.hpp
    include/adapters/mcc/Mcc1208LighthouseController.hpp
    include/adapters/mcc/Mcc1208Profile.hpp
    include/adapters/mcc/MccUniversalLibrary.hpp
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

    file(TO_CMAKE_PATH "${LUMO_SDK_ROOT}" HF_LUMO_SDK_ROOT_PATH)
    target_compile_definitions(app PRIVATE "HF_LUMO_SDK_ROOT=\"${HF_LUMO_SDK_ROOT_PATH}\"")

    set(_lumo_ni_icd_dir "${LUMO_SDK_ROOT}/external/NI")
    if(EXISTS "${_lumo_ni_icd_dir}")
        add_custom_command(
            TARGET app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${_lumo_ni_icd_dir}"
                    "$<TARGET_FILE_DIR:app>/external/NI"
            COMMENT "Copying Lumo NI ICD files to output directory")
    endif()

    set(_lumo_runtime_bin "${LUMO_SDK_ROOT}/bin/x64")
    if(EXISTS "${_lumo_runtime_bin}")
        file(GLOB _lumo_runtime_dlls "${_lumo_runtime_bin}/*.dll")
        foreach(_lumo_dll IN LISTS _lumo_runtime_dlls)
            add_custom_command(
                TARGET app POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${_lumo_dll}"
                        "$<TARGET_FILE_DIR:app>"
                COMMENT "Copying Lumo runtime DLL to output directory")
        endforeach()
    endif()
endif()

if(HF_HAVE_ZML)
    target_compile_definitions(app PRIVATE HF_HAVE_ZML)
    target_include_directories(app PRIVATE "${ZML_ROOT}/include")
    find_library(ZML_LIB zaber-motion PATHS "${ZML_ROOT}/lib/Release" NO_DEFAULT_PATH)
    if(ZML_LIB)
        target_link_libraries(app PRIVATE "${ZML_LIB}")
        message(STATUS "Linked Zaber Motion Library: ${ZML_LIB}")
    else()
        message(FATAL_ERROR "zaber-motion.lib not found under ${ZML_ROOT}/lib/Release")
    endif()

    foreach(_zml_dll IN ITEMS zaber-motion.dll zaber-motion-core.dll)
        set(_zml_dll_path "${ZML_ROOT}/bin/Release/${_zml_dll}")
        if(EXISTS "${_zml_dll_path}")
            add_custom_command(
                TARGET app POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${_zml_dll_path}"
                        "$<TARGET_FILE_DIR:app>/${_zml_dll}"
                COMMENT "Copying ${_zml_dll} to output directory")
        endif()
    endforeach()
endif()

set(HF_DEFAULT_MCC_UL_ROOT "C:/Program Files (x86)/Measurement Computing/DAQ")
if(DEFINED ENV{MCC_UL_ROOT} AND NOT "$ENV{MCC_UL_ROOT}" STREQUAL "")
    set(MCC_UL_ROOT "$ENV{MCC_UL_ROOT}")
elseif(NOT MCC_UL_ROOT)
    set(MCC_UL_ROOT "${HF_DEFAULT_MCC_UL_ROOT}")
endif()
set(MCC_UL_ROOT "${MCC_UL_ROOT}" CACHE PATH "MCC Universal Library runtime (cbw64.dll)")

if(EXISTS "${MCC_UL_ROOT}/cbw64.dll")
    message(STATUS "MCC Universal Library runtime found at ${MCC_UL_ROOT}")
    file(GLOB _mcc_runtime_dlls "${MCC_UL_ROOT}/*.dll")
    foreach(_mcc_dll IN LISTS _mcc_runtime_dlls)
        add_custom_command(
            TARGET app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_mcc_dll}"
                    "$<TARGET_FILE_DIR:app>"
            COMMENT "Copying MCC UL runtime DLL to output directory")
    endforeach()
else()
    message(STATUS "MCC UL runtime not found at ${MCC_UL_ROOT} (Light tab needs InstaCal/cbw64.dll at runtime)")
endif()

target_link_libraries(app PRIVATE Qt6::Widgets Qt6::Core Qt6::Network)

# Ship hyperfusion.cfg beside app.exe (hardware parameters).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hyperfusion.cfg")
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/hyperfusion.cfg"
                "$<TARGET_FILE_DIR:app>/hyperfusion.cfg"
        COMMENT "Copying hyperfusion.cfg to output directory")
endif()

# Ship calibration packs beside app.exe (RGB band dropdowns).
set(HF_CALIBRATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/calibration")
if(EXISTS "${HF_CALIBRATION_DIR}")
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${HF_CALIBRATION_DIR}"
                "$<TARGET_FILE_DIR:app>/calibration"
        COMMENT "Copying calibration packs to output directory")
endif()

set(HF_PROCESSING_REFERENCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/backend/processing/reference")
if(EXISTS "${HF_PROCESSING_REFERENCE_DIR}")
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:app>/backend/processing/reference"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${HF_PROCESSING_REFERENCE_DIR}/D_illuminants.json"
                "$<TARGET_FILE_DIR:app>/backend/processing/reference/D_illuminants.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${HF_PROCESSING_REFERENCE_DIR}/D_illuminants.mat"
                "$<TARGET_FILE_DIR:app>/backend/processing/reference/D_illuminants.mat"
        COMMENT "Copying preprocessing illuminant reference tables")
endif()

file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" HF_APP_SOURCE_DIR)
target_compile_definitions(app PRIVATE "HF_APP_SOURCE_DIR=\"${HF_APP_SOURCE_DIR}\"")
