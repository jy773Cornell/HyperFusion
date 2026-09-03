cmake_minimum_required(VERSION 3.16)

project(app VERSION 0.1 LANGUAGES C CXX)

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

# Texas Instruments DLPC-API (DLP3010EVM-LC / DLPC3478). Override with -DDLPC_API_ROOT=... or env.
set(HF_DEFAULT_DLPC_API_ROOT "C:/ti/DLPC-API-1.12")
if(DEFINED ENV{DLPC_API_ROOT} AND NOT "$ENV{DLPC_API_ROOT}" STREQUAL "")
    set(DLPC_API_ROOT "$ENV{DLPC_API_ROOT}")
elseif(NOT DLPC_API_ROOT)
    set(DLPC_API_ROOT "${HF_DEFAULT_DLPC_API_ROOT}")
endif()
if(NOT EXISTS "${DLPC_API_ROOT}/api/dlpc34xx.h")
    set(_hf_dlpc_onedrive
        "$ENV{USERPROFILE}/OneDrive - Cornell University/Documents/Texas Instruments/DLPC-API-1.12")
    if(EXISTS "${_hf_dlpc_onedrive}/api/dlpc34xx.h")
        set(DLPC_API_ROOT "${_hf_dlpc_onedrive}")
    endif()
endif()
set(DLPC_API_ROOT "${DLPC_API_ROOT}" CACHE PATH "TI DLPC-API root (api/, samples/, third_party/cyusbserial/)")
if(EXISTS "${DLPC_API_ROOT}/api/dlpc34xx.h"
   AND EXISTS "${DLPC_API_ROOT}/api/dlpc_common.c"
   AND EXISTS "${DLPC_API_ROOT}/third_party/cyusbserial/cyusbserial.lib")
    set(HF_HAVE_DLPC_API ON)
    message(STATUS "DLPC-API found at ${DLPC_API_ROOT}")
else()
    set(HF_HAVE_DLPC_API OFF)
    message(STATUS "DLPC-API not found (DLPC_API_ROOT). DLP adapter will build as stub.")
endif()

# Teledyne Spinnaker SDK (Blackfly S / BFS RGB). Override with -DSPINNAKER_ROOT=... or env.
set(HF_DEFAULT_SPINNAKER_ROOT "C:/Program Files/Teledyne/Spinnaker")
if(DEFINED ENV{SPINNAKER_ROOT} AND NOT "$ENV{SPINNAKER_ROOT}" STREQUAL "")
    set(SPINNAKER_ROOT "$ENV{SPINNAKER_ROOT}")
elseif(NOT SPINNAKER_ROOT)
    set(SPINNAKER_ROOT "${HF_DEFAULT_SPINNAKER_ROOT}")
endif()
set(SPINNAKER_ROOT "${SPINNAKER_ROOT}" CACHE PATH "Teledyne Spinnaker SDK root (include/, lib64/vs2015/, bin64/vs2015/)")
if(EXISTS "${SPINNAKER_ROOT}/include/Spinnaker.h")
    set(HF_HAVE_SPINNAKER ON)
    message(STATUS "Spinnaker SDK found at ${SPINNAKER_ROOT}")
else()
    set(HF_HAVE_SPINNAKER OFF)
    message(STATUS "Spinnaker SDK not found (SPINNAKER_ROOT). BFS adapter will build as stub.")
endif()

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

qt_add_executable(app WIN32
    src/main.cpp
    resources/app.qrc
    # frontend/widgets — QWidget subclasses only
    src/frontend/widgets/MainWindow.cpp
    src/frontend/widgets/DetectorCrosshairWidget.cpp
    src/frontend/widgets/ProfilePlotWidget.cpp
    src/frontend/widgets/StageAxisWidget.cpp
    src/frontend/widgets/IntensityBarWidget.cpp
    src/frontend/widgets/OperationWaitDialog.cpp
    src/frontend/widgets/CaptureSessionSummaryDialog.cpp
    src/frontend/widgets/Ur3eExternalControlWaitDialog.cpp
    src/frontend/widgets/WaterfallDisplayWidget.cpp
    src/frontend/widgets/StreamPaneHelpers.cpp
    src/frontend/widgets/CameraStreamTabBuilder.cpp
    # frontend/controllers — tab orchestration
    src/frontend/controllers/StagePanelController.cpp
    src/frontend/controllers/LightPanelController.cpp
    src/frontend/controllers/CameraPanelController.cpp
    src/frontend/controllers/UiSettingsController.cpp
    src/frontend/controllers/CapturePanelController.cpp
    src/frontend/controllers/Ur3ePanelController.cpp
    src/frontend/controllers/BfsPanelController.cpp
    src/frontend/controllers/DlpPanelController.cpp
    src/frontend/logging/AppLog.cpp
    src/frontend/logging/AppLogSession.cpp
    src/frontend/widgets/MainWindowTabHelpers.cpp
    src/frontend/widgets/SettingsTabPanelBuilder.cpp
    src/frontend/widgets/CameraSettingsTab.cpp
    src/frontend/widgets/StageSettingsTab.cpp
    src/frontend/widgets/LightSettingsTab.cpp
    src/frontend/widgets/Ur3eJointBarWidget.cpp
    src/frontend/widgets/Ur3eScanRoutePlanWidget.cpp
    src/frontend/widgets/Ur3eHemisphereScanSettingsWidget.cpp
    src/frontend/widgets/Ur3eHemisphereScanPreviewWidget.cpp
    src/frontend/widgets/Ur3eStreamTab.cpp
    src/frontend/widgets/Ur3eSettingsTab.cpp
    src/frontend/widgets/BfsCameraSettingsWidget.cpp
    src/frontend/widgets/DlpProjectorSettingsWidget.cpp
    src/frontend/widgets/CaptureSettingsTab.cpp
    src/frontend/streaming/FrameCoalescer.cpp
    src/frontend/streaming/CameraStreamPipeline.cpp
    src/frontend/processing/DetectorFrameProcessor.cpp
    src/frontend/processing/WavelengthLookup.cpp
    include/frontend/widgets/MainWindow.hpp
    include/frontend/widgets/OperationWaitDialog.hpp
    include/frontend/widgets/CaptureSessionSummaryDialog.hpp
    include/frontend/widgets/Ur3eExternalControlWaitDialog.hpp
    include/frontend/widgets/DetectorCrosshairWidget.hpp
    include/frontend/widgets/ProfilePlotWidget.hpp
    include/frontend/widgets/StageAxisWidget.hpp
    include/frontend/widgets/IntensityBarWidget.hpp
    include/frontend/widgets/Ur3eScanRoutePlanWidget.hpp
    include/frontend/widgets/Ur3eHemisphereScanSettingsWidget.hpp
    include/frontend/widgets/Ur3eHemisphereScanPreviewWidget.hpp
    include/frontend/widgets/Ur3eJointBarWidget.hpp
    include/frontend/widgets/BfsCameraSettingsWidget.hpp
    include/frontend/widgets/DlpProjectorSettingsWidget.hpp
    include/frontend/widgets/LumoCameraUi.hpp
    include/frontend/widgets/WaterfallDisplayWidget.hpp
    include/frontend/widgets/StreamPaneHelpers.hpp
    include/frontend/widgets/CameraStreamTabBuilder.hpp
    include/frontend/controllers/StagePanelController.hpp
    include/frontend/controllers/LightPanelController.hpp
    include/frontend/controllers/CameraPanelController.hpp
    include/frontend/controllers/UiSettingsController.hpp
    include/frontend/controllers/CapturePanelController.hpp
    include/frontend/controllers/Ur3ePanelController.hpp
    include/frontend/controllers/BfsPanelController.hpp
    include/frontend/controllers/DlpPanelController.hpp
    include/frontend/logging/AppLog.hpp
    include/frontend/logging/AppLogSession.hpp
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
    src/frontend/utils/AppBranding.cpp
    # frontend/settings — persistence
    src/frontend/settings/AppSettingsStore.cpp
    include/frontend/settings/AppSettingsStore.hpp
    # backend — device-agnostic workers, orchestration, I/O
    src/backend/camera/CameraWorker.cpp
    src/backend/camera/SdkLifecycleRunner.cpp
    src/backend/stage/StageWorker.cpp
    src/backend/light/LighthouseWorker.cpp
    src/backend/camera/LumoDatasetWriter.cpp
    src/backend/camera/CaptureWriterWorker.cpp
    src/backend/HyperFusionConfig.cpp
    src/backend/camera/DualCameraScanOrchestrator.cpp
    src/backend/camera/ICameraController.cpp
    src/backend/camera/CameraCoordinator.cpp
    src/backend/camera/processing/EnviBilReader.cpp
    src/backend/camera/processing/ReferenceSpectrumStats.cpp
    src/backend/camera/processing/ReferenceSpectrumPlot.cpp
    src/backend/camera/processing/CapturePostProcessor.cpp
    src/backend/camera/processing/CapturePostProcessorWorker.cpp
    src/backend/camera/processing/HfFusionRunner.cpp
    src/backend/camera/processing/HfFusionWorker.cpp
    src/backend/camera/processing/Gsam2SegmentationClient.cpp
    src/backend/camera/processing/GsamPlan.cpp
    src/backend/camera/processing/GsamPlanSegmentRunner.cpp
    src/backend/camera/processing/Gsam2ServerManager.cpp
    src/backend/camera/processing/Gsam2RoiAnalysis.cpp
    src/backend/camera/processing/Gsam2SpectrumPlot.cpp
    src/backend/camera/processing/GsamWslPathUtil.cpp
    src/backend/multiview/Ur3eClient.cpp
    src/backend/multiview/Ur3eHemisphereScan.cpp
    src/backend/multiview/Ur3eHomeScanCenter.cpp
    src/backend/multiview/Ur3eHemisphereScanReachability.cpp
    src/backend/multiview/Ur3eWorkspaceBoundary.cpp
    src/backend/multiview/Ur3eMountTransform.cpp
    src/backend/multiview/Ur3eMoveItManager.cpp
    src/backend/multiview/Ur3eRvizManager.cpp
    src/backend/multiview/Ur3eServerManager.cpp
    src/backend/multiview/Ur3eWslSetup.cpp
    src/backend/multiview/Ur3eWslLogUtil.cpp
    src/backend/multiview/Ur3eWslPathUtil.cpp
    src/backend/multiview/BfsSpinnakerCamera.cpp
    src/backend/multiview/BfsCameraWorker.cpp
    src/backend/fpp/DlpProjectorWorker.cpp
    src/backend/multiview/BfsTiffIo.cpp
    src/backend/multiview/Ur3eCameraTransforms.cpp
    src/backend/multiview/Ur3eScanPlanCache.cpp
    src/backend/multiview/Ur3eSemiFixedScan.cpp
    src/backend/multiview/Ur3eSemiFixedScanExecute.cpp
    src/backend/multiview/Ur3eAutoHemisphereScanExecute.cpp
    src/backend/camera/processing/IlluminantTables.cpp
    src/backend/camera/processing/ReferenceBuilder.cpp
    src/backend/camera/processing/EnviBilWriter.cpp
    src/backend/camera/processing/FlatFieldCorrector.cpp
    src/backend/camera/processing/HsiToColor.cpp
    src/backend/camera/processing/SwirBprCorrector.cpp
    src/backend/camera/processing/SwirBprCalpackMap.cpp
    src/backend/camera/processing/SwirBprApply.cpp
    src/backend/camera/processing/SwirColumnProfileCorrector.cpp
    src/backend/camera/processing/SwirRefBprCorrector.cpp
    include/backend/camera/CameraTypes.hpp
    include/backend/camera/ICameraController.hpp
    include/backend/camera/CameraWorker.hpp
    include/backend/camera/SdkLifecycleRunner.hpp
    include/backend/stage/StageTypes.hpp
    include/backend/stage/IStageController.hpp
    include/backend/stage/StageWorker.hpp
    include/backend/light/LighthouseTypes.hpp
    include/backend/light/ILighthouseController.hpp
    include/backend/light/LighthouseWorker.hpp
    include/backend/camera/CaptureWriterTypes.hpp
    include/backend/camera/LumoDatasetWriter.hpp
    include/backend/camera/CaptureWriterWorker.hpp
    include/backend/HyperFusionConfig.hpp
    include/backend/camera/DualCameraScanOrchestrator.hpp
    include/backend/camera/CameraCoordinator.hpp
    include/backend/camera/processing/Gsam2ServerManager.hpp
    include/backend/camera/processing/Gsam2SegmentationClient.hpp
    include/backend/camera/processing/GsamPlan.hpp
    include/backend/camera/processing/GsamPlanSegmentRunner.hpp
    include/backend/camera/processing/Gsam2RoiAnalysis.hpp
    include/backend/camera/processing/Gsam2SpectrumPlot.hpp
    include/backend/camera/processing/GsamWslPathUtil.hpp
    include/backend/multiview/Ur3eClient.hpp
    include/backend/multiview/Ur3eHemisphereScan.hpp
    include/backend/multiview/Ur3eHemisphereScanReachability.hpp
    include/backend/multiview/Ur3eWorkspaceBoundary.hpp
    include/backend/multiview/Ur3eMountTransform.hpp
    include/backend/multiview/Ur3eServerManager.hpp
    include/backend/multiview/Ur3eWslSetup.hpp
    include/backend/multiview/Ur3eMoveItManager.hpp
    include/backend/multiview/Ur3eRvizManager.hpp
    include/backend/multiview/Ur3eWslLogUtil.hpp
    include/backend/multiview/Ur3eWslPathUtil.hpp
    include/backend/multiview/BfsCameraTypes.hpp
    include/backend/multiview/BfsSpinnakerCamera.hpp
    include/backend/multiview/BfsCameraWorker.hpp
    include/backend/fpp/DlpTypes.hpp
    include/backend/fpp/DlpProjectorWorker.hpp
    include/backend/multiview/BfsTiffIo.hpp
    include/backend/multiview/Ur3eCameraTransforms.hpp
    include/backend/multiview/Ur3eScanPlanCache.hpp
    include/backend/camera/processing/CapturePostProcessor.hpp
    include/backend/camera/processing/HfFusionRunner.hpp
    include/backend/camera/processing/HfFusionWorker.hpp
    include/backend/camera/processing/SwirBprCorrector.hpp
    include/backend/camera/processing/SwirBprCalpackMap.hpp
    include/backend/camera/processing/SwirBprApply.hpp
    include/backend/camera/processing/SwirColumnProfileCorrector.hpp
    include/backend/camera/processing/SwirRefBprCorrector.hpp
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
    src/adapters/dlp/Dlpc3478Projector.cpp
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
    include/adapters/dlp/Dlpc3478Projector.hpp
    include/adapters/dlp/DlpcCypressI2c.h
)

target_include_directories(app PRIVATE include)

if(HF_HAVE_DLPC_API)
    target_compile_definitions(app PRIVATE HF_HAVE_DLPC_API)
    # TI cyusbserial.lib is Win32. Drive the EVM from a 32-bit helper next to app.exe.
    set(HF_DLPC_BRIDGE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/adapters/dlp/bridge")
    set(HF_DLPC_BRIDGE_BIN "${CMAKE_BINARY_DIR}/src/adapters/dlp/bridge")
    add_custom_target(hf_dlpc_bridge ALL
        COMMAND ${CMAKE_COMMAND}
                -S "${HF_DLPC_BRIDGE_SRC}"
                -B "${HF_DLPC_BRIDGE_BIN}"
                -G "${CMAKE_GENERATOR}"
                -A Win32
                "-DCMAKE_GENERATOR_INSTANCE=${CMAKE_GENERATOR_INSTANCE}"
                "-DDLPC_API_ROOT=${DLPC_API_ROOT}"
        COMMAND ${CMAKE_COMMAND} --build "${HF_DLPC_BRIDGE_BIN}" --config $<CONFIG>
        VERBATIM)
    add_dependencies(app hf_dlpc_bridge)
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${HF_DLPC_BRIDGE_BIN}/$<CONFIG>/hf_dlpc_bridge.exe"
                "$<TARGET_FILE_DIR:app>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${DLPC_API_ROOT}/third_party/cyusbserial/cyusbserial.dll"
                "$<TARGET_FILE_DIR:app>"
        COMMENT "Copying 32-bit DLP helper and cyusbserial.dll")
    message(STATUS "DLP backend: 32-bit hf_dlpc_bridge (TI Cypress USB-Serial is x86)")
endif()

if(HF_HAVE_SPINNAKER)
    target_compile_definitions(app PRIVATE HF_HAVE_SPINNAKER)
    target_include_directories(app PRIVATE "${SPINNAKER_ROOT}/include")
    find_library(SPINNAKER_LIB Spinnaker_v140
        PATHS
            "${SPINNAKER_ROOT}/lib64/vs2015"
            "${SPINNAKER_ROOT}/lib64"
        NO_DEFAULT_PATH)
    if(SPINNAKER_LIB)
        target_link_libraries(app PRIVATE "${SPINNAKER_LIB}")
        message(STATUS "Linked Spinnaker library: ${SPINNAKER_LIB}")
    else()
        message(FATAL_ERROR "Spinnaker_v140.lib not found under ${SPINNAKER_ROOT}/lib64/vs2015")
    endif()

    set(_spinnaker_runtime_bin "${SPINNAKER_ROOT}/bin64/vs2015")
    if(EXISTS "${_spinnaker_runtime_bin}")
        file(GLOB _spinnaker_runtime_dlls "${_spinnaker_runtime_bin}/*.dll")
        foreach(_spin_dll IN LISTS _spinnaker_runtime_dlls)
            add_custom_command(
                TARGET app POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${_spin_dll}"
                        "$<TARGET_FILE_DIR:app>"
                COMMENT "Copying Spinnaker runtime DLL to output directory")
        endforeach()
    endif()
endif()

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

if(WIN32)
    target_link_libraries(app PRIVATE shell32)
endif()

if(WIN32)
    target_sources(app PRIVATE resources/app.rc)
    set_source_files_properties(resources/app.rc PROPERTIES
        VS_RESOURCE_COMPILE_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources")
endif()

# Ship app/preset (hyperfusion.cfg, gsam_plans, mvs_semi_scan_plans, …) beside app.exe.
set(HF_PRESET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/preset")
if(EXISTS "${HF_PRESET_DIR}")
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${HF_PRESET_DIR}"
                "$<TARGET_FILE_DIR:app>"
        COMMENT "Copying app/preset to output directory")
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

set(HF_PROCESSING_REFERENCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/backend/camera/processing/reference")
if(EXISTS "${HF_PROCESSING_REFERENCE_DIR}")
    add_custom_command(
        TARGET app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:app>/backend/camera/processing/reference"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${HF_PROCESSING_REFERENCE_DIR}/D_illuminants.json"
                "$<TARGET_FILE_DIR:app>/backend/camera/processing/reference/D_illuminants.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${HF_PROCESSING_REFERENCE_DIR}/D_illuminants.mat"
                "$<TARGET_FILE_DIR:app>/backend/camera/processing/reference/D_illuminants.mat"
        COMMENT "Copying preprocessing illuminant reference tables")
endif()

file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" HF_APP_SOURCE_DIR)
target_compile_definitions(app PRIVATE "HF_APP_SOURCE_DIR=\"${HF_APP_SOURCE_DIR}\"")
