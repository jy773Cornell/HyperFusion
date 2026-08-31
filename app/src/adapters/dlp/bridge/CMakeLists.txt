# Win32 DLPC helper (adapters/dlp). Nested -A Win32 build — do not add_subdirectory from the x64 app.
# TI ships x86 cyusbserial only.
cmake_minimum_required(VERSION 3.16)
project(hf_dlpc_bridge LANGUAGES C)

if(NOT DLPC_API_ROOT)
    message(FATAL_ERROR "DLPC_API_ROOT is required")
endif()

set(HF_APP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(HF_DLP_ADAPTER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_executable(hf_dlpc_bridge
    hf_dlpc_bridge.c
    "${HF_DLP_ADAPTER_DIR}/DlpcCypressI2c.c"
    "${DLPC_API_ROOT}/api/dlpc_common.c"
    "${DLPC_API_ROOT}/api/dlpc34xx.c")

target_include_directories(hf_dlpc_bridge PRIVATE
    "${HF_APP_DIR}/include"
    "${DLPC_API_ROOT}/api"
    "${DLPC_API_ROOT}/third_party")

find_library(CYUSBSERIAL_LIB cyusbserial
    PATHS "${DLPC_API_ROOT}/third_party/cyusbserial"
    NO_DEFAULT_PATH)
if(NOT CYUSBSERIAL_LIB)
    message(FATAL_ERROR "cyusbserial.lib not found under ${DLPC_API_ROOT}/third_party/cyusbserial")
endif()
target_link_libraries(hf_dlpc_bridge PRIVATE "${CYUSBSERIAL_LIB}")

if(MSVC)
    target_compile_definitions(hf_dlpc_bridge PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

if(EXISTS "${DLPC_API_ROOT}/third_party/cyusbserial/cyusbserial.dll")
    add_custom_command(
        TARGET hf_dlpc_bridge POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${DLPC_API_ROOT}/third_party/cyusbserial/cyusbserial.dll"
                "$<TARGET_FILE_DIR:hf_dlpc_bridge>"
        COMMENT "Copying x86 cyusbserial.dll next to hf_dlpc_bridge.exe")
endif()
