cmake_minimum_required(VERSION 3.20)

project(old_project_phase0_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OLD_PROJECT_ROOT "E:/Git_Repository/STL-free Print/project" CACHE PATH
    "Path to the old STL-free Print project directory")
set(CURVED_SLICER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." CACHE PATH
    "Path to the new curved-slicer repository root")
set(OLD_PROJECT_VCPKG_PREFIX
    "${OLD_PROJECT_ROOT}/out/build/vs2022-x64-debug-vcpkg/vcpkg_installed/x64-windows"
    CACHE PATH
    "Existing old-project vcpkg package prefix")
set(OLD_PROJECT_IMGUI_DIR "D:/Code/STL-free Print/lib/general/imgui" CACHE PATH
    "Path containing imgui.h for old AppLog/ToolUtils headers")
set(OLD_PROJECT_LEGACY_THIRD_PARTY_DIR "D:/Code/STL-free Print/lib/general" CACHE PATH
    "Legacy third-party include root used by old project headers")
set(GLOBAL_VCPKG_PREFIX "D:/vcpkg/installed/x64-windows" CACHE PATH
    "Fallback vcpkg package prefix for split Boost header packages")

if(EXISTS "${OLD_PROJECT_VCPKG_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${OLD_PROJECT_VCPKG_PREFIX}")
    list(PREPEND CMAKE_MODULE_PATH "${OLD_PROJECT_VCPKG_PREFIX}/share/openvdb")
endif()
if(EXISTS "${GLOBAL_VCPKG_PREFIX}")
    list(APPEND CMAKE_PREFIX_PATH "${GLOBAL_VCPKG_PREFIX}")
endif()

set(_CSLC_EXISTING_DEPS_DIR "${CURVED_SLICER_ROOT}/build_nmake/_deps")
if(EXISTS "${_CSLC_EXISTING_DEPS_DIR}/tomlplusplus-src")
    set(FETCHCONTENT_SOURCE_DIR_TOMLPLUSPLUS "${_CSLC_EXISTING_DEPS_DIR}/tomlplusplus-src" CACHE PATH "" FORCE)
endif()
if(EXISTS "${_CSLC_EXISTING_DEPS_DIR}/eigen-src")
    set(FETCHCONTENT_SOURCE_DIR_EIGEN "${_CSLC_EXISTING_DEPS_DIR}/eigen-src" CACHE PATH "" FORCE)
endif()

if(NOT TARGET curved_slicer_phase0)
    add_subdirectory("${CURVED_SLICER_ROOT}" "${CMAKE_BINARY_DIR}/curved_slicer")
endif()

add_executable(old_project_phase0_benchmark
    old_project_phase0_benchmark.cpp
    "${OLD_PROJECT_ROOT}/basicDataType/STLReader.cpp"
    "${OLD_PROJECT_ROOT}/ToolUtils.cpp"
)

add_executable(old_project_phase1_baseline
    old_project_phase1_baseline.cpp
    "${OLD_PROJECT_ROOT}/basicDataType/STLReader.cpp"
    "${OLD_PROJECT_ROOT}/ToolUtils.cpp"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT/SFF.cpp"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT/MarchingCubes.cpp"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT/MarchingTriangles.cpp"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT/CurvatureField.cpp"
)

function(configure_old_project_target target_name)
target_include_directories(${target_name} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/eigen_compat"
    "${OLD_PROJECT_ROOT}"
    "${OLD_PROJECT_ROOT}/basicDataType"
    "${OLD_PROJECT_ROOT}/GLutils"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT"
)
if(EXISTS "${OLD_PROJECT_VCPKG_PREFIX}/include")
    target_include_directories(${target_name} PRIVATE "${OLD_PROJECT_VCPKG_PREFIX}/include")
endif()
if(EXISTS "${GLOBAL_VCPKG_PREFIX}/include")
    target_include_directories(${target_name} PRIVATE "${GLOBAL_VCPKG_PREFIX}/include")
endif()
if(EXISTS "${OLD_PROJECT_IMGUI_DIR}/imgui.h")
    target_include_directories(${target_name} PRIVATE "${OLD_PROJECT_IMGUI_DIR}")
endif()
if(EXISTS "${OLD_PROJECT_LEGACY_THIRD_PARTY_DIR}/eigen/Eigen/Geometry")
    target_include_directories(${target_name} PRIVATE "${OLD_PROJECT_LEGACY_THIRD_PARTY_DIR}")
    target_include_directories(${target_name} PRIVATE "${OLD_PROJECT_LEGACY_THIRD_PARTY_DIR}/eigen/Eigen")
endif()

target_compile_definitions(${target_name} PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
    _HAS_STD_BYTE=0
    NOMINMAX
    _USE_MATH_DEFINES
    UNICODE
    WIN64
    _WINDOWS
    GLM_ENABLE_EXPERIMENTAL
)

if(MSVC)
    target_compile_options(${target_name} PRIVATE /utf-8 /Zc:char8_t- /bigobj /W3)
endif()

find_package(Boost REQUIRED CONFIG COMPONENTS headers)
find_package(glm CONFIG REQUIRED)
find_package(OpenVDB REQUIRED)
find_library(STLFP_GMP_LIBRARY NAMES gmp)
find_library(STLFP_GMPXX_LIBRARY NAMES gmpxx)

if(TARGET Boost::headers)
    target_link_libraries(${target_name} PRIVATE Boost::headers)
endif()

if(TARGET glm::glm-header-only)
    target_link_libraries(${target_name} PRIVATE glm::glm-header-only)
elseif(TARGET glm::glm)
    target_link_libraries(${target_name} PRIVATE glm::glm)
endif()

target_link_libraries(${target_name} PRIVATE OpenVDB::openvdb)

if(STLFP_GMP_LIBRARY)
    target_link_libraries(${target_name} PRIVATE ${STLFP_GMP_LIBRARY})
endif()

if(STLFP_GMPXX_LIBRARY)
    target_link_libraries(${target_name} PRIVATE ${STLFP_GMPXX_LIBRARY})
endif()
endfunction()

configure_old_project_target(old_project_phase0_benchmark)
configure_old_project_target(old_project_phase1_baseline)

target_link_libraries(old_project_phase1_baseline PRIVATE curved_slicer_phase0)
