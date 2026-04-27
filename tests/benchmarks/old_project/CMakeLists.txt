cmake_minimum_required(VERSION 3.20)

project(old_project_phase0_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OLD_PROJECT_ROOT "E:/Git_Repository/STL-free Print/project" CACHE PATH
    "Path to the old STL-free Print project directory")

add_executable(old_project_phase0_benchmark
    old_project_phase0_benchmark.cpp
    "${OLD_PROJECT_ROOT}/basicDataType/STLReader.cpp"
    "${OLD_PROJECT_ROOT}/ToolUtils.cpp"
)

target_include_directories(old_project_phase0_benchmark PRIVATE
    "${OLD_PROJECT_ROOT}"
    "${OLD_PROJECT_ROOT}/basicDataType"
    "${OLD_PROJECT_ROOT}/GLutils"
    "${OLD_PROJECT_ROOT}/SUPPORT_FREE_PRINT"
)

target_compile_definitions(old_project_phase0_benchmark PRIVATE
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
    target_compile_options(old_project_phase0_benchmark PRIVATE /utf-8 /Zc:char8_t- /bigobj /W3)
endif()

find_package(Boost REQUIRED CONFIG COMPONENTS headers)
find_package(glm CONFIG REQUIRED)
find_package(OpenVDB CONFIG REQUIRED)

if(TARGET Boost::headers)
    target_link_libraries(old_project_phase0_benchmark PRIVATE Boost::headers)
endif()

if(TARGET glm::glm-header-only)
    target_link_libraries(old_project_phase0_benchmark PRIVATE glm::glm-header-only)
elseif(TARGET glm::glm)
    target_link_libraries(old_project_phase0_benchmark PRIVATE glm::glm)
endif()

target_link_libraries(old_project_phase0_benchmark PRIVATE OpenVDB::openvdb)

