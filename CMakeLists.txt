cmake_minimum_required(VERSION 3.20)

project(curved_slicer
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG 30172438cee64926dc41fdd9c11fb3ba5b2ba9de
    GIT_CONFIG http.sslBackend=openssl
    GIT_SUBMODULES ""
    GIT_PROGRESS TRUE
)

FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
    GIT_CONFIG http.sslBackend=openssl
    GIT_SUBMODULES ""
    GIT_PROGRESS TRUE
)

set(TOMLPLUSPLUS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(TOMLPLUSPLUS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(tomlplusplus)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

FetchContent_Declare(libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
    GIT_CONFIG http.sslBackend=openssl
    GIT_SUBMODULES ""
    GIT_PROGRESS TRUE
)

set(LIBIGL_EMBREE OFF CACHE BOOL "" FORCE)
set(LIBIGL_GLFW OFF CACHE BOOL "" FORCE)
set(LIBIGL_OPENGL OFF CACHE BOOL "" FORCE)
set(LIBIGL_PNG OFF CACHE BOOL "" FORCE)
set(LIBIGL_STB OFF CACHE BOOL "" FORCE)
set(LIBIGL_PREDICATES OFF CACHE BOOL "" FORCE)
set(LIBIGL_XML OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(libigl)

set(BUILD_TESTING ON CACHE BOOL "" FORCE)

add_library(curved_slicer_phase0
    src/io/stl_reader.cpp
    src/io/robot_writer.cpp
    src/io/config_loader.cpp
    src/geometry/voxel_grid.cpp
    src/geometry/bvh.cpp
    src/geometry/sdf.cpp
    src/field/kuka_projection.cpp
    src/field/laplacian.cpp
    src/field/poisson.cpp
    src/field/field_smoothing.cpp
    src/kinematics/forward_kin.cpp
    src/kinematics/ik_solver.cpp
    src/kinematics/reachability.cpp
    src/path/pose_from_path.cpp
    src/path/geodesic_paths.cpp
    src/metrics/curvature.cpp
    src/surface/iso_surface.cpp
    src/app/pipeline.cpp
)

target_include_directories(curved_slicer_phase0
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(curved_slicer_phase0
    PRIVATE
        tomlplusplus::tomlplusplus
    PUBLIC
        Eigen3::Eigen
        igl::core
)

if(WIN32)
    target_link_libraries(curved_slicer_phase0 PUBLIC Psapi)
endif()

if(MSVC)
    target_compile_options(curved_slicer_phase0 PRIVATE /utf-8 /W4 /permissive-)
    target_compile_definitions(curved_slicer_phase0 PUBLIC _USE_MATH_DEFINES UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    target_compile_options(curved_slicer_phase0 PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

function(configure_phase0_test target_name)
    target_link_libraries(${target_name} PRIVATE curved_slicer_phase0)
    target_compile_definitions(${target_name}
        PRIVATE
            _USE_MATH_DEFINES
            CSLC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /utf-8 /W4 /permissive-)
    endif()
endfunction()

add_executable(phase0_batch_test
    tests/phase0_batch_test.cpp
)

configure_phase0_test(phase0_batch_test)

add_test(NAME phase0_batch_test COMMAND phase0_batch_test)

add_executable(config_loader_unit_test
    tests/config_loader_unit_test.cpp
)

configure_phase0_test(config_loader_unit_test)

add_test(NAME config_loader_unit COMMAND config_loader_unit_test)

add_executable(eigen_smoke_test
    tests/eigen_smoke_test.cpp
)

target_link_libraries(eigen_smoke_test PRIVATE Eigen3::Eigen)

if(MSVC)
    target_compile_options(eigen_smoke_test PRIVATE /utf-8 /W4 /permissive-)
endif()

add_test(NAME eigen_smoke COMMAND eigen_smoke_test)

add_executable(sdf_smoke_test
    tests/sdf_smoke_test.cpp
)

configure_phase0_test(sdf_smoke_test)

add_test(NAME sdf_smoke COMMAND sdf_smoke_test)

add_executable(iso_surface_mc_test
    tests/iso_surface_mc_test.cpp
)

configure_phase0_test(iso_surface_mc_test)

add_test(NAME iso_surface_mc_test COMMAND iso_surface_mc_test)

add_executable(laplacian_smoke_test
    tests/laplacian_smoke_test.cpp
)

configure_phase0_test(laplacian_smoke_test)

add_test(NAME laplacian_smoke COMMAND laplacian_smoke_test)

add_executable(poisson_smoke_test
    tests/poisson_smoke_test.cpp
)

configure_phase0_test(poisson_smoke_test)

add_test(NAME poisson_smoke COMMAND poisson_smoke_test)

add_executable(phase2_field_batch_report
    tests/phase2_field_batch_report.cpp
)

configure_phase0_test(phase2_field_batch_report)
# Manual Phase 2 acceptance driver. It is intentionally not part of default
# ctest because the three-model 0.5mm run is long-running.

add_executable(phase0_io_voxel_benchmark
    tests/benchmarks/phase0_io_voxel_benchmark.cpp
)

configure_phase0_test(phase0_io_voxel_benchmark)

add_executable(geometric_z_bc_unit_test
    tests/geometric_z_bc_unit_test.cpp
)

configure_phase0_test(geometric_z_bc_unit_test)

add_test(NAME geometric_z_bc_unit COMMAND geometric_z_bc_unit_test)

add_executable(forward_kin_unit_test
    tests/forward_kin_unit_test.cpp
)

configure_phase0_test(forward_kin_unit_test)

target_link_libraries(forward_kin_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME forward_kin_unit COMMAND forward_kin_unit_test)

add_executable(ik_solver_unit_test
    tests/ik_solver_unit_test.cpp
)

configure_phase0_test(ik_solver_unit_test)

target_link_libraries(ik_solver_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME ik_solver_unit COMMAND ik_solver_unit_test)

add_executable(pose_from_path_unit_test
    tests/pose_from_path_unit_test.cpp
)

configure_phase0_test(pose_from_path_unit_test)

target_link_libraries(pose_from_path_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME pose_from_path_unit COMMAND pose_from_path_unit_test)

add_executable(reachability_unit_test
    tests/reachability_unit_test.cpp
)

configure_phase0_test(reachability_unit_test)

target_link_libraries(reachability_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME reachability_unit COMMAND reachability_unit_test)

add_executable(voxel_filter_unit_test
    tests/voxel_filter_unit_test.cpp
)

configure_phase0_test(voxel_filter_unit_test)

target_link_libraries(voxel_filter_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME voxel_filter_unit COMMAND voxel_filter_unit_test)

add_executable(path_filter_unit_test
    tests/path_filter_unit_test.cpp
)

configure_phase0_test(path_filter_unit_test)

target_link_libraries(path_filter_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME path_filter_unit COMMAND path_filter_unit_test)

add_executable(igl_smoke_test
    tests/igl_smoke_test.cpp
)

configure_phase0_test(igl_smoke_test)

add_test(NAME igl_smoke COMMAND igl_smoke_test)

add_executable(geodesic_paths_unit_test
    tests/geodesic_paths_unit_test.cpp
)

configure_phase0_test(geodesic_paths_unit_test)

target_link_libraries(geodesic_paths_unit_test PRIVATE Eigen3::Eigen)

add_test(NAME geodesic_paths_unit COMMAND geodesic_paths_unit_test)
