cmake_minimum_required(VERSION 3.25)

project(CamStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCASCADE REQUIRED COMPONENTS
    FoundationClasses
    ModelingData
    ModelingAlgorithms
    Visualization
    ApplicationFramework
    DataExchange
)

add_library(camstudio_core
    src/core/motion.cpp
    src/core/trajectory.cpp
)

target_include_directories(camstudio_core PUBLIC src)
target_compile_options(camstudio_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(camstudio_cad
    src/cad/cad_export.cpp
)
target_include_directories(camstudio_cad PUBLIC src ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(camstudio_cad PUBLIC
    camstudio_core
    TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep
    TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKShHealing TKOffset TKMesh
    TKDE TKXSBase TKDESTEP TKDESTL
)
target_compile_options(camstudio_cad PRIVATE -Wall -Wextra -Wpedantic)

add_executable(camstudio src/cli/main.cpp)
target_link_libraries(camstudio PRIVATE camstudio_cad)

enable_testing()
add_executable(camstudio_tests tests/trajectory_tests.cpp)
target_link_libraries(camstudio_tests PRIVATE camstudio_core)
target_compile_definitions(camstudio_tests PRIVATE
    CAMSTUDIO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
)
add_test(NAME trajectory_tests COMMAND camstudio_tests)

add_executable(camstudio_cad_tests tests/cad_export_tests.cpp)
target_link_libraries(camstudio_cad_tests PRIVATE camstudio_cad)
add_test(NAME cad_export_tests COMMAND camstudio_cad_tests)
set_tests_properties(cad_export_tests PROPERTIES TIMEOUT 120)
