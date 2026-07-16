cmake_minimum_required(VERSION 4.2)
project(bow_connect_parallel)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)

set(CMAKE_CXX_STANDARD 20)
include_directories(
        include
        include/limbo
        include/planner
        include/common
        include/motion
        ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_FILES
        include/common/CollisionChecker.hh
        src/planner/BOWConnect.cpp
        src/planner/MotionTree.cpp
        src/collision/OccupancyMap.cpp
        src/collision/lodepng.cpp)

add_executable(bow_connect_parallel main.cpp ${SRC_FILES})

target_link_libraries(bow_connect_parallel
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    -lnlopt_cxx
)

option(BUILD_PYTHON_BINDINGS "Build the bow_connect Python module" ON)
if(BUILD_PYTHON_BINDINGS)
    # Prefer the pybind11 that ships with the Python interpreter (pip install pybind11);
    # the distro package can be too old for this CMake version.
    find_package(Python3 COMPONENTS Interpreter Development QUIET)
    if(Python3_Interpreter_FOUND)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
            OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(PYBIND11_CMAKE_DIR)
            list(PREPEND CMAKE_PREFIX_PATH ${PYBIND11_CMAKE_DIR})
        endif()
    endif()
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
        pybind11_add_module(bow_connect python/bindings.cpp ${SRC_FILES})
        target_link_libraries(bow_connect PRIVATE
            yaml-cpp::yaml-cpp
            Eigen3::Eigen
            -lnlopt_cxx
        )
    else()
        message(STATUS "pybind11 not found - skipping Python bindings")
    endif()
endif()
