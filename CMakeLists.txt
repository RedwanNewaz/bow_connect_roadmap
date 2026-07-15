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
