# FindCollisionChecker.cmake
#
# Provides:
#   CollisionChecker_FOUND
#   CollisionChecker_INCLUDE_DIRS
#   CollisionChecker_LIBRARIES
#
# Imported targets:
#   CollisionChecker::OccupancyMap
#   CollisionChecker::Octree
#   CollisionChecker::Quadtree
#   CollisionChecker::Vamp

if(NOT CollisionChecker_ROOT)
    set(CollisionChecker_ROOT "${CMAKE_SOURCE_DIR}/lib/collision" CACHE PATH "Root directory of CollisionChecker")
endif()
include(FindPackageHandleStandardArgs)

# ---------------------------------------------------------
# Find include directory
# ---------------------------------------------------------
find_path(CollisionChecker_INCLUDE_DIR
        NAMES CollisionChecker.hh
        PATH_SUFFIXES
        include
        include/collision
        HINTS ${CollisionChecker_ROOT}
        NO_DEFAULT_PATH # Ensures it looks in your specific lib folder first
)

# Find additional include directories (common, FCL, etc.)
find_path(CollisionChecker_COMMON_INCLUDE_DIR
        NAMES ParamManager.h
        PATH_SUFFIXES
        include/common
        HINTS ${CollisionChecker_ROOT}
        NO_DEFAULT_PATH
)

# ---------------------------------------------------------
# Find libraries
# ---------------------------------------------------------
#We use HINTS and PATH_SUFFIXES instead of link_directories
set(_CC_LIB_SUFFIXES lib bin)
find_library(CollisionChecker_OCCUPANCY_LIB
        NAMES OccupancyMap
        HINTS ${CollisionChecker_ROOT}
        PATH_SUFFIXES ${_CC_LIB_SUFFIXES}
)

find_library(CollisionChecker_OCTREE_LIB
        NAMES OctreeCollisionChecker
        HINTS ${CollisionChecker_ROOT}
        PATH_SUFFIXES ${_CC_LIB_SUFFIXES}
)

find_library(CollisionChecker_QUADTREE_LIB
        NAMES QuadtreeCollisionChecker
        HINTS ${CollisionChecker_ROOT}
        PATH_SUFFIXES ${_CC_LIB_SUFFIXES}
)

find_library(CollisionChecker_VAMP_LIB
        NAMES VampCollisionChecker
        HINTS ${CollisionChecker_ROOT}
        PATH_SUFFIXES ${_CC_LIB_SUFFIXES}
)


find_library(CollisionChecker_FCL_LIB
        NAMES FCL
        HINTS ${CollisionChecker_ROOT}
        PATH_SUFFIXES ${_CC_LIB_SUFFIXES}
)

# ---------------------------------------------------------
# Standard package handling
# ---------------------------------------------------------
find_package_handle_standard_args(
        CollisionChecker
        REQUIRED_VARS
        CollisionChecker_INCLUDE_DIR
        CollisionChecker_OCCUPANCY_LIB
        CollisionChecker_OCTREE_LIB
        CollisionChecker_QUADTREE_LIB
        CollisionChecker_VAMP_LIB
        CollisionChecker_FCL_LIB
)

# ---------------------------------------------------------
# Export variables
# ---------------------------------------------------------
if (CollisionChecker_FOUND)
    set(CollisionChecker_INCLUDE_DIRS
            ${CollisionChecker_INCLUDE_DIR}
            ${CollisionChecker_ROOT}/include
            ${CollisionChecker_ROOT}/include/common
            ${CollisionChecker_ROOT}/include/collision
    )

    set(CollisionChecker_LIBRARIES
            ${CollisionChecker_OCCUPANCY_LIB}
            ${CollisionChecker_OCTREE_LIB}
            ${CollisionChecker_QUADTREE_LIB}
            ${CollisionChecker_VAMP_LIB}
            ${CollisionChecker_FCL_LIB}
    )

    # -----------------------------------------------------
    # Imported targets (modern CMake)
    # -----------------------------------------------------
    if (NOT TARGET CollisionChecker::OccupancyMap)
        add_library(CollisionChecker::OccupancyMap SHARED IMPORTED)
        set_target_properties(CollisionChecker::OccupancyMap PROPERTIES
                IMPORTED_LOCATION ${CollisionChecker_OCCUPANCY_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${CollisionChecker_INCLUDE_DIR}
        )
    endif()

    if (NOT TARGET CollisionChecker::FCL)
        add_library(CollisionChecker::FCL SHARED IMPORTED)
        set_target_properties(CollisionChecker::FCL PROPERTIES
                IMPORTED_LOCATION ${CollisionChecker_FCL_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${CollisionChecker_INCLUDE_DIR}
        )
    endif()

    if (NOT TARGET CollisionChecker::Octree)
        add_library(CollisionChecker::Octree SHARED IMPORTED)
        set_target_properties(CollisionChecker::Octree PROPERTIES
                IMPORTED_LOCATION ${CollisionChecker_OCTREE_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${CollisionChecker_INCLUDE_DIR}
        )
    endif()

    if (NOT TARGET CollisionChecker::Quadtree)
        add_library(CollisionChecker::Quadtree SHARED IMPORTED)
        set_target_properties(CollisionChecker::Quadtree PROPERTIES
                IMPORTED_LOCATION ${CollisionChecker_QUADTREE_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${CollisionChecker_INCLUDE_DIR}
        )
    endif()

    if (NOT TARGET CollisionChecker::Vamp)
        add_library(CollisionChecker::Vamp SHARED IMPORTED)
        set_target_properties(CollisionChecker::Vamp PROPERTIES
                IMPORTED_LOCATION ${CollisionChecker_VAMP_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${CollisionChecker_INCLUDE_DIR}
        )
    endif()
endif()
