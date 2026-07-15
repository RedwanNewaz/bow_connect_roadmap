# FindOctree.cmake
#
# Finds the OctreeCollisionChecker library.
#
# This will define the following variables:
#
# OCTREE_FOUND          - True if OctreeCollisionChecker was found.
# OCTREE_INCLUDE_DIRS   - Include directories for OctreeCollisionChecker.
# OCTREE_LIBRARIES      - Libraries for OctreeCollisionChecker.
#
# And the following imported target:
#
# Octree::Octree

find_path(OCTREE_INCLUDE_DIR
    NAMES OctreeCollisionChecker.h
    PATH_SUFFIXES Octree
)

find_library(OCTREE_LIBRARY
    NAMES OctreeCollisionChecker
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Octree
    REQUIRED_VARS OCTREE_LIBRARY OCTREE_INCLUDE_DIR
)

if(OCTREE_FOUND)
    set(OCTREE_INCLUDE_DIRS ${OCTREE_INCLUDE_DIR})
    set(OCTREE_LIBRARIES ${OCTREE_LIBRARY})
    
    if(NOT TARGET Octree::Octree)
        add_library(Octree::Octree SHARED IMPORTED)
        set_target_properties(Octree::Octree PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${OCTREE_INCLUDE_DIR}"
            IMPORTED_LOCATION "${OCTREE_LIBRARY}"
        )
    endif()
endif()

mark_as_advanced(OCTREE_INCLUDE_DIR OCTREE_LIBRARY)
