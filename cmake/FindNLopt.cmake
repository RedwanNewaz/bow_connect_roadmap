# FindNLopt.cmake

find_path(NLopt_INCLUDE_DIR
  NAMES nlopt.h
  PATH_SUFFIXES nlopt
  HINTS
    ${NLopt_ROOT}
    $ENV{NLopt_ROOT}
    /usr/local/include
    /usr/include
)

find_library(NLopt_LIBRARY
  NAMES nlopt nlopt_cxx
  HINTS
    ${NLopt_ROOT}
    $ENV{NLopt_ROOT}
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NLopt
  REQUIRED_VARS NLopt_LIBRARY NLopt_INCLUDE_DIR
)

if(NLopt_FOUND)
  set(NLopt_INCLUDE_DIRS ${NLopt_INCLUDE_DIR})
  set(NLopt_LIBRARIES ${NLopt_LIBRARY})
  if(NOT TARGET NLopt::NLopt)
    add_library(NLopt::NLopt UNKNOWN IMPORTED)
    set_target_properties(NLopt::NLopt PROPERTIES
      IMPORTED_LOCATION "${NLopt_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${NLopt_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(NLopt_INCLUDE_DIR NLopt_LIBRARY)
