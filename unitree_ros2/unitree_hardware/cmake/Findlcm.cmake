find_path(lcm_INCLUDE_DIR
  NAMES lcm/lcm.h
  PATH_SUFFIXES include
)

find_library(lcm_LIBRARY
  NAMES lcm
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(lcm DEFAULT_MSG lcm_INCLUDE_DIR lcm_LIBRARY)

if(lcm_FOUND)
  set(lcm_INCLUDE_DIRS ${lcm_INCLUDE_DIR})
  set(lcm_LIBRARIES    ${lcm_LIBRARY})
endif()
