# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_flightstack_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED flightstack_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(flightstack_FOUND FALSE)
  elseif(NOT flightstack_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(flightstack_FOUND FALSE)
  endif()
  return()
endif()
set(_flightstack_CONFIG_INCLUDED TRUE)

# output package information
if(NOT flightstack_FIND_QUIETLY)
  message(STATUS "Found flightstack: 1.0.0 (${flightstack_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'flightstack' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${flightstack_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(flightstack_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${flightstack_DIR}/${_extra}")
endforeach()
