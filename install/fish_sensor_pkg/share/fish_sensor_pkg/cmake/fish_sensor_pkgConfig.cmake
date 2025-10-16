# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fish_sensor_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fish_sensor_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fish_sensor_pkg_FOUND FALSE)
  elseif(NOT fish_sensor_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fish_sensor_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_fish_sensor_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fish_sensor_pkg_FIND_QUIETLY)
  message(STATUS "Found fish_sensor_pkg: 0.1.0 (${fish_sensor_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fish_sensor_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fish_sensor_pkg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fish_sensor_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fish_sensor_pkg_DIR}/${_extra}")
endforeach()
