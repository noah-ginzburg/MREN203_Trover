# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_trover_drivers_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED trover_drivers_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(trover_drivers_FOUND FALSE)
  elseif(NOT trover_drivers_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(trover_drivers_FOUND FALSE)
  endif()
  return()
endif()
set(_trover_drivers_CONFIG_INCLUDED TRUE)

# output package information
if(NOT trover_drivers_FIND_QUIETLY)
  message(STATUS "Found trover_drivers: 0.0.0 (${trover_drivers_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'trover_drivers' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${trover_drivers_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(trover_drivers_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${trover_drivers_DIR}/${_extra}")
endforeach()
