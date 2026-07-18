#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "legged_perceptive_interface::legged_perceptive_interface" for configuration "Debug"
set_property(TARGET legged_perceptive_interface::legged_perceptive_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(legged_perceptive_interface::legged_perceptive_interface PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/liblegged_perceptive_interface.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS legged_perceptive_interface::legged_perceptive_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_legged_perceptive_interface::legged_perceptive_interface "${_IMPORT_PREFIX}/lib/liblegged_perceptive_interface.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
