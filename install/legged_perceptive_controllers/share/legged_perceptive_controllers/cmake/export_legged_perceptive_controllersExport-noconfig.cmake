#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "legged_perceptive_controllers::legged_perceptive_controllers" for configuration ""
set_property(TARGET legged_perceptive_controllers::legged_perceptive_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(legged_perceptive_controllers::legged_perceptive_controllers PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblegged_perceptive_controllers.so"
  IMPORTED_SONAME_NOCONFIG "liblegged_perceptive_controllers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS legged_perceptive_controllers::legged_perceptive_controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_legged_perceptive_controllers::legged_perceptive_controllers "${_IMPORT_PREFIX}/lib/liblegged_perceptive_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
