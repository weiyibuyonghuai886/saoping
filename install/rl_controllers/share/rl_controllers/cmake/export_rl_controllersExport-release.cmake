#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rl_controllers::rl_controllers" for configuration "Release"
set_property(TARGET rl_controllers::rl_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rl_controllers::rl_controllers PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "yaml-cpp"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librl_controllers.so"
  IMPORTED_SONAME_RELEASE "librl_controllers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rl_controllers::rl_controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_rl_controllers::rl_controllers "${_IMPORT_PREFIX}/lib/librl_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
