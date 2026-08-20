#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "glfw" for configuration "None"
set_property(TARGET glfw APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(glfw PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libglfw.so.3.3"
  IMPORTED_SONAME_NONE "libglfw.so.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS glfw )
list(APPEND _IMPORT_CHECK_FILES_FOR_glfw "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libglfw.so.3.3" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
