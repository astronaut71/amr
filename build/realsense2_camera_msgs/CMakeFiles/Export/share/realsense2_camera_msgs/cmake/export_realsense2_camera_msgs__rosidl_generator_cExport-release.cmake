#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "realsense2_camera_msgs::realsense2_camera_msgs__rosidl_generator_c" for configuration "Release"
set_property(TARGET realsense2_camera_msgs::realsense2_camera_msgs__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(realsense2_camera_msgs::realsense2_camera_msgs__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librealsense2_camera_msgs__rosidl_generator_c.so"
  IMPORTED_SONAME_RELEASE "librealsense2_camera_msgs__rosidl_generator_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2_camera_msgs::realsense2_camera_msgs__rosidl_generator_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2_camera_msgs::realsense2_camera_msgs__rosidl_generator_c "${_IMPORT_PREFIX}/lib/librealsense2_camera_msgs__rosidl_generator_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
