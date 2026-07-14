include(CMakeFindDependencyMacro)
find_dependency(Qt5 COMPONENTS Core)

# Make the packml_ros_generate_error_codes() function available to downstream packages.
include("${CMAKE_CURRENT_LIST_DIR}/packml_ros_generate_error_codes.cmake")
