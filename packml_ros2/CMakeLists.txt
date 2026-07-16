cmake_minimum_required(VERSION 3.8)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(
  ${pkg_extracted_name}
  VERSION ${pkg_extracted_version}
  LANGUAGES CXX)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

find_package(ament_cmake REQUIRED)
find_package(packml_ros REQUIRED)

# Aggregate the demo's error_map.yaml into machine_error_catalog.yaml at build
# time (see packml_ros_aggregate_error_catalog.cmake for the ALL-target/DEPFILE
# rationale — this package has no compile targets of its own to hang the
# dependency off of otherwise).
packml_ros_aggregate_error_catalog(
  ${PROJECT_NAME}_error_catalog
  config/error_map.yaml
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/error_catalog
)

ament_package()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/error_catalog/
  DESTINATION share/${PROJECT_NAME}/config)
