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

ament_package()
