cmake_minimum_required(VERSION 3.8)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(
  ${pkg_extracted_name}
  VERSION ${pkg_extracted_version}
  LANGUAGES CXX)

# set(CMAKE_XX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++")

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
# find_package(pluginlib REQUIRED)
find_package(packml_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)

#makes boost signals work together with qt's when using cmake
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(QT_MOC
 include/packml_plugin/packml_plugin.hpp
 include/packml_plugin/packml_widget.hpp
)

# set(SOURCES )

qt5_wrap_ui(QT_UI_FILES src/packml.ui)

add_library(${PROJECT_NAME} SHARED
  ${QT_UI_FILES}
  ${QT_MOC}

  src/packml_plugin.cpp
  src/packml_widget.cpp
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp

  ${packml_msgs_TARGETS}

  rviz_common::rviz_common

  Qt5::Widgets
  Qt5::Core
  Qt5::Gui)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# ament_target_dependencies(${PROJECT_NAME}
# "rviz_common" "Qt5" "packml_msgs" "std_msgs" "rclcpp" "rosidl_typesupport_cpp")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

#install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

