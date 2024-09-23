cmake_minimum_required(VERSION 3.8)
project(packml_ros)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(
  ${pkg_extracted_name}
  VERSION ${pkg_extracted_version}
  LANGUAGES CXX)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++23
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 23)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(packml_msgs REQUIRED)
find_package(packml_sm REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

add_executable(${PROJECT_NAME}_node src/packml_ros_node.cpp)

target_include_directories(
    ${PROJECT_NAME}_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  ${PROJECT_NAME}_node
  PRIVATE rclcpp::rclcpp
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          # std_msgs
          Qt5::Core
          Qt5::Gui)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE rclcpp::rclcpp
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          # std_msgs
          Qt5::Core)

#install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # We need to install to lib/${PROJECT_NAME}.
  # If we follow https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html#installing RUNTIME Destination is bin
  # Maybe something to do with: https://github.com/ros-controls/ros2_control/issues/172#issuecomment-705957030
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(${PROJECT_NAME}_utest test/utest.cpp)

  target_include_directories(
    ${PROJECT_NAME}_utest PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

  # ament_target_dependencies(${PROJECT_NAME}_utest rclcpp packml_msgs packml_sm std_msgs Qt5)
endif()

#Substituting the catkin_package () components:
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(packml_sm)
ament_package()
