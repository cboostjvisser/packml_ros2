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
find_package(Qt5 COMPONENTS Core REQUIRED)

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
          transition_guard
          # std_msgs
          Qt5::Core)

add_executable(example_packml_node examples/example_packml_node.cpp)

target_include_directories(
    example_packml_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  example_packml_node
  PRIVATE rclcpp::rclcpp
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          transition_guard
          Qt5::Core)

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
          transition_guard
          # std_msgs
          Qt5::Core)

# TransitionGuard: ROS-independent client protocol logic.
# Separate shared library so pybind11 can link it without pulling in ROS/Qt.
add_library(transition_guard SHARED src/transition_guard.cpp)

target_include_directories(
    transition_guard
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(transition_guard PUBLIC packml_sm::packml_sm)

#install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node example_packml_node transition_guard
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

  # Main test executable with all packml_ros tests
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test_main.cpp
    test/test_manager_state_transitions.cpp
    test/test_manager_mode_changes.cpp
    test/test_manager_status_publication.cpp
    test/test_manager_client_fanout.cpp
    test/test_legacy_smnode.cpp
    test/test_node_interface.cpp
    test/test_command_mapping.cpp
    test/test_ros_logger.cpp
    test/test_modes_config.cpp
    test/test_transition_guard.cpp
    # test/test_disabled_future_features.cpp
  )

  target_include_directories(
    ${PROJECT_NAME}_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
                              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} transition_guard Qt5::Core)
endif()

#Substituting the catkin_package () components:
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(packml_sm)
ament_package(CONFIG_EXTRAS cmake/packml_ros-extras.cmake)
