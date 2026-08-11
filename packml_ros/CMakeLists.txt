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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(packml_msgs REQUIRED)
find_package(packml_sm REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_executable(${PROJECT_NAME}_node src/packml_ros_node.cpp)

target_include_directories(
    ${PROJECT_NAME}_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  ${PROJECT_NAME}_node
  PRIVATE rclcpp::rclcpp
          rclcpp_action::rclcpp_action
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          transition_guard
          error_catalog
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
          rclcpp_action::rclcpp_action
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          transition_guard
          error_catalog
          Qt5::Core)

add_executable(example_health_demo examples/example_health_demo.cpp)

target_include_directories(
    example_health_demo
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  example_health_demo
  PRIVATE rclcpp::rclcpp
          rclcpp_action::rclcpp_action
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          transition_guard
          error_catalog
          Qt5::Core)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE rclcpp::rclcpp
          rclcpp_action::rclcpp_action
          packml_sm::packml_sm
          ${packml_msgs_TARGETS}
          transition_guard
          error_catalog
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

# ErrorCatalog: implementor-defined error codes with multi-language descriptions.
# Separate shared library (like transition_guard); yaml-cpp and packml_msgs are
# private implementation details (the public header error_catalog.hpp is msg-free).

# The error-catalog YAML's section/field key names (e.g. "error_codes",
# "descriptions") are generated into a private C++ header from the same Python
# module that defines them for generate_error_codes_header.py's own codegen,
# so error_catalog.cpp's parser and the Python script can't independently
# drift on the same schema.
set(_error_catalog_schema_dir "${CMAKE_CURRENT_BINARY_DIR}/error_catalog_schema/include")
set(_error_catalog_schema_header "${_error_catalog_schema_dir}/packml_ros/error_catalog_schema_keys.hpp")
add_custom_command(
  OUTPUT "${_error_catalog_schema_header}"
  COMMAND ${Python3_EXECUTABLE}
          "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_error_codes_header.py"
          --schema-keys-header "${_error_catalog_schema_header}"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_error_codes_header.py"
  COMMENT "Generating error-catalog schema key constants"
)
add_custom_target(error_catalog_schema_keys_gen DEPENDS "${_error_catalog_schema_header}")

add_library(error_catalog SHARED src/error_catalog.cpp)
add_dependencies(error_catalog error_catalog_schema_keys_gen)

target_include_directories(
    error_catalog
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    PRIVATE $<BUILD_INTERFACE:${_error_catalog_schema_dir}>)

target_link_libraries(error_catalog PRIVATE ${packml_msgs_TARGETS} yaml-cpp)

#install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node example_packml_node example_health_demo transition_guard error_catalog
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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_aggregate_error_catalog_tests test/test_aggregate_error_catalog.py)
  ament_add_pytest_test(${PROJECT_NAME}_setup_helper_tests test/test_packml_ros_setup.py)

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
    test/test_health_monitor.cpp
    test/test_health_monitor_integration.cpp
    test/test_completion_tracker.cpp
    test/test_completion_integration.cpp
    test/test_monkey_scenarios.cpp
    test/test_monkey_reporting.cpp
    test/test_monkey_health.cpp
    test/test_monkey_fanout.cpp
    test/test_monkey_multi_mode.cpp
    test/test_monkey_edge_cases.cpp
    test/test_monkey_alarms.cpp
    test/test_monkey_manager_restart.cpp
    test/test_monkey_fuzzer.cpp
    test/test_error_catalog.cpp
    # test/test_disabled_future_features.cpp
    # Combined target runs ~150-190s pre-monkey-expansion (heartbeat/health/completion/monkey
    # tests all use real-time waits, kept as short as each test can afford, but the suite has
    # grown well past the ~60s this comment originally described, and the 8 new
    # test_monkey_*.cpp files above add real-time waits of their own) -- bumped again with real
    # headroom once actual runtime is observed post-build.
    TIMEOUT 600
  )

  target_include_directories(
    ${PROJECT_NAME}_tests PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
                              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} transition_guard error_catalog Qt5::Core)

  # This binary's own ROS domain; see test_main.cpp for why every test binary needs a distinct
  # one. Applied in test_main.cpp rather than as an ament_add_test ENV so it also holds when the
  # binary is run directly, which is how the contamination it prevents was first observed.
  target_compile_definitions(${PROJECT_NAME}_tests PRIVATE PACKML_TEST_ROS_DOMAIN_ID=71)

  # Shakedown probes: adversarial attack-surface tests (rogue publishers, poisoned config,
  # reentrancy during in-flight waits, state-graph dark corners) kept in their OWN binary,
  # deliberately separate from ${PROJECT_NAME}_tests: these hunt for crashes and deadlocks,
  # and a probe that kills the process must not destabilize the polished main suite. Probes
  # are promoted into the main suite (or dropped) after triage of what they find.
  ament_add_gtest(${PROJECT_NAME}_shakedown
    test/test_main.cpp
    test/test_shakedown_wire.cpp
    test/test_shakedown_config.cpp
    test/test_shakedown_interleave.cpp
    test/test_shakedown_stategraph.cpp
    test/test_shakedown_guard.cpp
    TIMEOUT 600
  )

  target_include_directories(
    ${PROJECT_NAME}_shakedown PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
                              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_link_libraries(${PROJECT_NAME}_shakedown ${PROJECT_NAME} transition_guard error_catalog Qt5::Core)

  target_compile_definitions(${PROJECT_NAME}_shakedown PRIVATE PACKML_TEST_ROS_DOMAIN_ID=72)
endif()

# Error-catalog code generation function + script, for downstream node packages
# (mirrors packml_sm's exported packml_sm_generate_modes).
install(
  FILES
    cmake/packml_ros_generate_error_codes.cmake
    cmake/generate_error_codes_header.py
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Demo error catalog: must be installed (not just present in the source tree)
# because aggregate_error_catalog.py resolves package://packml_ros/... URIs via
# the ament package index, which only sees installed files. packml_ros2's
# error_map.yaml references this one at build time.
install(FILES examples/config/error_catalog.yaml DESTINATION share/${PROJECT_NAME}/config)

# Offline error-catalog aggregation: the exported CMake function + its script,
# for downstream bringup packages (see packml_ros_aggregate_error_catalog.cmake).
install(
  FILES
    cmake/packml_ros_aggregate_error_catalog.cmake
    scripts/aggregate_error_catalog.py
  DESTINATION share/${PROJECT_NAME}/cmake
)
# Also installed as a `ros2 run packml_ros aggregate_error_catalog` executable,
# for previewing the sheet / debugging lint failures directly.
install(
  PROGRAMS scripts/aggregate_error_catalog.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aggregate_error_catalog
)
# The setup.py-side equivalent of packml_ros_aggregate_error_catalog(), for
# ament_python bringup packages that have no CMakeLists.txt to call it from.
# Installed as an importable module (not into share/) because a setup.py can
# only reach it by `import packml_ros_setup`.
ament_python_install_module(scripts/packml_ros_setup.py)

#Substituting the catkin_package () components:
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(packml_sm)
ament_package(CONFIG_EXTRAS cmake/packml_ros-extras.cmake)
