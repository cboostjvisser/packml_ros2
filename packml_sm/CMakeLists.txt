cmake_minimum_required(VERSION 3.8)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(
  ${pkg_extracted_name}
  VERSION ${pkg_extracted_version}
  LANGUAGES CXX)

# set(CMAKE_XX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++")

# set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# prevents weird Qt error
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)


# #include all directories
# include_directories(
#   include
# )

qt5_wrap_cpp(packml_sm_MOCS
  include/packml_sm/state_machine.hpp

  include/packml_sm/states/state.hpp
  # include/packml_sm/states/toplevel_states.hpp
  # include/packml_sm/states/wait_state.hpp
  # include/packml_sm/states/acting_state.hpp

  # include/packml_sm/transitions/cmd_transition.hpp
  # include/packml_sm/transitions/sc_transition.hpp
  # include/packml_sm/transitions/error_transition.hpp

  # include/packml_sm/events/cmd_event.hpp
  # include/packml_sm/events/sc_event.hpp
  # include/packml_sm/events/error_event.hpp

  # include/packml_sm/common.hpp
  )

#add libraries
add_library(${PROJECT_NAME} SHARED
  # src/states/toplevel_states.hpp
  src/states/wait_state.cpp
  src/states/acting_state.cpp
  src/states/state.cpp

  src/transitions/cmd_transition.cpp
  src/transitions/sc_transition.cpp
  src/transitions/error_transition.cpp

  src/state_machine.cpp

  ${packml_sm_MOCS})

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
          # ${rqt_gui_cpp_TARGETS}
          Qt5::Core
          Qt5::Gui)


ament_target_dependencies(${PROJECT_NAME} PUBLIC rqt_gui_cpp)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 23)

#install
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)

  target_include_directories(
      ${PROJECT_NAME}_utest
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  # ament_target_dependencies(${PROJECT_NAME}_utest rclcpp rqt_gui_cpp Qt5)

  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} rclcpp::rclcpp Qt5::Core Qt5::Gui)
endif()

#Substituting the catkin_package () components:
#INCLUDE_DIRS
# ament_export_include_directories(include)
#LIBRARIES
# ament_export_libraries(${PROJECT_NAME})
#CATKIN_DEPENDS
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rqt_gui_cpp)
ament_package()
