cmake_minimum_required(VERSION 3.14)
project(packml_ros_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(packml_ros REQUIRED)
find_package(packml_sm REQUIRED)

# Install the Python package (pure Python layer)
ament_python_install_package(${PROJECT_NAME})

# Build pybind11 extension module
pybind11_add_module(_packml_bindings src/bindings.cpp)

target_link_libraries(_packml_bindings PRIVATE
  packml_ros::transition_guard
  packml_sm::packml_sm
)

# Install the compiled extension into the Python package directory
install(TARGETS _packml_bindings
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Generate .pyi type stubs from the compiled pybind11 module at install time.
install(CODE "
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11_stubgen
      packml_ros_py._packml_bindings
      --output-dir \"\${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}\"
    RESULT_VARIABLE _stubgen_result
  )
  if(NOT _stubgen_result EQUAL 0)
    message(WARNING \"pybind11-stubgen failed (exit \${_stubgen_result}). Type stubs not generated.\")
  endif()
")

# Install Python executable (has shebang, ros2 run finds it in lib/<pkg>)
install(PROGRAMS examples/example_node.py
  RENAME example_packml_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(packml_ros_py_tests test/)
endif()

ament_package()
