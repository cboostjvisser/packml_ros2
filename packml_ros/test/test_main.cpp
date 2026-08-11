// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <QtCore/QCoreApplication>
#include <cstdlib>
#include <thread>

// Every test binary linking this main gets its OWN ROS domain, declared by its CMake target.
// Not optional, and not defaulted: two binaries sharing a domain share the manager's status,
// alarm and heartbeat topics, which are deliberately bare global names (ros_names.hpp) that any
// participant may write. Share a domain and the suites poison each other: packml_ros_shakedown's
// probes republish EXECUTE at ~5 Hz and spray spoofed states on packml_status by design, which
// makes NodeInterfaceTest's node in packml_ros_tests believe it is in EXECUTE between that test's
// own status publication and its request. A new binary must pick an unused number here rather
// than inherit one silently.
#ifndef PACKML_TEST_ROS_DOMAIN_ID
#error "define PACKML_TEST_ROS_DOMAIN_ID per test target (see packml_ros/CMakeLists.txt)"
#endif

#define PACKML_STRINGIFY_(x) #x
#define PACKML_STRINGIFY(x) PACKML_STRINGIFY_(x)

int main(int argc, char ** argv)
{
  // Before rclcpp::init(), which is where the domain and the discovery range are read. Set
  // unconditionally rather than only when unset: an inherited ROS_DOMAIN_ID from the
  // developer's shell is the case this exists to defend against, and tests that reach a real
  // robot or a stray leftover process are worse than tests that ignore an override. LOCALHOST
  // keeps the traffic off the subnet as well, which the domain number alone does not do.
  setenv("ROS_DOMAIN_ID", PACKML_STRINGIFY(PACKML_TEST_ROS_DOMAIN_ID), 1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);

  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Start Qt event loop in background (needed by packml_sm state machine).
  // QCoreApplication is heap-allocated and intentionally never deleted to
  // avoid a crash in its destructor during atexit (it would be destroyed
  // from the wrong thread).
  std::thread([]() {
      static int qt_argc = 0;
      auto * app = new QCoreApplication(qt_argc, nullptr);
      app->exec();
    }).detach();

  // Give Qt time to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int result = RUN_ALL_TESTS();

  // Shutdown ROS2 and Qt after tests. Test fixtures must implement
  // TearDownTestSuite() to release their ROS2 nodes and SM objects
  // BEFORE this point (GTest calls it before RUN_ALL_TESTS returns).
  rclcpp::shutdown();
  QCoreApplication::quit();
  return result;
}
