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
#include <thread>

int main(int argc, char ** argv)
{
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
