// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Single shared bootstrap for the split packml_sm test suite.
//
// PackML's state machine builds on Qt's QStateMachine, which requires a
// running QCoreApplication on the same thread that processes events.  We spin
// the event loop on a worker thread so gtest can run on the main thread.
// ---

#include <QCoreApplication>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "packml_sm/logging.hpp"
#include "packml_sm/state_machine.hpp"

static void qt_worker(int argc, char * argv[])
{
  QCoreApplication app(argc, argv);
  app.exec();
}

int main(int argc, char ** argv)
{
  // Make logging visible during tests.
  ::packml_sm::Logging::instance().set_level(::packml_sm::LogLevel::DEBUG);

  std::thread qt(qt_worker, argc, argv);
  while (QCoreApplication::instance() == nullptr) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  qt.detach();

  testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();

  if (auto * app = QCoreApplication::instance()) {
    app->quit();
  }
  return rc;
}
