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
// Validates the framework-agnostic logging bridge.  These tests confirm
// integrators can intercept log output (e.g. to forward to RCLCPP) without
// pulling ROS into the library itself.
// ---

#include <atomic>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <vector>

#include "packml_sm/logging.hpp"

using packml_sm::LogLevel;
using packml_sm::Logging;

namespace
{
struct CapturedLog
{
  LogLevel level;
  std::string logger;
  std::string message;
};

class LogCapture
{
public:
  LogCapture()
  {
    Logging::instance().set_level(LogLevel::DEBUG);
    Logging::instance().set_sink([this](LogLevel l, const std::string & n, const std::string & m) {
      std::lock_guard<std::mutex> lock(mu_);
      records_.push_back({l, n, m});
    });
  }
  ~LogCapture()
  {
    Logging::instance().clear_sink();
    Logging::instance().set_level(LogLevel::INFO);
  }
  std::vector<CapturedLog> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return records_;
  }

private:
  mutable std::mutex mu_;
  std::vector<CapturedLog> records_;
};
}  // namespace

TEST(Logging, StreamMacrosRouteToCustomSink)
{
  LogCapture cap;
  PACKML_INFO_STREAM("test", "hello " << 42);
  PACKML_WARN_STREAM("test", "warning");
  const auto records = cap.snapshot();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].level, LogLevel::INFO);
  EXPECT_EQ(records[0].logger, "test");
  EXPECT_EQ(records[0].message, "hello 42");
  EXPECT_EQ(records[1].level, LogLevel::WARN);
  EXPECT_EQ(records[1].message, "warning");
}

TEST(Logging, PrintfMacrosRouteToCustomSink)
{
  LogCapture cap;
  PACKML_DEBUG("test", "x=%d y=%s", 7, "abc");
  PACKML_ERROR("test", "boom");
  const auto records = cap.snapshot();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].level, LogLevel::DEBUG);
  EXPECT_EQ(records[0].message, "x=7 y=abc");
  EXPECT_EQ(records[1].level, LogLevel::ERROR);
  EXPECT_EQ(records[1].message, "boom");
}

TEST(Logging, LevelFilterSuppressesLowerSeverity)
{
  LogCapture cap;
  Logging::instance().set_level(LogLevel::WARN);
  PACKML_DEBUG_STREAM("test", "debug");
  PACKML_INFO_STREAM("test", "info");
  PACKML_WARN_STREAM("test", "warn");
  PACKML_ERROR_STREAM("test", "error");
  const auto records = cap.snapshot();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].level, LogLevel::WARN);
  EXPECT_EQ(records[1].level, LogLevel::ERROR);
}

TEST(Logging, ClearSinkRevertsToDefault)
{
  // Default sink writes to stdout/stderr; we just verify clearing the sink
  // means our captor stops receiving events.
  bool received = false;
  Logging::instance().set_sink([&](LogLevel, const std::string &, const std::string &) {
    received = true;
  });
  PACKML_INFO("test", "x");
  EXPECT_TRUE(received);

  received = false;
  Logging::instance().clear_sink();
  PACKML_INFO("test", "y");  // Goes to default sink (stdout); our flag stays false.
  EXPECT_FALSE(received);
}
