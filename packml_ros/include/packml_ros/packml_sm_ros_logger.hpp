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
// Optional adapter: forward packml_sm logging to ROS2 (rclcpp) loggers.
//
// Usage from any ROS-aware translation unit (e.g. SMNode constructor):
//
//   #include "packml_ros/packml_sm_ros_logger.hpp"
//   ...
//   packml_ros::install_packml_sm_ros_logger();
//
// Once installed, every PACKML_* call inside the state machine library is
// republished via RCLCPP_* with the corresponding severity.  The library
// itself remains framework-agnostic.
// ---

#ifndef PACKML_ROS__PACKML_SM_ROS_LOGGER_HPP_
#define PACKML_ROS__PACKML_SM_ROS_LOGGER_HPP_

#include <string>

#include "packml_sm/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace packml_ros
{

inline void install_packml_sm_ros_logger()
{
  packml_sm::Logging::instance().set_level(packml_sm::LogLevel::DEBUG);
  packml_sm::Logging::instance().set_sink(
    [](packml_sm::LogLevel level, const std::string & logger_name, const std::string & message)
    {
      auto logger = rclcpp::get_logger(logger_name);
      switch (level) {
        case packml_sm::LogLevel::DEBUG:
          RCLCPP_DEBUG(logger, "%s", message.c_str()); break;
        case packml_sm::LogLevel::INFO:
          RCLCPP_INFO(logger,  "%s", message.c_str()); break;
        case packml_sm::LogLevel::WARN:
          RCLCPP_WARN(logger,  "%s", message.c_str()); break;
        case packml_sm::LogLevel::ERROR:
          RCLCPP_ERROR(logger, "%s", message.c_str()); break;
      }
    });
}

inline void uninstall_packml_sm_ros_logger()
{
  packml_sm::Logging::instance().clear_sink();
}

}  // namespace packml_ros

#endif  // PACKML_ROS__PACKML_SM_ROS_LOGGER_HPP_
