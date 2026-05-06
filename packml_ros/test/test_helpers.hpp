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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace packml_ros_test {

/// Spin a node in a background thread for test lifetime
class SpinHelper
{
public:
  explicit SpinHelper(rclcpp::Node::SharedPtr node)
  : exec_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
    running_(true)
  {
    exec_->add_node(node);
    thread_ = std::thread([this]() {
        while (running_.load()) {
          exec_->spin_some(std::chrono::milliseconds(5));
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
  }

  ~SpinHelper()
  {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::atomic<bool> running_;
  std::thread thread_;
};

/// Wait for a service to become available
template<typename ServiceT>
bool wait_for_service(
  typename rclcpp::Client<ServiceT>::SharedPtr client,
  std::chrono::seconds timeout = std::chrono::seconds(5))
{
  return client->wait_for_service(timeout);
}

/// Call a service synchronously (spin the node while waiting)
template<typename ServiceT>
typename ServiceT::Response::SharedPtr call_service_sync(
  rclcpp::Node::SharedPtr node,
  typename rclcpp::Client<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request,
  std::chrono::seconds timeout = std::chrono::seconds(5))
{
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  return nullptr;
}

/// Generate a unique node name to avoid conflicts between tests
inline std::string unique_node_name(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return prefix + "_" + std::to_string(counter.fetch_add(1));
}

}  // namespace packml_ros_test
