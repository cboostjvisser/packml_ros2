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
// Example C++ PackML "Equipment Module" node that can be managed by the manager.

#include <rclcpp/rclcpp.hpp>
#include "packml_ros/interface/packml_interface.hpp"

class ExampleCppPackmlNode : public PackmlNodeInterface
{
public:
  explicit ExampleCppPackmlNode(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    init(node);
    RCLCPP_INFO(node_->get_logger(), "C++ PackML equipment module initialized");
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    RCLCPP_INFO(node_->get_logger(), "State transition to %s requested - accepted",
                packml_sm::to_string(state).c_str());
    return true;
  }

  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    RCLCPP_INFO(node_->get_logger(), "Mode transition to %s requested - accepted",
                packml_sm::to_string(mode).c_str());
    return true;
  }

  void on_status_changed() override
  {
    RCLCPP_INFO(node_->get_logger(), "Status changed - state: %s, mode: %s",
                packml_sm::to_string(get_current_packml_state()).c_str(),
                packml_sm::to_string(get_current_packml_mode()).c_str());
  }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_cpp_packml_node");
  ExampleCppPackmlNode packml_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
