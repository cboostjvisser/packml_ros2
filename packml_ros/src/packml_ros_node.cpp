// Copyright (c) 2017 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//


#include <QtCore>

#include <chrono>
#include <thread>
#include "packml_ros/packml_ros-new.hpp"
// #include "packml_ros/packml_ros.hpp"


int main(int argc, char * argv[])
{
  // Start node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  // Spin all the Qt components as a thread
  std::thread thr(qtWorker, argc, argv);
  while (NULL == QCoreApplication::instance()) {
    printf("Waiting for QCore application to start\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  thr.detach();

  // Construction fails if the state machine cannot start. Exiting non-zero is the whole point:
  // an unhandled exception here would abort() with a core dump, and a node that stayed up would
  // advertise services it can never honour. A non-zero exit is what a launch file's
  // on_exit/respawn and a systemd unit both already know how to act on.
  try {
    SMNode_new thenode(node);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL_STREAM(node->get_logger(), "packml_ros_node is shutting down: " << e.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
