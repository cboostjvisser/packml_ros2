// Copyright (c) 2016 Shaun Edwards
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include "packml_ros/packml_ros.hpp"
#include <rclcpp/executor.hpp>
using ::testing::Return;

class MockSMNode : public SMNode
{
public:
  explicit MockSMNode(rclcpp::Node::SharedPtr node)
  : SMNode(node) {}

  MOCK_METHOD0(getCurrentState, packml_sm::State());
};


TEST(Packml_ros, constructor_test)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  SMNode thenode(node);
  rclcpp::spin_some(node);
}


TEST(Packml_ros, test_transRequest_commands)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto transition_client_ = node->create_client<packml_msgs::srv::Transition>("transition");
  SMNode thenode(node);
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 1;
  auto result_future = transition_client_->async_send_request(trans);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 6;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 2;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 100;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 101;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 4;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 102;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 7;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 5;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 1;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 6;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 3;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 100;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 105;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    if (result->success == true) {
      check1 = true;
    }
  }
  ASSERT_TRUE(check1 == true);
  std::this_thread::sleep_for(std::chrono::seconds(10));
}

TEST(Packml_ros, test_statusRequest_commands_2)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::STOPPED));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_3)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::STARTING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_4)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::IDLE));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_5)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::SUSPENDED));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_6)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::EXECUTE));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_7)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::STOPPING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_8)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::ABORTING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_9)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::ABORTED));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_10)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::HOLDING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_11)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::HELD));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_12)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::UNHOLDING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_13)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::SUSPENDING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_14)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::UNSUSPENDING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_15)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::RESETTING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_16)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::COMPLETING));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_17)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::COMPLETE));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_wrong)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(packml_sm::State::UNDEFINED));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(
      node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_execute_method)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  SMNode thenode(node);
  int result = thenode.myExecuteMethod();
  EXPECT_EQ(result, 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::thread thr(qtWorker, argc, argv);
  // qtWorker(argc, argv);
  while (NULL == QCoreApplication::instance()) {
    printf("Waiting for QCore application to start\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  thr.detach();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
