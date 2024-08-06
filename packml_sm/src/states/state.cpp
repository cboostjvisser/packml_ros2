// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 Dejanira Araiza Illan, ROS-Industrial Asia Pacific
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

#include <QtConcurrent/QtConcurrent>

#include <iostream>
#include <chrono>

#include "packml_sm/states/state.hpp"

namespace packml_sm
{

void PackmlState::onEntry(QEvent * /*e*/)  // NOLINT(readability/casting)
{
  std::cout << "Entering state: " << name_.toUtf8().constData() << "(" << state_ << ")" <<
    std::endl;
  emit stateEntered(state_, name_);
  enter_time_ = std::chrono::system_clock::now();
}


void PackmlState::onExit(QEvent * /*e*/)  // NOLINT(readability/casting)
{
  std::cout << "Exiting state: " << name_.toUtf8().constData() << "(" << state_ << ")" <<
    std::endl;
  exit_time_ = std::chrono::system_clock::now();
  cummulative_time_ = cummulative_time_ + (exit_time_ - enter_time_);
  std::cout << "Updating cummulative time, for state: " << name_.toUtf8().constData() << "(" <<
    state_ << ") to: " << cummulative_time_.count() << std::endl;
}

}  // namespace packml_sm
