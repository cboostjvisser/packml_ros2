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

#pragma once

#include "QState"
#include "packml_sm/common.hpp"

namespace packml_sm
{

struct PackmlState : public QState
{
Q_OBJECT

public:
  // PackmlState(State state_value, QString name_value)
  //   : state_(state_value),
  //     name_(name_value),
  //     cummulative_time_(0) {}

  PackmlState(State state_value, QString name_value, QState * super_state = nullptr)
    : QState(super_state),
      state_(state_value),
      name_(name_value),
      cummulative_time_(0) {}

  State state() const {return state_;}
  const std::string name() const {return name_.toStdString();}
  virtual ~PackmlState() {}

signals:
  void stateEntered(int value, QString name);

protected:
  State state_;
  QString name_;

  std::chrono::time_point<std::chrono::system_clock> enter_time_;
  std::chrono::time_point<std::chrono::system_clock> exit_time_;
  std::chrono::duration<double> cummulative_time_;

  virtual void onEntry(QEvent * e);
  virtual void operation() {}
  virtual void onExit(QEvent * e);
};

}  // namespace packml_sm
