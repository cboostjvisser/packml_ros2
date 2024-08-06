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

#pragma  once

#include "QState"
#include "QFuture"
#include "packml_sm/common.hpp"
#include "packml_sm/states/state.hpp"
#include <qchar.h>

namespace packml_sm
{

struct ActingState : public PackmlState
{
public:
  static ActingState * Resetting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::RESETTING, stoppable, delay_ms_value);
  }

  static ActingState * Starting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::STARTING, stoppable, delay_ms_value);
  }

  static ActingState * Unholding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::UNHOLDING, stoppable, delay_ms_value);
  }

  static ActingState * Unsuspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::UNSUSPENDING, stoppable, delay_ms_value);
  }

  static ActingState * Holding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::HOLDING, stoppable, delay_ms_value);
  }

  static ActingState * Suspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::SUSPENDING, stoppable, delay_ms_value);
  }

  static ActingState * Execute(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::EXECUTE, stoppable, delay_ms_value);
  }

  static ActingState * Execute(QState * stoppable, std::function<int()> function_value)
  {
    return new ActingState(State::EXECUTE, stoppable, function_value);
  }

  static ActingState * Completing(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::COMPLETING, stoppable, delay_ms_value);
  }

  static ActingState * Aborting(int delay_ms_value = 200)
  {
    return new ActingState(State::ABORTING, delay_ms_value);
  }

  static ActingState * Clearing(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(State::CLEARING, abortable, delay_ms_value);
  }

  static ActingState * Stopping(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(State::STOPPING, abortable, delay_ms_value);
  }

  ActingState(State state_value, const char * name_value, int delay_ms_value = 200)
    : PackmlState(state_value, QString(name_value)), delay_ms(delay_ms_value) {}

  ActingState(State state_value, const QString & name_value, QState * super_state, int delay_ms_value = 200)
    : PackmlState(state_value, name_value, super_state), delay_ms(delay_ms_value) {}

  ActingState(State state_value, const char * name_value, QState * super_state, std::function<int()> function_value)
    : PackmlState(state_value, QString(name_value), super_state), function_(function_value) {}


  ActingState(State state_value, int delay_ms_value = 200)
    : PackmlState(state_value, to_string(state_value).c_str()), delay_ms(delay_ms_value) {}

  ActingState(State state_value, QState * super_state, int delay_ms_value = 200)
    : PackmlState(state_value, to_string(state_value).c_str(), super_state), delay_ms(delay_ms_value) {}

  ActingState(State state_value, QState * super_state, std::function<int()> function_value)
    : PackmlState(state_value, to_string(state_value).c_str(), super_state), function_(function_value) {}

  bool setOperationMethod(std::function<int()> function_value)
  {
    function_ = function_value;
    return true;
  }
  virtual void operation();
  virtual ~ActingState() {}

protected:
  virtual void onEntry(QEvent * e);
  virtual void onExit(QEvent * e);

private:
  int delay_ms;
  std::function<int()> function_;
  QFuture<void> function_state_;
};

// TODO: needed?
typedef ActingState DualState;


}  // namespace packml_sm