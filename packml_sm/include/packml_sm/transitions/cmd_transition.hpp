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

#pragma once

#include <iostream>

#include "QEvent"
#include "QAbstractTransition"
#include "packml_sm/states/state.hpp"
#include "packml_sm/common.hpp"

namespace packml_sm
{

/**
* @brief Class that implements transitions between the states of a standard PackML state machine
*/
class CmdTransition : public QAbstractTransition
{
public:
  /**
  * @brief Function to transition to the clear state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * clear(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::CLEAR, "clear", from, to);
  }
  static CmdTransition * clear()
  {
    return new CmdTransition(TransitionCmd::CLEAR, to_string(TransitionCmd::CLEAR).c_str());
  }


  /**
  * @brief Function to transition to the start state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * start(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::START, "start", from, to);
  }
    static CmdTransition * start()
  {
    return new CmdTransition(TransitionCmd::START, to_string(TransitionCmd::START).c_str());
  }


  /**
  * @brief Function to transition to the stop state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * stop(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::STOP, "stop", from, to);
  }
  static CmdTransition * stop()
  {
    return new CmdTransition(TransitionCmd::STOP, to_string(TransitionCmd::STOP).c_str());
  }


  /**
  * @brief Function to transition to the hold state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * hold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::HOLD, "hold", from, to);
  }
  static CmdTransition * hold()
  {
    return new CmdTransition(TransitionCmd::HOLD, to_string(TransitionCmd::HOLD).c_str());
  }

  /**
  * @brief Function to transition to the unhold state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * unhold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::UNHOLD, "unhold", from, to);
  }
  static CmdTransition * unhold()
  {
    return new CmdTransition(TransitionCmd::UNHOLD, to_string(TransitionCmd::UNHOLD).c_str());
  }



  /**
  * @brief Function to transition to the abort state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * abort(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::ABORT, "abort", from, to);
  }
  static CmdTransition * abort()
  {
    return new CmdTransition(TransitionCmd::ABORT, to_string(TransitionCmd::ABORT).c_str());
  }


  /**
  * @brief Function to transition to the reset state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * reset(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::RESET, "reset", from, to);
  }
  static CmdTransition * reset()
  {
    return new CmdTransition(TransitionCmd::RESET, to_string(TransitionCmd::RESET).c_str());
  }

  // /**
  // * @brief Function to transition to the e stop state
  // * @param from - original state
  // * @param to - ending state
  // */
  // static CmdTransition * estop(PackmlState & from, PackmlState & to)
  // {
  //   return new CmdTransition(TransitionCmd::ESTOP, "estop", from, to);
  // }


  /**
  * @brief Function to transition to the abort state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * suspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::SUSPEND, "suspend", from, to);
  }
  static CmdTransition * suspend()
  {
    return new CmdTransition(TransitionCmd::SUSPEND, to_string(TransitionCmd::SUSPEND).c_str());
  }


  /**
  * @brief Function to transition to the unsuspend state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * unsuspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(TransitionCmd::UNSUSPEND, "unsuspend", from, to);
  }
  static CmdTransition * unsuspend()
  {
    return new CmdTransition(TransitionCmd::UNSUSPEND, to_string(TransitionCmd::UNSUSPEND).c_str());
  }

  /**
  * @brief Function to define a transition from a triggering event
  * @param cmd_value - number of the new requested state
  * @param name_value - name of the new requested state
  */
  CmdTransition(const TransitionCmd & cmd_value, const QString & name_value)
  : cmd(cmd_value), name(name_value) {}


  /**
  * @brief Function to define a transition from a triggering event
  * @param cmd_value - number of the new requested state
  * @param name_value - name of the new requested state
  * @param from - original state
  * @param to - ending state
  */
  CmdTransition(
    const TransitionCmd & cmd_value, const QString & name_value,
    PackmlState & from, PackmlState & to);

protected:
  /**
  * @brief Function to check if a transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e);


  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}


  /**
  * @brief Number of the state
  */
  TransitionCmd cmd;


  /**
  * @brief Name of the state
  */
  QString name;
};

} // namespace packml_sm