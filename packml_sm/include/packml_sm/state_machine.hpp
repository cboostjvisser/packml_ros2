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


#ifndef PACKML_SM__STATE_MACHINE_HPP_
#define PACKML_SM__STATE_MACHINE_HPP_

#include <QtGui>

#include <functional>
#include <memory>
#include <qcoreevent.h>
#include <qstatemachine.h>
#include "QEvent"
#include "QAbstractTransition"
// #include "packml_sm/events.hpp"
#include <iostream>
#include <expected>

#include "packml_sm/common.hpp"
// #include "packml_sm/events.hpp"
#include "packml_sm/events/sc_event.hpp"
#include "packml_sm/states/toplevel_states.hpp"

#include "packml_sm/states/acting_state.hpp"
#include "packml_sm/states/wait_state.hpp"

#include "packml_sm/events/cmd_event.hpp"
#include "packml_sm/events/error_event.hpp"
#include "packml_sm/events/sc_event.hpp"
// #include "packml_sm/states_generator.hpp"
// #include "packml_sm/transitions.hpp"
// #include "rclcpp/rclcpp.hpp"

namespace packml_sm
{
   class StatesGenerator;


  class PackmlStateMachine : public QStateMachine
  {
    // https://stackoverflow.com/questions/4818863/how-can-i-detect-ignored-rejected-posted-qevent-to-qstatemachine
    void endSelectTransitions(QEvent *event) override
    {
      if (event->type() == PACKML_CMD_EVENT_TYPE)
      {
        if (event->isAccepted())
        {
          prom.set_value(true);
          std::cout << "We have accepted the event!" << std::endl;
        }
        else
        {
          prom.set_value(false);
          std::cout << "Event has not been accepted!" << std::endl;
        }
      }
      else if (event->type() == PACKML_ERROR_EVENT_TYPE || event->type() == PACKML_STATE_COMPLETE_EVENT_TYPE)
      {}
      else
      {
        std::cout << "This is not a user defined event!" << std::endl;
      }
    }
  public:
    std::promise<bool> prom;
  };



/**
 * @brief The StateMachineInterface class defines a implementation independent interface
 * to a PackML state machine.
 */
class StateMachineInterface
{
public:
  /**
  * @brief Function to activate the state machine
  */
  virtual bool activate() = 0;


  /**
  * @brief Function to bind a function for the Execute state
  * @param execute_method - a function for the Execute state
  */
  virtual bool setExecute(std::function<int()> execute_method) = 0;


  /**
  * @brief Function to bind a function for the Resetting state
  * @param resetting_method - a function for the Resetting state
  */
  virtual bool setResetting(std::function<int()> resetting_method) = 0;


  /**
  * @brief Function that returns whether the state machine is active or not
  */
  virtual bool isActive() = 0;


  /**
  * @brief Function that returns the current state of the state machine
  */
  virtual State getCurrentState() = 0;

  virtual std::expected<bool, std::string> changeMode(ModeType mode) = 0;


  /**
  * @brief Function that implements the start state
  */
  virtual bool start();


  /**
  * @brief Function that implements the clear state
  */
  virtual bool clear();


  /**
  * @brief Function that implements the reset state
  */
  virtual bool reset();


  /**
  * @brief Function that implements the hold state
  */
  virtual bool hold();


  /**
  * @brief Function that implements the unhold state
  */
  virtual bool unhold();


  /**
  * @brief Function that implements the suspend state
  */
  virtual bool suspend();


  /**
  * @brief Function that implements the unsuspend state
  */
  virtual bool unsuspend();


  /**
  * @brief Function that implements the stop state
  */
  virtual bool stop();


  /**
  * @brief Function that implements the abort state
  */
  virtual bool abort();

protected:
  /**
  * @brief Function that binds a QT event to the function for the state start
  */
  virtual bool _start() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state clear
  */
  virtual bool _clear() = 0;

  /**
  * @brief Function that binds a QT action to the function for the state reset
  */
  virtual bool _reset() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state hold
  */
  virtual bool _hold() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state unhold
  */
  virtual bool _unhold() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state suspend
  */
  virtual bool _suspend() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state unsuspend
  */
  virtual bool _unsuspend() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state stop
  */
  virtual bool _stop() = 0;


  /**
  * @brief Function that binds a QT action to the function for the state abort
  */
  virtual bool _abort() = 0;
};


/**
* @brief Function to start and run a state machine
* @param argc - number of command line arguments
* @param argv - list of command line arguments
*/
void init(int argc, char * argv[]);


/**
* @brief Class that implements a state machine object
*/
class StateMachine : public QObject, public StateMachineInterface, public std::enable_shared_from_this<StateMachine>
{
  Q_OBJECT

public:
  /**
  * @brief Function to create a single cycle state machine (executes once)
  */
  static std::shared_ptr<StateMachine> singleCyleSM();


  /**
  * @brief Function to create a continuous cycle state machine (executes forever until stopped)
  */
  static std::shared_ptr<StateMachine> continuousCycleSM();


  /**
  * @brief Function to activate the state machine
  */
  bool activate();


  /**
  * @brief Function to deactivate the state machine
  */
  bool deactivate();


  /**
  * @brief Function to bind the Execute state to a function
  * @param execute_method - Function for the Execute state
  */
  bool setExecute(std::function<int()> execute_method);


  /**
  * @brief Function to bind the Resetting state to a function
  * @param execute_method - Function for the Resetting state
  */
  bool setResetting(std::function<int()> resetting_method);


  /**
  * @brief Function that returns whether the state machine is active or not
  */
  bool isActive()
  {
    return sm_internal_.isRunning();
  }


  /**
  * @brief Function that returns the current state of the state machine
  */
  State getCurrentState()
  {
    return state_value_;
  }

  virtual std::expected<bool, std::string> changeMode(ModeType mode);


  /**
  * @brief Class destructor
  */
  virtual ~StateMachine() {}

protected:
  /**
  * @brief Class constructor
  */
  StateMachine();

  std::shared_ptr<StatesGenerator> gen;

  /**
  * @brief Function that binds a QT action to the function for the state start
  */
  virtual bool _start();


  /**
  * @brief Function that binds a QT action to the function for the state clear
  */
  virtual bool _clear();


  /**
  * @brief Function that binds a QT action to the function for the state reset
  */
  virtual bool _reset();


  /**
  * @brief Function that binds a QT action to the function for the state hol
  */
  virtual bool _hold();


  /**
  * @brief Function that binds a QT action to the function for the state unhold
  */
  virtual bool _unhold();


  /**
  * @brief Function that binds a QT action to the function for the state suspend
  */
  virtual bool _suspend();


  /**
  * @brief Function that binds a QT action to the function for the state unsuspend
  */
  virtual bool _unsuspend();


  /**
  * @brief Function that binds a QT action to the function for the state stop
  */
  virtual bool _stop();


  /**
  * @brief Function that binds a QT action to the function for the state abort
  */
  virtual bool _abort();


  /**
  * @brief Number of the current state
  */
  State state_value_;


  /**
  * @brief Name of the current state
  */
  QString state_name_;


  /**
  * @brief Waiting for event to transition to abort state
  */
  PackmlSuperState * abortable_;


  /**
  * @brief Waiting for event to transition to stop state
  */
  PackmlSuperState * stoppable_;


  /**
  * @brief Waiting for event to transition outside hold state
  */
  WaitState * held_;


  /**
  * @brief Waiting for event to transition outside idle state
  */
  WaitState * idle_;

  /**
  * @brief Waiting for event to transition outside suspended state
  */
  WaitState * suspended_;


  /**
  * @brief Waiting for event to transition outside stopped state
  */
  WaitState * stopped_;


  /**
  * @brief Waiting for event to transition outside complete state
  */
  WaitState * complete_;


  /**
  * @brief Waiting for event to transition outside abort state
  */
  WaitState * aborted_;


  /**
  * @brief Internal state definiton for unholding
  */
  ActingState * unholding_;


  /**
  * @brief Internal state definiton for holding
  */
  ActingState * holding_;


  /**
  * @brief Internal state definiton for starting
  */
  ActingState * starting_;


  /**
  * @brief Internal state definiton for completing
  */
  ActingState * completing_;


  /**
  * @brief Internal state definiton for resetting
  */
  ActingState * resetting_;


  /**
  * @brief Internal state definiton for unsuspending
  */
  ActingState * unsuspending_;


  /**
  * @brief Internal state definiton for suspending
  */
  ActingState * suspending_;


  /**
  * @brief Internal state definiton for stopping
  */
  ActingState * stopping_;


  /**
  * @brief Internal state definiton for clearing
  */
  ActingState * clearing_;


  /**
  * @brief Internal state definiton for aborting
  */
  ActingState * aborting_;


  /**
  * @brief Internal state definiton for execute
  */
  DualState * execute_;


  /**
  * @brief QT state machine object
  */
  PackmlStateMachine sm_internal_;

public slots:
  /**
  * @brief Function to start a state
  * @param value - state number
  * @param name - state name
  */
  void setState(State value, QString name);

signals:
  /**
  * @brief Function to trigger QT objects when the state has changed
  * @param value - new state number
  * @param name - new state name
  */
  void stateChanged(State value, QString name);
};


/**
* @brief Class for the definition of a continuous cycle state machine
*/
class ContinuousCycle : public StateMachine
{
  Q_OBJECT

public:
  /**
  * @brief Class constructor
  */
  ContinuousCycle();

  void init();

  /**
  * @brief Class desstructor
  */
  virtual ~ContinuousCycle() {}
};


/**
* @brief Class for the definition of a single cycle state machine
*/
class SingleCycle : public StateMachine
{
  Q_OBJECT

public:
  /**
  * @brief Class constructor
  */
  SingleCycle();

  void init();

  /**
  * @brief Class destructor
  */
  virtual ~SingleCycle() {}
};

}  // namespace packml_sm

#endif  // PACKML_SM__STATE_MACHINE_HPP_
