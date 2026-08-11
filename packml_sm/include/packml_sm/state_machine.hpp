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

#include <atomic>
#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <vector>
#include <qcoreevent.h>
#include <qstatemachine.h>
#include <QThreadPool>
#include "QEvent"
#include "QAbstractTransition"
// #include "packml_sm/events.hpp"
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
#include "packml_sm/transitions/cmd_transition.hpp"
// #include "packml_sm/states_generator.hpp"
// #include "packml_sm/transitions.hpp"
#include "packml_sm/logging.hpp"

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
        auto * ce = static_cast<CmdEvent *>(event);
        if (event->isAccepted())
        {
          PACKML_INFO_STREAM("packml_sm", "Accepted command: " << ce->cmd);
        }
        else
        {
          PACKML_WARN_STREAM("packml_sm", "No transition claimed command: " << ce->cmd);
        }
        // Never set_value() directly: the event's destructor answers too, and whichever arrives
        // second must be the one that does nothing rather than the one that throws.
        ce->answer(event->isAccepted());
      }
      else if (event->type() == PACKML_ERROR_EVENT_TYPE)
      {
        // Surface the error code to the owning StateMachine, if it has
        // subscribed via on_error_code.
        auto * ee = static_cast<ErrorEvent *>(event);
        if (on_error_code) {
          on_error_code(ee->code);
        }
      }
      else if (event->type() == PACKML_STATE_COMPLETE_EVENT_TYPE)
      {
        // We can do something here with these custom packml events
      }
      else
      {
        PACKML_DEBUG("packml_sm", "This is not a user defined event!");
      }
    }

  public:
    // Optional hook (set by the owning StateMachine) so error codes raised by
    // ActingState::operation() can be observed via getLastErrorCode().
    std::function<void(int)> on_error_code;
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
  *
  * Discarding the result leaves a machine that looks alive and answers nothing: it never enters
  * a state, so it never publishes one, and every command is refused. Callers must decide what a
  * failure means to them.
  */
  [[nodiscard]] virtual bool activate() = 0;


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

  /// Change to @p mode using the provided @p avail map to configure which
  /// states are active.  The base-class implementation simply delegates to
  /// changeMode(mode) so that concrete classes that do not override it still
  /// compile.  StateMachine provides a full override.
  virtual std::expected<bool, std::string> changeMode(ModeType mode, AvailableStates avail)
  {
    (void)avail;
    return changeMode(mode);
  }

  virtual std::expected<bool, std::string> changeState(TransitionCmd command) = 0;


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
  * @param delay_ms Default delay for acting states (ms). Lower values speed
  *        up tests.  Production default: 200.
  */
  static std::shared_ptr<StateMachine> singleCycleSM(int delay_ms = 200);


  /**
  * @brief Function to create a continuous cycle state machine (executes forever until stopped)
  * @param delay_ms Default delay for acting states (ms). Lower values speed
  *        up tests.  Production default: 200.
  */
  static std::shared_ptr<StateMachine> continuousCycleSM(int delay_ms = 200);


  /**
  * @brief Function to activate the state machine
  */
  [[nodiscard]] bool activate();


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
  * @brief Bind a custom operation to any acting state by PackML state enum.
  *        Returns false if the state is not found or not an ActingState.
  */
  bool setStateOperation(State state, std::function<int()> method);

  /**
  * @brief Like setStateOperation(), but `method` receives a std::stop_token that
  *        becomes stop_requested() the moment this state is asked to exit EARLY --
  *        i.e. a transition out of it (an operator's HOLD/SUSPEND/ABORT/STOP, or any
  *        other command accepted while `method` is still running) was accepted before
  *        `method` returned on its own. A `method` that observes the token can return
  *        promptly instead of blocking the state machine's own exit for however long
  *        its own internal wait/timeout would otherwise take. Returns false if the
  *        state is not found or not an ActingState.
  */
  bool setInterruptibleStateOperation(State state, std::function<int(std::stop_token)> method);

  /**
  * @brief Returns the cumulative time (seconds) spent in the given state
  *        since the SM was created.  Returns 0 if the state is not found.
  */
  double getStateCumulativeTime(State state) const;

  /**
  * @brief Names of the acting states that do NOT own an ERROR transition reaching their
  *        declared failure target.  Must be empty: activate() refuses to start the machine
  *        otherwise, because a failure of such a state's bound operation posts an event that
  *        matches no transition, which Qt discards in silence -- leaving the machine wedged in
  *        that state with no status change, no alarm and no log.
  */
  std::vector<std::string> errorEscapeReport() const;


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
    return state_value_.load(std::memory_order_acquire);
  }

  /**
  * @brief Returns the most recent ModeType that was applied via changeMode().
  *        Defaults to ModeType{} (== 0) before the first successful change.
  *
  * Reads the applied mode back out of the generator under mode_mask_mutex(),
  * rather than from a cached copy: the mode and the availability mask it
  * selects are stored together and written in one locked pass, so the two can
  * never be observed disagreeing.  Out of line because StatesGenerator is only
  * forward-declared here.
  */
  ModeType getCurrentMode() const;

  /**
  * @brief Returns the AvailableStates mask that was applied with the most
  *        recent successful changeMode() call.  Same storage and same lock as
  *        getCurrentMode().
  */
  AvailableStates getAvailableStates() const;

  /**
  * @brief Returns the last error code reported by an Acting state's bound
  *        function (the non-zero value returned by setExecute / setResetting
  *        callbacks).  Returns 0 when no error has been recorded since the
  *        last clear() / reset().
  */
  int getLastErrorCode() const
  {
    return last_error_code_;
  }

  /**
  * @brief Test/instrumentation hook -- internal acting states call this when
  *        their bound function returns a non-zero error code.  Not intended
  *        for application code.
  */
  void setLastErrorCode(int code)
  {
    last_error_code_ = code;
  }

  virtual std::expected<bool, std::string> changeMode(ModeType mode);

  virtual std::expected<bool, std::string> changeMode(ModeType mode, AvailableStates avail);

  virtual std::expected<bool, std::string> changeState(TransitionCmd mode);

  std::function<void(State value, QString name)> on_state_changed = [](packml_sm::State value, QString name){
      PACKML_INFO_STREAM("packml_sm", "Default callback; State changed to: " << name.toStdString() << "(" << value << ")");
    };
  std::function<void(ModeType value)> on_mode_changed = [](packml_sm::ModeType value) {
      PACKML_INFO_STREAM("packml_sm", "Default callback; Mode changed to: " << packml_sm::to_string(value));
    };

  /**
  * @brief Class destructor
  */
  virtual ~StateMachine();

protected:
  /**
  * @brief Block until every ActingState's currently-running operation
  *        (`function_state_` future) has returned.  Safe to call before
  *        or after the inner QStateMachine has been stopped.  Bounded only
  *        by the user's bound-function duration -- supports lambdas of
  *        any length without time-based hacks.
  */
  void drainActingStates();

  /// Point every ActingState this machine owns at operation_pool_. Called once per cycle init,
  /// after the states exist and before the machine can be activated.
  void bindOperationThreadPool();

  /**
  * @brief Ask every ActingState's in-flight bound operation to stop, without waiting for any of
  *        them.  The request onExit() makes on every ordinary path, made on the one path Qt skips:
  *        QStateMachine::stop() does not call onExit() on active states.
  *
  *        Called twice by deactivate() on purpose.  Before stop(), so an interruptible operation
  *        can return early instead of the drain waiting out its full duration.  After the machine
  *        has stopped, because a transition could still have fired in between and onEntry() gives
  *        the new visit a FRESH stop source -- once stopped, no further entry can happen, so the
  *        second call is the one that makes the coverage complete.
  */
  void requestActingStatesStop();

  /**
  * @brief Hand @p command to the inner QStateMachine and block until transition selection has
  *        answered it.  Returns true only when a transition claimed the command.
  *
  *        Returns false -- promptly, and with an ERROR naming which one it is -- in the four
  *        cases where an answer can never arrive: the machine is not running, the caller is the
  *        Qt event-loop thread itself, the caller is a bound operation the machine's exit path
  *        is waiting on, and the machine stops while the command is still queued.  None of
  *        those is a rejected transition, and none of them may look like one in the log.
  */
  bool postCommand(TransitionCmd command);

  /**
  * @brief Class constructor
  */
  StateMachine();

  /// This machine's OWN pool for acting-state bound operations.
  ///
  /// Declared before `gen` so it is destroyed after the states that submit to it. It exists because
  /// ~StateMachine() has to wait for those operations, and waiting on
  /// QThreadPool::globalInstance() waited for every other QtConcurrent user in the process too --
  /// an assumption about who else is in the address space, not a guarantee this class can make.
  /// Owning the pool also means the operations draw threads from a budget nobody else can exhaust.
  QThreadPool operation_pool_;

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
  std::atomic<State> state_value_;


  /**
  * @brief Name of the current state
  */
  QString state_name_;

  /**
  * @brief Last error code reported by an acting-state operation.
  */
  int last_error_code_{0};


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

  void init(int delay_ms = 200);

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

  void init(int delay_ms = 200);

  /**
  * @brief Class destructor
  */
  virtual ~SingleCycle() {}
};

}  // namespace packml_sm

#endif  // PACKML_SM__STATE_MACHINE_HPP_
