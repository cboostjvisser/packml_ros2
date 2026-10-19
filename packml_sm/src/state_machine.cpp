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

#include "packml_sm/state_machine.hpp"

#include <QCoreApplication>
#include <QEventLoop>
#include <QThread>
#include <QThreadPool>

#include "packml_sm/common.hpp"
#include "packml_sm/states/wait_state.hpp"
#include "packml_sm/states_generator.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"
#include "packml_sm/transitions/sc_transition.hpp"
#include "packml_sm/transitions/error_transition.hpp"

// #include "packml_sm/events.hpp"
#include "packml_sm/states/acting_state.hpp"
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "packml_sm/logging.hpp"

// Included for its registration side effect, not for its constants: it publishes packml_sm's own
// mode vocabulary so is_known_mode() answers for a program that links the library and defines no
// modes of its own. This translation unit is used by anything that runs a state machine, which is
// what makes the registration reliable -- a file that only held the include could be initialised
// lazily or dropped.
#include "packml_sm/default_modes.hpp"

namespace packml_sm {

bool StateMachineInterface::start() {
  return _start();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::IDLE:
  //   _start();
  //   return true;
  // default:
  //   std::cout << "Ignoring START command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::clear() {
  return _clear();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::ABORTED:
  //   return _clear();
  //   return true;
  // default:
  //   std::cout << "Ignoring CLEAR command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::reset() {
  return _reset();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::COMPLETE:
  // case State::STOPPED:
  //   _reset();
  //   return true;
  // default:
  //   std::cout << "Ignoring RESET command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::hold() {
  return _hold();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::EXECUTE:
  //   _hold();
  //   return true;
  // default:
  //   std::cout << "Ignoring HOLD command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::unhold() {
  return _unhold();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::HELD:
  //   _unhold();
  //   return true;
  // default:
  //   std::cout << "Ignoring HELD command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::suspend() {
  return _suspend();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::EXECUTE:
  //   _suspend();
  //   return true;
  // default:
  //   std::cout << "Ignoring SUSPEND command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::unsuspend() {
  return _unsuspend();
  // return true;
  // switch (State(getCurrentState())) {
  // case State::SUSPENDED:
  //   _unsuspend();
  //   return true;
  // default:
  //   std::cout << "Ignoring UNSUSPEND command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::stop() {
  return _stop();
  // return true;
  // switch (State(getCurrentState())) {
  // // case StatesEnum::STOPPABLE:
  // case State::STARTING:
  // case State::IDLE:
  // case State::SUSPENDED:
  // case State::EXECUTE:
  // case State::HOLDING:
  // case State::HELD:
  // case State::SUSPENDING:
  // case State::UNSUSPENDING:
  // case State::UNHOLDING:
  // case State::COMPLETING:
  // case State::COMPLETE:
  //   _stop();
  //   return true;
  // default:
  //   std::cout << "Ignoring STOP command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::abort() {
  return _abort();
  // return true;
  // switch (State(getCurrentState())) {
  // // case StatesEnum::ABORTABLE:
  // case State::STOPPED:
  // case State::STARTING:
  // case State::IDLE:
  // case State::SUSPENDED:
  // case State::EXECUTE:
  // case State::HOLDING:
  // case State::HELD:
  // case State::SUSPENDING:
  // case State::UNSUSPENDING:
  // case State::UNHOLDING:
  // case State::COMPLETING:
  // case State::COMPLETE:
  // case State::CLEARING:
  // case State::STOPPING:
  //   _abort();
  //   return true;
  // default:
  //   std::cout << "Ignoring ABORT command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

QCoreApplication *a;
void init(int argc, char *argv[]) {
  if (NULL == QCoreApplication::instance()) {
    printf("Starting QCoreApplication\n");
    a = new QCoreApplication(argc, argv);
  }
}

std::vector<std::string> StateMachine::errorEscapeReport() const {
  if (!gen) {
    return {};
  }
  return gen->error_escape_report();
}

bool StateMachine::activate() {
  // Refuse to run a graph in which some acting state's failure has nowhere to go. This is the
  // one moment where that is still cheap to say out loud: once the machine is running, such a
  // failure is indistinguishable from a state that simply takes a long time.
  const auto missing_escapes = errorEscapeReport();
  if (!missing_escapes.empty()) {
    for (const auto & name : missing_escapes) {
      PACKML_ERROR_STREAM("packml_sm", "Acting state owns no ERROR transition to its declared "
        "failure target; a failure of its bound operation would wedge the machine: " << name);
    }
    return false;
  }

  // A state that holds when unbound (EXECUTE in a continuous-cycle machine) and has nothing bound
  // will do exactly that: sit there. That is the right behaviour -- better than inventing
  // production -- but it is invisible from outside, since a machine holding in EXECUTE on purpose
  // and one holding because nobody wrote its operation look identical. Say it once, here, rather
  // than let an integrator wonder why the line never produces.
  if (gen) {
    for (auto & kv : gen->states) {
      auto * acting = dynamic_cast<ActingState *>(kv.second);
      if (nullptr != acting && acting->holdsWhenUnbound() && !acting->hasBoundOperation()) {
        PACKML_ERROR_STREAM("packml_sm", "No operation is bound to " << kv.first <<
          "; the machine will enter it and hold there, producing nothing, until it is commanded "
          "out. Bind one with setExecute()/setStateOperation()/setInterruptibleStateOperation() "
          "-- or, if holding is what this deployment wants, bind an operation that says so.");
      }
    }
  }

  printf("Checking if QCore application is running\n");
  if (NULL == QCoreApplication::instance()) {
    printf("QCore application is not running, QCoreApplication must be created "
           "in main");
    printf(" thread for state machine to run\n");
    return false;
  }

  // The wait below is for the Qt loop to make progress, so it cannot be done from that loop's own
  // thread. Same reasoning, and the same refusal, as postCommand()'s event-loop-thread guard.
  if (QThread::currentThread() == QCoreApplication::instance()->thread()) {
    PACKML_ERROR_STREAM("packml_sm", "activate() called from the Qt event-loop thread itself; it "
      "waits for that loop to start the machine, so it can only be called from another thread");
    return false;
  }

  printf("Moving state machine to Qcore thread\n");
  sm_internal_.moveToThread(QCoreApplication::instance()->thread());
  this->moveToThread(QCoreApplication::instance()->thread());

  // Return only once the machine is genuinely running, mirroring what deactivate() already does
  // for stop().
  //
  // QStateMachine::start() is asynchronous -- it queues _q_start() onto the Qt loop -- so
  // returning before that lands would report true while the machine is still not running. Every
  // later isRunning() check would then be a check-then-act on a value about to change on its own,
  // and two of them decide teardown: deactivate() takes an early return that skips stop(), and
  // ~StateMachine() picks between deactivating and merely draining. A deactivate() (or a
  // destructor) landing inside that window would tear the machine down while the Qt loop was
  // still about to start it, walking a state graph whose objects were being freed underneath it.
  // Nothing downstream can defend against that; the window has to not exist.
  // The wait state is SHARED-OWNED and captured BY VALUE, not a set of locals captured by
  // reference. That is the whole difference between this and a stack use-after-free.
  //
  // QObject::disconnect() does not wait for a Qt::DirectConnection slot that is already running on
  // another thread -- it returns while the slot is still executing. So on the timeout path the Qt
  // thread can still be inside this lambda, taking the mutex and writing the flag, in a frame that
  // this function is about to pop; that is a stack-use-after-return under AddressSanitizer. The
  // success path has the same hole in miniature, because the lambda
  // releases the lock before notifying, so a waiter released at that instant can destroy the
  // condition variable while the notify is in flight.
  //
  // Deliberately NOT solved with deactivate()'s Qt::BlockingQueuedConnection barrier: that barrier
  // is only correct when the loop is known to be running. Here the failure being handled IS a
  // stalled loop, so a blocking queued call would never return and a 5 s timeout would become a
  // permanent hang -- strictly worse than the race it closes. Refcounting the state costs one
  // allocation per activate() and needs no cooperation from the loop at all.
  struct StartWait
  {
    std::mutex started_mutex;
    std::condition_variable started_cv;
    bool started = false;
  };
  auto wait_state = std::make_shared<StartWait>();
  auto conn = QObject::connect(
    &sm_internal_, &QStateMachine::started,
    &sm_internal_, [wait_state]() {
      {
        std::lock_guard<std::mutex> lk(wait_state->started_mutex);
        wait_state->started = true;
      }
      wait_state->started_cv.notify_all();
    },
    Qt::DirectConnection);
  sm_internal_.start();

  bool started = false;
  {
    std::unique_lock<std::mutex> lk(wait_state->started_mutex);
    started = wait_state->started_cv.wait_for(
      lk, std::chrono::seconds(5), [&wait_state] {return wait_state->started;});
  }
  QObject::disconnect(conn);

  if (!started) {
    PACKML_ERROR_STREAM("packml_sm", "State machine did not start within 5 s of start() being "
      "queued; the Qt event loop is not running or is blocked");
    return false;
  }

  printf("State machine thread created and started\n");
  return true;
}

// Drain every ActingState owned by this state machine: block until each
// state's bound function (`function_state_` future started in onEntry) has
// returned.  Required because QStateMachine::stop() does NOT invoke
// onExit() on currently active states (Qt5 documented behaviour), so the
// QtConcurrent::run worker can outlive the SM and SIGSEGV when posting a
// StateCompleteEvent / ErrorEvent on a destroyed `machine()`.
//
// Bounded only by the user's own lambda duration -- safe for arbitrarily
// long execute / resetting / ... callbacks.
void StateMachine::drainActingStates() {
  if (!gen) {
    return;
  }
  for (auto & kv : gen->states) {
    if (auto * acting = dynamic_cast<ActingState *>(kv.second)) {
      acting->waitForOperationFinished();
    }
  }
}

void StateMachine::bindOperationThreadPool() {
  if (!gen) {
    return;
  }
  for (auto & kv : gen->states) {
    if (auto * acting = dynamic_cast<ActingState *>(kv.second)) {
      acting->setOperationThreadPool(&operation_pool_);
    }
  }
}

void StateMachine::requestActingStatesStop() {
  if (!gen) {
    return;
  }
  for (auto & kv : gen->states) {
    if (auto * acting = dynamic_cast<ActingState *>(kv.second)) {
      acting->requestOperationStop();
    }
  }
}

bool StateMachine::deactivate() {
  printf("Deactivating state machine\n");
  if (!sm_internal_.isRunning()) {
    requestActingStatesStop();
    drainActingStates();
    return true;
  }

  // See requestActingStatesStop(): this is the promptness half. An interruptible operation that
  // observes its token returns now rather than at the end of its own duration, so the drain below
  // is short instead of as long as the user's lambda.
  requestActingStatesStop();

  // Synchronously wait for QStateMachine::stop() to take effect.  stop() is
  // asynchronous (it posts a stop event onto the SM's event loop) and does
  // not call onExit() on currently active states, so we cannot rely on Qt
  // to drain ActingState futures for us.
  std::mutex stopped_mutex;
  std::condition_variable stopped_cv;
  bool stopped_flag = false;
  auto conn = QObject::connect(
    &sm_internal_, &QStateMachine::stopped,
    &sm_internal_, [&]() {
      {
        std::lock_guard<std::mutex> lk(stopped_mutex);
        stopped_flag = true;
      }
      stopped_cv.notify_all();
    },
    Qt::DirectConnection);
  sm_internal_.stop();
  {
    std::unique_lock<std::mutex> lk(stopped_mutex);
    stopped_cv.wait_for(lk, std::chrono::seconds(2), [&] { return stopped_flag; });
  }
  QObject::disconnect(conn);

  // Observing stopped() is NOT the same as the Qt thread being finished with this machine.
  // The signal is emitted from inside Qt's own stop handling, and the connection above is
  // Direct, so the lambda runs on the Qt thread partway through that handling -- while the
  // main thread is woken and free to run on. What follows here is a drain and then, for a
  // caller who deactivates and drops the object, its destruction: the state graph gets freed
  // underneath a Qt thread still working through the rest of its stop.
  //
  // This is the barrier that was missing. A blocking queued call cannot begin until the event
  // handler that emitted stopped() has returned, so once it comes back, the Qt thread has
  // finished with this machine and it is safe to take it apart. Skipped when this IS the Qt
  // thread, where such a call would deadlock and no barrier is needed anyway.
  if (QThread::currentThread() != sm_internal_.thread()) {
    QMetaObject::invokeMethod(&sm_internal_, [] {}, Qt::BlockingQueuedConnection);
  }

  // Now that the SM has stopped, no new ActingState worker can be spawned. Ask again, because a
  // transition between the request above and the stop taking effect would have entered a state
  // whose onEntry() installed a fresh stop source that the earlier request never saw. Then drain
  // whichever worker was started by the last active ActingState before stop() arrived -- this
  // replaces the previous fixed-time sleep and supports user-defined lambdas of any duration.
  requestActingStatesStop();
  drainActingStates();
  return stopped_flag;
}

StateMachine::~StateMachine() {
  // Even if the user already called deactivate(), make
  // sure no QtConcurrent worker is still touching us before QObject
  // members are torn down.
  if (sm_internal_.isRunning()) {
    deactivate();
  } else {
    requestActingStatesStop();
    drainActingStates();
  }
  // Final safety net: even after we drained every ActingState we own, the pool may still hold a
  // worker for an operation that completed but whose runner hasn't been recycled yet. Wait
  // indefinitely — drainActingStates() guarantees user lambdas have returned, so this only waits
  // for thread-pool bookkeeping.
  //
  // Our own pool, not QThreadPool::globalInstance(): waiting on the global one also waited for
  // every other QtConcurrent user in the address space, including other StateMachine instances,
  // which made a bounded wait here depend on who else happened to be in the process.
  operation_pool_.waitForDone(-1);
}


std::shared_ptr<StateMachine> StateMachine::singleCycleSM(int delay_ms) {
  auto SS = std::make_shared<SingleCycle>();
  SS->init(delay_ms);
  return SS;
}

std::shared_ptr<StateMachine> StateMachine::continuousCycleSM(int delay_ms) {
  auto CS = std::make_shared<ContinuousCycle>();
  CS->init(delay_ms);
  return CS;
}

/*
 * NOTES:
 * Create factory methods that take std::bind as an argument for
 * a custom call back in the "onExit" method.
 *
 * StateMachine will consist of several public SLOTS for each
 * PackML command.  The implementations will post events to the SM
 * when called.
 *
 * Specializations of StateMachine (like ROS StateMachine) will use
 * state entered events to trigger status publishing via SLOTS
 *
 * Mode handling will be achieved using a hiearchy of state machines
 * that reference/utilize many of the same transitions/states (maybe)
 */

StateMachine::StateMachine() : gen(std::make_shared<StatesGenerator>()) {
  // A floor, not a calculation. One thread per acting state would be the true bound if a state
  // could only ever have one operation outstanding, and it cannot -- a re-entered state can start
  // a second while the first is still returning. What the floor rules out is the degenerate case:
  // QThreadPool defaults to idealThreadCount(), which is 1 on a single-core container, where one
  // lingering worker would make the next state's operation wait for it.
  static constexpr int kMinOperationThreads = 4;
  operation_pool_.setMaxThreadCount(std::max(kMinOperationThreads, QThread::idealThreadCount()));

  printf("State machine constructor\n");
  // Hook the inner Qt state machine's ErrorEvent observer back to *this* so
  // applications can call getLastErrorCode() after an acting state's bound
  // function returns a non-zero error code.
  sm_internal_.on_error_code = [this](int code) { last_error_code_ = code; };
  // printf("Constructiong super states\n");
  abortable_ = PackmlSuperState::Abortable();
  stoppable_ = PackmlSuperState::Stoppable(abortable_);

  // printf("Constructing acting/wait states\n");
  // TODO: These are unused, replaced by statesgenerator
  held_ = WaitState::Held(stoppable_);
  idle_ = WaitState::Idle(stoppable_);
  complete_ = WaitState::Complete(stoppable_);
  suspended_ = WaitState::Suspended(stoppable_);
  stopped_ = WaitState::Stopped(abortable_);
  aborted_ = WaitState::Aborted();
  unholding_ = ActingState::Unholding(stoppable_, 200);
  holding_ = ActingState::Holding(stoppable_, 200);
  starting_ = ActingState::Starting(stoppable_);
  completing_ = ActingState::Completing(stoppable_);
  resetting_ = ActingState::Resetting(stoppable_);
  unsuspending_ = ActingState::Unsuspending(stoppable_);
  suspending_ = ActingState::Suspending(stoppable_);
  stopping_ = ActingState::Stopping(abortable_);
  clearing_ = ActingState::Clearing(abortable_);
  aborting_ = ActingState::Aborting();
  execute_ = ActingState::Execute(stoppable_);

  connect(abortable_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stoppable_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)

  connect(unholding_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(held_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(holding_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(idle_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(starting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(completing_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(complete_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(resetting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(unsuspending_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(suspended_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(suspending_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stopped_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stopping_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(clearing_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(aborted_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(aborting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(execute_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)

  printf("Adding states to state machine\n");
  sm_internal_.addState(abortable_);
  sm_internal_.addState(aborted_);
  sm_internal_.addState(aborting_);
}

// Callback from QT state machine when state changed
void StateMachine::setState(State value, QString name) {
  std::string nameUtf = name.toStdString();
  PACKML_INFO_STREAM("packml_sm", "State changed(event) to: " << nameUtf << "(" << value << ")");
  state_value_.store(value, std::memory_order_release);
  state_name_ = name;
  on_state_changed(value, name);
  // emit stateChanged(value, name);
}

bool StateMachine::setExecute(std::function<int()> execute_method) {
  printf("Initializing state machine with EXECUTE function pointer\n");
  // Must be set on the state in the LIVE graph (built by `gen->generate_all_packml_states`),
  // not on the legacy `execute_` member ActingState the constructor creates: that one is never
  // wired into the running graph, so binding to it silently drops the caller's callback and
  // leaves the internal default running instead.
  if (gen) {
    auto it = gen->states.find(to_string(State::EXECUTE));
    if (it != gen->states.end()) {
      if (auto * a = dynamic_cast<ActingState *>(it->second)) {
        a->setOperationMethod(execute_method);
      }
    }
  }
  return execute_->setOperationMethod(execute_method);
}

bool StateMachine::setResetting(std::function<int()> resetting_method) {
  printf("Initializing state machine with RESETTING function pointer\n");
  // BUGFIX: same as setExecute() above -- target the live RESETTING state in
  // gen->states, not the orphaned legacy `resetting_` member.
  if (gen) {
    auto it = gen->states.find(to_string(State::RESETTING));
    if (it != gen->states.end()) {
      if (auto * a = dynamic_cast<ActingState *>(it->second)) {
        a->setOperationMethod(resetting_method);
      }
    }
  }
  return resetting_->setOperationMethod(resetting_method);
}

bool StateMachine::setStateOperation(State state, std::function<int()> method) {
  if (!gen) return false;
  auto it = gen->states.find(to_string(state));
  if (it == gen->states.end()) return false;
  auto * acting = dynamic_cast<ActingState *>(it->second);
  if (!acting) return false;
  return acting->setOperationMethod(method);
}

bool StateMachine::setInterruptibleStateOperation(
  State state, std::function<int(std::stop_token)> method)
{
  if (!gen) return false;
  auto it = gen->states.find(to_string(state));
  if (it == gen->states.end()) return false;
  auto * acting = dynamic_cast<ActingState *>(it->second);
  if (!acting) return false;
  return acting->setInterruptibleOperationMethod(method);
}

double StateMachine::getStateCumulativeTime(State state) const {
  if (!gen) return 0.0;
  auto it = gen->states.find(to_string(state));
  if (it == gen->states.end()) return 0.0;
  return it->second->cumulativeTime().count();
}


// Change state triggers asynchronous switching of state machine. If successful it will call callback on_state_changed
std::expected<bool, std::string> StateMachine::changeState(TransitionCmd command)
{
  bool command_rtn = false;
  bool command_valid = true;
  std::string error_message;

  std::stringstream ss;
  PACKML_INFO_STREAM("packml_sm", "Evaluating transition request command: " << command);

  switch (command) {
    case TransitionCmd::ABORT:
      command_rtn = abort();
      break;
    case  TransitionCmd::STOP:
      command_rtn = stop();
      break;
    case TransitionCmd::CLEAR:
      command_rtn = clear();
      break;
    case TransitionCmd::HOLD:
      command_rtn = hold();
      break;
    case TransitionCmd::RESET:
      command_rtn = reset();
      break;
    case TransitionCmd::START:
      command_rtn = start();
      break;
    case TransitionCmd::SUSPEND:
      command_rtn = suspend();
      break;
    case TransitionCmd::UNHOLD:
      command_rtn = unhold();
      break;
    case TransitionCmd::UNSUSPEND:
      command_rtn = unsuspend();
      break;
    default:
      command_valid = false;
      break;
  }

  if (!command_valid) {
    error_message = "Invalid transition request command: " + to_string(command);
    PACKML_ERROR_STREAM("packml_sm", error_message);
    return std::unexpected<std::string>(error_message);
  }

  if (!command_rtn) {
    error_message =  "Transition command failed: " + to_string(command);
    PACKML_WARN_STREAM("packml_sm", error_message);
    return std::unexpected<std::string>(error_message);
  }

  return command_valid;
}


std::expected<bool, std::string> StateMachine::changeMode(ModeType mode)
{
  AvailableStates avail{
    {State::ABORTING, true},
    {State::ABORTED, true},
    {State::CLEARING, true},
    {State::STOPPING, true},
    {State::STOPPED, true},
    {State::RESETTING, true},
    {State::IDLE, true},
    {State::STARTING, true},
    {State::EXECUTE, true},
    {State::HOLDING, true},
    {State::HELD, true},
    {State::UNHOLDING, true},
    {State::SUSPENDING, true},
    {State::SUSPENDED, true},
    {State::UNSUSPENDING, true},
    {State::COMPLETING, true},
    {State::COMPLETE, true},
  };
  return changeMode(mode, avail);
}

std::expected<bool, std::string> StateMachine::changeMode(ModeType mode, AvailableStates avail)
{
  // Applied on every path into a mode, not only when a mask is parsed from a file, so no caller
  // can install a mask that disables a state the machine is required to keep reachable.
  for (const auto & state_name : enforce_mandatory_states(avail)) {
    PACKML_WARN_STREAM("packml_sm", "Mode " << mode << " disables " << state_name <<
      ", which is mandatory -- restoring it");
  }

  StatesGenerator::Mode mode1 = StatesGenerator::Mode(mode, to_string(mode), avail);

  // mode_switcher() holds mode_mask_mutex() across the whole application, so a command being
  // evaluated on the state machine's own thread sees either the old mask or the new one, never a
  // mix, and never a property being written underneath it. See mode_mask_mutex(). The mode value
  // travels inside mode1 and is stored by that same locked pass, which is what getCurrentMode()
  // reads back.
  auto return_val = gen->mode_switcher(shared_from_this(), mode1);

  if (return_val.has_value()) {
    on_mode_changed(mode);
  }
  return return_val;
}

ModeType StateMachine::getCurrentMode() const
{
  std::lock_guard<std::mutex> lk(mode_mask_mutex());
  return gen->currentMode.value;
}

void StateMachine::set_manual_mode(ModeType mode)
{
  // Synchronize with mode_switcher() so each admission decision reads one
  // complete manual-mode value.
  std::lock_guard<std::mutex> lk(mode_mask_mutex());
  gen->manual_mode = mode;
}

AvailableStates StateMachine::getAvailableStates() const
{
  std::lock_guard<std::mutex> lk(mode_mask_mutex());
  return gen->currentMode.available_states;
}

namespace {
// How often a blocked command re-reads the machine's own liveness. Not a deadline: an answered
// command wakes the wait immediately whatever this is set to, and an unanswerable one is ended by
// the machine having stopped, not by time passing.
constexpr auto kLivenessCheckInterval = std::chrono::milliseconds(50);
}  // namespace

bool StateMachine::postCommand(TransitionCmd command) {
  // Ordered so the first guard that matches gives the accurate diagnosis. Liveness comes first
  // because before activate() the machine is BOTH not running and still sitting on the thread
  // that constructed it -- which is usually the thread now issuing the command, so an
  // event-loop-thread check placed first would blame the caller's thread for a machine that had
  // simply never been started.
  //
  // QStateMachine::postEvent() discards -- and leaks -- an event posted to a machine that is not
  // running, saying so only as a qWarning on stderr; the promise inside that event is then never
  // fulfilled and never destroyed, so the caller waits for the life of the process. It does admit
  // events while the machine is Starting, but start-up empties the queue before entering the
  // initial state, so a command admitted there is dropped just the same. Refusing during Starting
  // is the accurate answer, not a conservative one.
  if (!sm_internal_.isRunning()) {
    PACKML_ERROR_STREAM("packml_sm", "Refusing command " << command << ": the state machine is "
      "not running. Either activate() was never called or did not succeed, or deactivate() has "
      "already stopped it");
    return false;
  }

  // Qt answers a command from inside its own event loop, so a caller that IS that loop would be
  // waiting for a reply only it can send. Both shapes deadlock permanently: posting from an idle
  // loop schedules the processing step BEHIND the frame about to block, and posting from inside a
  // macrostep appends to the very queue that frame was draining.
  if (QThread::currentThread() == sm_internal_.thread()) {
    PACKML_ERROR_STREAM("packml_sm", "Refusing command " << command << " issued from the state "
      "machine's own event-loop thread: only that loop can answer it, and this call would be what "
      "stops it running. Issue commands from another thread");
    return false;
  }

  // An acting state's exit path joins the worker running its bound operation, so a bound operation
  // that waits on the machine and a machine that waits on the bound operation hold each other
  // permanently. Unlike the two guards above, this one cannot be spotted from the machine's own
  // state: it is running, and a thread-pool worker is not the loop thread.
  if (ActingState::callerIsBoundOperation()) {
    PACKML_ERROR_STREAM("packml_sm", "Refusing command " << command << " issued from inside an "
      "acting state's bound operation: the state machine waits for that operation to return "
      "before it processes anything, so this call would deadlock both. Return a non-zero error "
      "code instead to escalate to the state's declared failure target");
    return false;
  }

  auto p = std::make_shared<std::promise<bool>>();
  auto f = p->get_future();
  sm_internal_.postEvent(new CmdEvent(command, std::move(p)));

  // Deliberately no deadline. Transition selection answers a command as soon as the event loop
  // reaches it; the only thing that delays that is a bound operation still running in the state
  // being left, which onExit() waits out for as long as the integrator's lambda takes -- the same
  // bound deactivate() and ~StateMachine() already accept. A wall-clock false would also be a lie:
  // the command stays queued and still takes effect, and Qt offers no way to withdraw a posted
  // event. To time out honestly you must first be able to cancel.
  //
  // What does end the wait is the machine stopping with the command still queued. Once isRunning()
  // is false the processing loop has already exited, so no answer can be in flight.
  while (f.wait_for(kLivenessCheckInterval) != std::future_status::ready) {
    if (sm_internal_.isRunning()) {
      continue;
    }
    if (f.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      break;
    }
    PACKML_ERROR_STREAM("packml_sm", "Command " << command << " was still queued when the state "
      "machine stopped; it will never be evaluated");
    return false;
  }

  try {
    return f.get();
  } catch (const std::future_error & e) {
    // Unreachable while every path that destroys a CmdEvent answers it first, and kept anyway.
    // Deleting that destructor re-arms a std::future_error thrown out of a ROS service callback,
    // which takes the manager node down during shutdown -- the one moment nobody is watching it.
    PACKML_ERROR_STREAM("packml_sm", "Command " << command << " was destroyed before it was "
      "answered: " << e.what());
    return false;
  }
}

bool StateMachine::_start()     { return postCommand(TransitionCmd::START); }
bool StateMachine::_clear()     { return postCommand(TransitionCmd::CLEAR); }
bool StateMachine::_reset()     { return postCommand(TransitionCmd::RESET); }
bool StateMachine::_hold()      { return postCommand(TransitionCmd::HOLD); }
bool StateMachine::_unhold()    { return postCommand(TransitionCmd::UNHOLD); }
bool StateMachine::_suspend()   { return postCommand(TransitionCmd::SUSPEND); }
bool StateMachine::_unsuspend() { return postCommand(TransitionCmd::UNSUSPEND); }
bool StateMachine::_stop()      { return postCommand(TransitionCmd::STOP); }
bool StateMachine::_abort()     { return postCommand(TransitionCmd::ABORT); }

ContinuousCycle::ContinuousCycle() {
  printf("Forming CONTINUOUS CYCLE state machine (states + transitions)\n");
  // // Naming <from state>_<to state>
  // CmdTransition::abort(*abortable_, *aborting_);
  // ErrorTransition *abortable_aborting_on_error =
  //     new ErrorTransition(*abortable_, *aborting_);

  // StateCompleteTransition *aborting_aborted =
  //     new StateCompleteTransition(*aborting_, *aborted_);

  // CmdTransition::clear(*aborted_, *clearing_);
  // StateCompleteTransition *clearing_stopped_ =
  //     new StateCompleteTransition(*clearing_, *stopped_);

  // CmdTransition::stop(*stoppable_, *stopping_);
  // StateCompleteTransition *stopping_stopped =
  //     new StateCompleteTransition(*stopping_, *stopped_);

  // CmdTransition::reset(*stopped_, *resetting_);
  // StateCompleteTransition *unholding_execute_ =
  //     new StateCompleteTransition(*unholding_, *execute_);

  // CmdTransition::unhold(*held_, *unholding_);
  // StateCompleteTransition *holding_held_ =
  //     new StateCompleteTransition(*holding_, *held_);

  // CmdTransition::start(*idle_, *starting_);
  // StateCompleteTransition *starting_execute_ =
  //     new StateCompleteTransition(*starting_, *execute_);

  // CmdTransition::hold(*execute_, *holding_);
  // StateCompleteTransition *execute_execute_ =
  //     new StateCompleteTransition(*execute_, *execute_);

  // StateCompleteTransition *completing_complete =
  //     new StateCompleteTransition(*completing_, *complete_);

  // CmdTransition::reset(*complete_, *resetting_);
  // StateCompleteTransition *resetting_idle_ =
  //     new StateCompleteTransition(*resetting_, *idle_);

  // CmdTransition::suspend(*execute_, *suspending_);
  // StateCompleteTransition *suspending_suspended_ =
  //     new StateCompleteTransition(*suspending_, *suspended_);

  // CmdTransition::unsuspend(*suspended_, *unsuspending_);
  // StateCompleteTransition *unsuspending_execute_ =
  //     new StateCompleteTransition(*unsuspending_, *execute_);

  // abortable_->setInitialState(clearing_);
  // stoppable_->setInitialState(resetting_);
  // sm_internal_.setInitialState(aborted_);
}
void ContinuousCycle::init(int delay_ms){
  gen->generate_all_packml_states(shared_from_this(), delay_ms);
  bindOperationThreadPool();
  // Add parent states to state machine
  // All other states are added 'automatically' because they are under the superstate "abortable"
  sm_internal_.addState(gen->states[to_string(SuperState::ABORTABLE)]);
  sm_internal_.addState(gen->states[to_string(State::ABORTED)]);
  sm_internal_.addState(gen->states[to_string(State::ABORTING)]);

  // PackML mandates power-on into STOPPED (inside abortable group).
  sm_internal_.setInitialState(gen->states[to_string(SuperState::ABORTABLE)]);

  // Test to see if we can adjust the state machines transitions
  auto list = gen->states[to_string(State::EXECUTE)]->transitions();
  for (const auto& item : list)
  {
    if (item->targetState() == gen->states[to_string(State::COMPLETING)])
    {
      PACKML_INFO_STREAM("packml_sm", "Found transition!");
      gen->states[to_string(State::EXECUTE)]->removeTransition(item);
      PACKML_INFO_STREAM("packml_sm", "Removed transition!");
      auto trans = gen->generate_transition(gen->states[to_string(State::EXECUTE)], StatesGenerator::TransitionType::STATE_COMPLETED);
      gen->states[to_string(State::EXECUTE)]->addTransition(trans);
      PACKML_INFO_STREAM("packml_sm", "Added transition to self!");
    }
  }

  // ContinuousCycle EXECUTE holds when nothing is bound to it -- it does NOT complete on a timer.
  //
  // Binding a placeholder like "sleep(delay_ms); return 0" so the demo does something completes a
  // production cycle several times a second on a machine that has produced nothing, and because
  // the transition above sends every one of those completions straight back into EXECUTE, a
  // manager sitting in its normal production state then fans a full state-transition goal out to
  // every equipment module at that rate forever. The duration is not the problem: a real bound
  // execute of the same length does the same thing. Completing EXECUTE is the end of production --
  // the standard leaves it when the product
  // counter reaches its limit or on an explicit Complete command -- so a machine with nothing
  // bound has nothing to complete, and should sit still and say so (see activate(), which names
  // the omission) rather than fabricate cycles.
  ((ActingState*) gen->states[to_string(State::EXECUTE)])->holdWhenUnbound();

  printf("State machine formed\n");
}

SingleCycle::SingleCycle() {
  printf("Forming SINGLE CYCLE state machine (states + transitions)\n");
  // Naming <from state>_<to state>
  // auto aborttrans = CmdTransition::abort(*abortable_, *aborting_);
  // ErrorTransition *abortable_aborting_on_error =
  //     new ErrorTransition(*abortable_, *aborting_);

  // StateCompleteTransition *aborting_aborted =
  //     new StateCompleteTransition(*aborting_, *aborted_);

  // CmdTransition::clear(*aborted_, *clearing_);
  // StateCompleteTransition *clearing_stopped_ =
  //     new StateCompleteTransition(*clearing_, *stopped_);

  // CmdTransition::stop(*stoppable_, *stopping_);
  // StateCompleteTransition *stopping_stopped =
  //     new StateCompleteTransition(*stopping_, *stopped_);

  // CmdTransition::reset(*stopped_, *resetting_);
  // StateCompleteTransition *unholding_execute_ =
  //     new StateCompleteTransition(*unholding_, *execute_);

  // CmdTransition::unhold(*held_, *unholding_);
  // StateCompleteTransition *holding_held_ =
  //     new StateCompleteTransition(*holding_, *held_);

  // CmdTransition::start(*idle_, *starting_);
  // StateCompleteTransition *starting_execute_ =
  //     new StateCompleteTransition(*starting_, *execute_);

  // CmdTransition::hold(*execute_, *holding_);
  // StateCompleteTransition *execute_completing_ =
  //     new StateCompleteTransition(*execute_, *completing_);

  // StateCompleteTransition *completing_complete =
  //     new StateCompleteTransition(*completing_, *complete_);

  // CmdTransition::reset(*complete_, *resetting_);
  // StateCompleteTransition *resetting_idle_ =
  //     new StateCompleteTransition(*resetting_, *idle_);

  // CmdTransition::suspend(*execute_, *suspending_);
  // StateCompleteTransition *suspending_suspended_ =
  //     new StateCompleteTransition(*suspending_, *suspended_);

  // CmdTransition::unsuspend(*suspended_, *unsuspending_);
  // StateCompleteTransition *unsuspending_execute_ =
  //     new StateCompleteTransition(*unsuspending_, *execute_);

  // abortable_->setInitialState(clearing_);
  // stoppable_->setInitialState(resetting_);

  }
void SingleCycle::init(int delay_ms){
  gen->generate_all_packml_states(shared_from_this(), delay_ms);
  bindOperationThreadPool();

  // Add parent states to state machine
  // All other states are added 'automatically' because they are under the superstate "abortable"
  sm_internal_.addState(gen->states[to_string(SuperState::ABORTABLE)]);
  sm_internal_.addState(gen->states[to_string(State::ABORTED)]);
  sm_internal_.addState(gen->states[to_string(State::ABORTING)]);

  // PackML mandates power-on into STOPPED (inside abortable group).
  sm_internal_.setInitialState(gen->states[to_string(SuperState::ABORTABLE)]);

  // // Test to see if we can adjust the state machines transitions
  // auto list = gen.states[to_string(State::EXECUTE)]->transitions();
  // for (const auto& item : list)
  // {
  //     if (item->targetState() == gen.states[to_string(State::COMPLETING)])
  //     {
  //       std::cout << "Foind transition!" << std::endl;
  //       gen.states[to_string(State::EXECUTE)]->removeTransition(item);
  //       std::cout << "Removed transition!" << std::endl;
  //       auto trans = gen.generate_transition(gen.states[to_string(State::EXECUTE)], StatesGenerator::TransitionType::STATE_COMPLETED);
  //       gen.states[to_string(State::EXECUTE)]->addTransition(trans);
  //       std::cout << "Added transition to self!" << std::endl;
  //     }
  // }

  // SingleCycle default EXECUTE: uses the same delay; users override via setExecute().
  auto exec_delay = std::max(delay_ms, 1);
  ((ActingState*) gen->states[to_string(State::EXECUTE)])->setOperationMethod(
    [exec_delay]() -> int {
      std::this_thread::sleep_for(std::chrono::milliseconds(exec_delay));
      return 0;
    });

  printf("End of single cycle setup\n");

}

} // namespace packml_sm
