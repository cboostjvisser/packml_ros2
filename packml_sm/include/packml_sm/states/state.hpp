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

#include <atomic>
#include <cstdint>

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

  /// One bit of the mode mask: whether a COMMAND may transition into this state in the currently
  /// selected mode. Read by CmdTransition, written by StatesGenerator::mode_switcher().
  ///
  /// A plain member, and deliberately not a Qt DYNAMIC PROPERTY. setProperty()
  /// on a dynamic property does not merely store a value: it dispatches a
  /// QDynamicPropertyChangeEvent through QCoreApplication::sendEvent -- synchronously, on the
  /// CALLING thread. The mask is written by whichever thread calls changeMode, while these objects
  /// belong to the state machine's own thread, so that would drag Qt's notify machinery (including
  /// a non-atomic update of the RECEIVER's QThreadData) into a cross-thread write that no lock on
  /// this side can serialize. A bool has none of that machinery, so the lock below is sufficient
  /// rather than merely necessary.
  ///
  /// Caller must hold packml_sm::mode_mask_mutex(). Not taken in here on purpose: a mode is applied
  /// as a SET, and locking per access would still let a command see a half-applied mode.
  bool availableInMode() const {return available_in_mode_;}
  void setAvailableInMode(bool available) {available_in_mode_ = available;}
  const std::string name() const {return name_.toStdString();}
  std::chrono::duration<double> cumulativeTime() const { return cummulative_time_; }

  // Which visit to this state we are on. Bumped on entry AND on exit, so it identifies one
  // specific activation rather than the state in general: odd while the state is active, even
  // once it has been left, and 0 before it has ever been entered. An event stamped with the
  // value read at entry (see ActivationToken) therefore stops matching the moment this state is
  // left, and does not start matching again when it is next entered.
  //
  // Atomic because it is written on the Qt thread (onEntry/onExit) and read both there
  // (transition eventTest) and on the QtConcurrent worker that runs an acting state's bound
  // operation.
  //
  // One known hole: QStateMachine::stop() does not call onExit, so whichever state was active at
  // stop time keeps an odd count. Harmless -- deactivate() joins every worker before the machine
  // goes away, and Qt discards events posted to a stopped machine.
  uint64_t activation() const {return activation_.load(std::memory_order_acquire);}

  virtual ~PackmlState() {}

signals:
  void stateEntered(State value, QString name);

protected:
  // Defaults to available so a freshly built machine is usable before any changeMode() -- see
  // availableInMode(). Narrowing it is what a mode mask does.
  bool available_in_mode_{true};
  State state_;
  QString name_;

  std::chrono::time_point<std::chrono::system_clock> enter_time_;
  std::chrono::time_point<std::chrono::system_clock> exit_time_;
  std::chrono::duration<double> cummulative_time_;
  std::atomic<uint64_t> activation_{0};

  virtual void onEntry(QEvent * e);
  virtual void operation() {}
  virtual void onExit(QEvent * e);
};

}  // namespace packml_sm
