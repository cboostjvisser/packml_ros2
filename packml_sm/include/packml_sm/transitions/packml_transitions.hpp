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

#include <memory>

#include "QEvent"
#include "QAbstractTransition"
#include "packml_sm/events/activation_token.hpp"
#include "packml_sm/logging.hpp"
#include "packml_sm/states/state.hpp"
#include "packml_sm/common.hpp"
// #include "packml_sm/states/state.hpp"
// #include "packml_sm/states_generator.hpp"

namespace packml_sm
{

/**
* @brief Class to define transitions that are not valid
*/
class PackmlTransition : public QAbstractTransition
{
public:
  /**
  * @brief Constructor of the class
  */
  PackmlTransition() {}

//   std::shared_ptr<std::vector<StatesGenerator::Mode>> modes;
//   std::string current_mode = "";

  /**
  * @brief Destructor of the class
  */
  virtual ~PackmlTransition() {}

protected:
  /**
  * @brief Is this transition's target state permitted by the active mode mask?
  *
  * DELIBERATELY NOT A VIRTUAL eventTest() OVERRIDE, and deliberately named for what it
  * actually decides. This is COMMAND-ADMISSION POLICY: a mode declares which states an
  * operator may command the machine INTO. Only CmdTransition may consult it.
  *
  * On the base class's eventTest() it would also be consulted by StateCompleteTransition and
  * ErrorTransition, letting a mode mask decide whether the machine may FINISH a state it is
  * already executing, or escalate an error out of one. That is not a policy question: by the
  * time an acting state is running the decision to be in it has already been made, and
  * refusing its completion leaves it with no exit. Masking a single state (e.g. Maintenance
  * with EXECUTE unavailable) then makes START succeed and strand the machine in STARTING
  * indefinitely -- no further status, no alarm, no timeout, no log.
  *
  * Keeping the check off the shared base is the structural half: a future transition type
  * cannot silently inherit command policy it has no business applying.
  *
  * @param e - triggering event; ignored (in the Qt sense) when the target is unavailable,
  *            so the command is reported as rejected rather than silently swallowed.
  */
  bool target_available_in_current_mode(QEvent * e)
  {
    auto * statetarget = qobject_cast<PackmlState *>(targetState());
    if (nullptr == statetarget) {
      // Refuse rather than assume. Every state in this graph is a PackmlState, so this is
      // unreachable today; if that ever stops being true, a command must not be admitted by a
      // target whose mode mask nobody can read.
      PACKML_DEBUG("packml_sm", "Transition target is not a PackmlState; refusing the command");
      e->ignore();
      return false;
    }
    bool available = false;
    {
      // See mode_mask_mutex(): the mask is written from whichever thread calls changeMode.
      std::lock_guard<std::mutex> lk(mode_mask_mutex());
      available = statetarget->availableInMode();
    }
    if (available) {
      PACKML_DEBUG("packml_sm", "Transition is available!");
      return true;
    }
    PACKML_DEBUG("packml_sm", "Transition to next state: is not available in this mode!");
    e->ignore();
    return false;
  };

  /**
  * @brief Was the event minted by this transition's own source state, or by a state inside it?
  *
  * A superstate's transition legitimately serves its descendants (`abortable`'s ERROR edge is
  * how a fault in any state it contains escalates), so containment -- not equality -- is the
  * test. What it rules out is the reverse: an event minted by some OTHER state being consumed
  * here, which is how one state's completion or fault came to be credited to whichever state
  * Qt happened to have active by the time it processed the event.
  */
  /// This transition's source state, for log messages only.
  std::string source_name() const
  {
    const auto * source = dynamic_cast<const PackmlState *>(sourceState());
    return (nullptr != source) ? source->name() : std::string("<unnamed source>");
  }

  bool originates_at_or_within_source(const ActivationToken & token) const
  {
    for (const QAbstractState * candidate = token.origin;
         nullptr != candidate;
         candidate = candidate->parentState())
    {
      if (candidate == sourceState()) {
        return true;
      }
    }
    return false;
  }

  /**
  * @brief Does this completion event report the completion of THIS transition's source state, on
  *        the visit it is currently on?
  *
  * Both halves are needed and they catch different things.
  *
  * Attribution catches the wrong state: a bound operation runs on a worker thread and its event
  * is queued, so the machine can have moved on before Qt processes it. Its successor's own
  * completion edge is then armed and, on a type-only test, fires -- skipping that successor's
  * work entirely. ABORTING is the case that matters: it is where every fault is funnelled, so a
  * stale completion from the state the abort interrupted would satisfy ABORTING's own edge and
  * carry the machine straight to ABORTED without the abort ever running.
  *
  * The activation comparison catches the same state, a visit later: a state can be left and
  * re-entered while its own earlier completion is still queued (EXECUTE under ContinuousCycle
  * self-loops, and both EXECUTE and RESETTING are re-enterable by command), and there attribution
  * passes because the origin IS the source. Only the visit number distinguishes the ghost.
  */
  bool completion_matches_this_activation(const ActivationToken & token) const
  {
    if (nullptr == token.origin) {
      PACKML_WARN("packml_sm", "Refusing an unattributed state-completion event: a completion has "
        "to say which state finished, or it can only be credited to a guess");
      return false;
    }
    const auto current = token.origin->activation();
    if (token.activation != current) {
      PACKML_WARN_STREAM("packml_sm", "Discarding a stale completion from " <<
        token.origin->name() << ": stamped on visit " << token.activation <<
        ", that state is now on visit " << current);
      return false;
    }
    if (!originates_at_or_within_source(token)) {
      PACKML_WARN_STREAM("packml_sm", "Not crediting " << token.origin->name() <<
        "'s completion to " << source_name());
      return false;
    }
    return true;
  }

  /**
  * @brief May this transition escalate the fault carried by @p token?
  *
  * Deliberately weaker than the completion check, in the same way and for the same reason that
  * ErrorTransition ignores the mode mask: a fault must always be able to escalate. So there is
  * no activation comparison here -- a fault raised by a state the machine has since left is
  * still a real fault, and dropping it for being late would trade a misattributed abort for a
  * missed one. It escalates through the nearest enclosing superstate's edge instead.
  *
  * The one thing refused is attribution to a bystander: without this, a fault raised by EXECUTE
  * while a HOLD was already queued would abort from HOLDING as though HOLDING had failed.
  *
  * An unattributed fault (no origin) escalates. Nothing in the tree mints one -- an acting
  * state's operation is the only producer of error events -- but if something ever does, a fault
  * of unknown provenance is the last thing that should be swallowed.
  */
  bool fault_may_be_attributed_here(const ActivationToken & token) const
  {
    if (nullptr == token.origin) {
      return true;
    }
    if (originates_at_or_within_source(token)) {
      return true;
    }
    PACKML_WARN_STREAM("packml_sm", "Not attributing " << token.origin->name() <<
      "'s fault to another state; leaving it to an enclosing superstate to escalate");
    return false;
  }

  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {PACKML_DEBUG("packml_sm", "Transition triggered, event pointer: %p", static_cast<void*>(e));}
};

}  // namespace packml_sm