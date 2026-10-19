// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---

#include "packml_ros/transition_guard.hpp"

namespace packml_ros {

/// All of TransitionGuard's state, behind one mutex, in a block that outstanding InFlightTokens
/// co-own so a token may safely outlive the guard.
struct InFlightToken::Core
{
  std::mutex guard_mutex;
  packml_sm::State current_state{packml_sm::State::UNDEFINED};
  packml_sm::ModeType current_mode{0};
  packml_sm::State switching_state{packml_sm::State::UNDEFINED};
  packml_sm::ModeType switching_mode{0};
  bool waiting_for_mode{false};

  // Identity of the state arm currently held, or 0 for none. An id rather than a bool so a
  // release can prove it owns the arm it is giving back. Monotonic, so a superseded arm's id is
  // never reissued and its token's release is harmlessly ignored.
  uint64_t state_arm_id{0};
  uint64_t next_arm_id{1};

  // Where admit_state() parks its decision until claim_state() takes it. Single-slot because
  // exactly one state-transition goal executes at a time per node -- the manager's wait for the
  // previous one resolves before it sends the next -- the same invariant CompletionTracker and
  // the deferred-completion fields already rely on.
  struct Admission
  {
    packml_sm::State target{packml_sm::State::UNDEFINED};
    TransitionResult result;
    uint64_t arm_id{0};
  };
  std::optional<Admission> pending;
};

void InFlightToken::release() noexcept
{
  if (nullptr == core_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(core_->guard_mutex);
    // Only the arm currently held can be given back. A token whose arm was superseded, or which
    // is releasing a second time, names an id that no longer matches and does nothing -- which is
    // what lets a detached deferred-completion thread release late without stealing the arm of
    // whatever started while it was waiting.
    if (0 != arm_id_ && core_->state_arm_id == arm_id_) {
      core_->state_arm_id = 0;
      core_->switching_state = packml_sm::State::UNDEFINED;
    }
  }
  core_.reset();
  arm_id_ = 0;
}

TransitionGuard::TransitionGuard()
: core_(std::make_shared<InFlightToken::Core>()) {}

TransitionGuard::~TransitionGuard() = default;

packml_sm::State TransitionGuard::current_state() const
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);
  return core_->current_state;
}

packml_sm::ModeType TransitionGuard::current_mode() const
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);
  return core_->current_mode;
}

bool TransitionGuard::is_switching_state() const
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);
  return 0 != core_->state_arm_id;
}

bool TransitionGuard::is_switching_mode() const
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);
  return core_->waiting_for_mode;
}

TransitionResult TransitionGuard::decide_state(packml_sm::State target)
{
  TransitionResult result;

  // Already in the requested state -- no-op success. Decided here, at admission, precisely
  // because core_->current_state is still this goal's pre-echo view; see admit_state().
  if (target == core_->current_state) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // A transition is already in flight: do NOT override it with another. Reading core_->switching_state
  // distinguishes a benign repeat of the same in-flight target from a conflicting new target.
  // Either way we do not re-arm.
  if (0 != core_->state_arm_id) {
    result.accepted = false;
    result.error = (target == core_->switching_state)
      ? "State transition already in progress"
      : "Rejected: a state transition is already in progress";
    return result;
  }
  if (core_->waiting_for_mode) {
    result.accepted = false;
    result.error = "Rejected: a mode change is in progress";
    return result;
  }

  result.accepted = true;
  return result;
}

TransitionResult TransitionGuard::admit_state(packml_sm::State target)
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);

  // A previous admission that was never claimed -- its goal died between admission and
  // execution -- would otherwise hold the arm forever. Drop it: this admission supersedes it,
  // and the abandoned token's id no longer matches, so its eventual release is a no-op.
  if (core_->pending.has_value() && core_->pending->arm_id == core_->state_arm_id) {
    core_->state_arm_id = 0;
    core_->switching_state = packml_sm::State::UNDEFINED;
  }
  core_->pending.reset();

  TransitionResult result = decide_state(target);

  InFlightToken::Core::Admission admission;
  admission.target = target;
  admission.result = result;
  if (result.accepted && !result.already_there) {
    admission.arm_id = core_->next_arm_id++;
    core_->state_arm_id = admission.arm_id;
    core_->switching_state = target;
  }
  core_->pending = admission;
  return result;
}

ArmedTransition TransitionGuard::claim_state(packml_sm::State target)
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);
  ArmedTransition claimed;

  if (core_->pending.has_value() && core_->pending->target == target) {
    claimed.result = core_->pending->result;
    if (0 != core_->pending->arm_id) {
      claimed.arm = InFlightToken(core_, core_->pending->arm_id);
    }
    core_->pending.reset();
    return claimed;
  }

  // Nothing was admitted for this target. Deciding now is correct for a caller with no admission
  // step, and for a goal it re-opens the racing-echo hole -- core_->current_state may already carry
  // this goal's own echo, which turns real work into an already_there shortcut. Said out loud
  // rather than passed silently, because the difference is invisible in the result.
  claimed.result = decide_state(target);
  claimed.result.error = claimed.result.error.empty()
    ? std::string("Claimed without admission; already_there was decided against a view that may "
                  "already carry this request's own status echo")
    : claimed.result.error;
  if (claimed.result.accepted && !claimed.result.already_there) {
    const uint64_t arm_id = core_->next_arm_id++;
    core_->state_arm_id = arm_id;
    core_->switching_state = target;
    claimed.arm = InFlightToken(core_, arm_id);
  }
  return claimed;
}

ArmedTransition TransitionGuard::request_state(packml_sm::State target)
{
  admit_state(target);
  return claim_state(target);
}

TransitionResult TransitionGuard::request_mode(packml_sm::ModeType target)
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);   // guard all transition state.
  TransitionResult result;

  // Already in the requested mode — no-op success.
  if (target == core_->current_mode) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // A transition is already in flight: do NOT override it. core_->switching_mode
  // distinguishes a benign repeat from a conflicting new target.
  if (core_->waiting_for_mode) {
    result.accepted = false;
    result.error = (target == core_->switching_mode)
      ? "Mode change already in progress"
      : "Rejected: a mode change is already in progress";
    return result;
  }
  if (0 != core_->state_arm_id) {
    result.accepted = false;
    result.error = "Rejected: a state change is in progress";
    return result;
  }

  // Accept and record the in-flight target.
  core_->waiting_for_mode = true;
  core_->switching_mode = target;
  result.accepted = true;
  return result;
}

bool TransitionGuard::on_status_update(packml_sm::State state, packml_sm::ModeType mode)
{
  std::lock_guard<std::mutex> lk(core_->guard_mutex);   // guard all transition state.
  bool changed = false;

  if (state != core_->current_state) {
    core_->current_state = state;
    changed = true;
  }

  // A status confirming the state this node is switching TO ends the arm, whether or not that
  // status was a change. The arm exists to keep a second goal from racing this one into the same
  // node; once the manager has published the target, the transition is the manager's fact and the
  // node must be free to take the next goal -- above all an ABORT, which has to be able to
  // interrupt a deferred completion that may legitimately still be waiting for many seconds.
  //
  // Keyed on core_->switching_state, not on "the state changed". A change-gated rule gets both
  // halves wrong: it cannot release when a node adopts exactly the state it was asked for (no
  // change to notice), and it releases on any unrelated echo, ending an arm that does not belong
  // to it. InFlightToken still covers the paths where no confirming status ever arrives
  // -- the node refuses the switch, the deferred wait times out, the goal is cancelled.
  if (0 != core_->state_arm_id && core_->switching_state == state) {
    core_->state_arm_id = 0;
    core_->switching_state = packml_sm::State::UNDEFINED;
  }

  if (mode != core_->current_mode) {
    core_->current_mode = mode;
    core_->waiting_for_mode = false;
    changed = true;
  } else if (core_->waiting_for_mode && core_->switching_mode == mode) {
    // DEFENSIVE, AND UNREACHABLE THROUGH THIS CLASS'S OWN API. NO TEST COVERS IT, AND NO TEST CAN.
    //
    // Entering here needs all three at once: an arm outstanding, that arm's target equal to the
    // arriving mode, and the arriving mode equal to current_mode. The first two are easy; the
    // third is impossible to combine with them. request_mode() refuses to arm at all when its
    // target already equals current_mode, and the ONLY writer of current_mode is the branch
    // directly above, which clears the arm on its way past. So by the time current_mode could
    // equal an armed target, the arm is gone.
    //
    // Kept anyway, for one reason: it states the same rule the state side needs -- a release must
    // belong to the request that armed it, which is why it is keyed on switching_mode rather than
    // disarming on any status at all. Symmetry between the two dimensions is worth three lines.
    //
    // No "stranded arm" wedge is reachable through this branch: arming requires
    // target != current_mode, so a status carrying that target differs from current_mode and
    // trips the branch above, which disarms. Treat any such wedge as unconfirmed; a reachable
    // trace would be news.
    //
    // ShakedownGuardTest.RepeatedModeConfirmationsLeaveNoArmStranded covers the reachable half --
    // that ordinary and duplicated mode churn strands nothing -- and says in its own comment that
    // it cannot reach this line.
    core_->waiting_for_mode = false;
  }

  return changed;
}

}  // namespace packml_ros
