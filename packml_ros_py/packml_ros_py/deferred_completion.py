# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Per-goal deferred completion for Python PackML nodes.

SYNC: keep in lockstep with packml_ros/include/packml_ros/deferred_completion.hpp -- both
implement the same contract, that a deferred completion belongs to exactly one goal and can only
be resolved through the handle that goal's on_deferred_work() was given.
"""

import threading

from packml_ros_py.enums import State


class CompletionSignal:
    """One wakeup shared by every deferral a node has in flight, and by the node itself.

    Shared rather than per-deferral so a node can wake all of its waiters at once; referenced by
    the deferrals rather than reached through the node so that a handle held by a subclass's own
    work thread stays usable after the node is gone (reporting into it then does nothing, which is
    the point).
    """

    def __init__(self):
        self.deferrals_lock = threading.Lock()
        self.deferral_progress_cv = threading.Condition(self.deferrals_lock)


class Deferral:
    """One goal's deferred completion.

    Every field except `state` is guarded by signal.deferrals_lock.
    """

    def __init__(self, signal: CompletionSignal, state: State):
        self.signal = signal
        self.state = state
        self.reported = False
        self.success = False
        self.error_code = 0
        self.message = ''
        # Set once the waiting thread has stopped waiting on this record without a report
        # (cancelled or timed out). Only a diagnostic: a report arriving afterwards is already
        # inert, and this is what lets it say so instead of vanishing.
        self.abandoned = False


class DeferredCompletion:
    """The one way to resolve a deferred state transition.

    Handed to on_deferred_work() for the goal it belongs to, and carried by the subclass into
    whatever thread does that goal's work. Cheap to pass around and safe to hold for as long as
    the work takes.
    """

    def __init__(self, deferral: Deferral, logger=None):
        self._deferral = deferral
        self._logger = logger

    def report(self, success: bool = True, error_code: int = 0, message: str = '') -> None:
        """Resolve this goal: success False aborts it, carrying error_code/message back.

        Safe to call at any time, including after the goal is gone -- a report for a goal nothing
        is waiting on is discarded with a warning, never applied to whatever goal came next. First
        report wins; a second one for the same goal is discarded the same way.
        """
        d = self._deferral
        with d.signal.deferral_progress_cv:
            if d.abandoned:
                self._warn(
                    f'Deferred completion reported for {d.state.name} after its goal was '
                    'cancelled or timed out -- discarding. The work this reports on kept running '
                    'past the goal that asked for it; check whether it should be observing '
                    'abandoned and stopping early.')
                return
            if d.reported:
                self._warn(
                    f'Deferred completion for {d.state.name} reported more than once -- keeping '
                    'the first report and discarding this one.')
                return
            d.reported = True
            d.success = success
            d.error_code = error_code
            d.message = message
            d.signal.deferral_progress_cv.notify_all()

    @property
    def abandoned(self) -> bool:
        """True once nothing is waiting on this goal any more.

        Long-running work can poll this to stop early instead of finishing into a report that will
        be discarded.
        """
        with self._deferral.signal.deferral_progress_cv:
            return self._deferral.abandoned

    @property
    def state(self) -> State:
        """The state this goal is completing."""
        return self._deferral.state

    def _warn(self, text: str) -> None:
        if self._logger is not None:
            self._logger.warn(text)
