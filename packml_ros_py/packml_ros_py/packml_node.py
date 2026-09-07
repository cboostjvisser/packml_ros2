# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PackmlNode base class for Python-based PackML managed nodes.

This mirrors the C++ PackmlNodeInterface: it exposes the services that the
PackML manager calls to coordinate state/mode transitions, and subscribes
to the manager's status topic to track current system state.

The shared protocol logic (transition coordination + heartbeat state) is
delegated to the C++ PackmlNodeProtocol class via pybind11, ensuring
single-source-of-truth behaviour between C++ and Python nodes.
"""

import threading
import time
import traceback

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from packml_msgs.action import StateTransition as StateTransitionAction
from packml_msgs.srv import ModeTransition
from packml_msgs.msg import Status, NodeHealth, NodeHeartbeat

from packml_ros_py.enums import State
from packml_ros_py.deferred_completion import (
    CompletionSignal,
    Deferral,
    DeferredCompletion,
)
from packml_ros_py._packml_bindings import (
    PackmlNodeProtocol as _Protocol,
    STATE_TRANSITION_ACTION,
    MODE_TRANSITION_SERVICE,
    STATUS_TOPIC,
    HEARTBEAT_TOPIC,
    PARAM_HEARTBEAT_INTERVAL_MS,
    PARAM_DEFERRED_COMPLETION_TIMEOUT_MS,
)


# How often a deferred-completion wait re-reads its goal's cancellation state. Not a deadline: a
# report wakes the wait immediately whatever this is set to, and the overall bound stays the
# per-state deferred_completion_timeout_ms. See _wait_for_deferred_completion().
_CANCEL_RECHECK_INTERVAL_S = 0.05


class PackmlNode(Node):
    """Base class for Python PackML managed nodes.

    Subclass this and override the callback methods to implement your
    equipment module logic. The manager will call the state/mode transition
    services on this node.

    Usage::

        class MyModule(PackmlNode):
            def __init__(self):
                super().__init__('my_module')

            def on_state_transition_request(self, target_state: State) -> bool:
                # Return True to accept the transition
                return True

            def on_mode_transition_request(self, target_mode: int) -> bool:
                return True

            def on_packml_status_changed(self, status: Status) -> None:
                pass
    """

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # C++ PackmlNodeProtocol — single source of truth for shared protocol logic
        self._protocol = _Protocol()

        # Deferred-completion synchronization: the wakeup shared by every deferral this node has
        # in flight. Each deferral's own payload lives in that goal's Deferral record instead of
        # here, which is what keeps one goal's completion out of another's wait -- only the goal's
        # own handle can reach its own record. Mirrors the C++ PackmlNodeInterface's
        # completion_signal_ exactly; a single name-keyed slot let a cancelled goal's late report
        # resolve a later goal that happened to be deferring the same state name.
        self._completion_signal = CompletionSignal()

        deferred_completion_timeout_ms = self.declare_parameter(
            PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, 30000).value
        self._deferred_completion_timeout_s = deferred_completion_timeout_ms / 1000.0

        # Per-state override, named "deferred_completion_timeout_ms.<STATE NAME>" and defaulting to
        # the value above. Mirrors packml_interface.hpp's deferred_completion_timeout_ms_by_state_.
        #
        # Declared LAZILY, on the first deferral for a state, and that is a correctness requirement
        # rather than a saving. Deciding which states to declare means asking defers_completion(),
        # which subclasses override -- and this runs inside the base __init__, i.e. before the
        # subclass constructor body. Any override that reads an attribute of its own
        # (`return state in self._deferred_states`) therefore raised AttributeError during
        # construction, for a subclass written exactly the way this class's own docstring shows.
        # The C++ side cannot fail this way: it probes from init(), which a subclass calls from its
        # constructor body once its own members exist.
        #
        # The visible difference from C++ is that an override appears in `ros2 param list` only
        # after that state has deferred once. A parameter that cannot be declared without possibly
        # crashing the node is the worse trade.
        self._deferred_completion_timeout_ms_default = deferred_completion_timeout_ms
        self._deferred_completion_timeout_s_by_state = {}

        # Action: ~/packml_state_transition
        self._state_transition_action_server = ActionServer(
            self,
            StateTransitionAction,
            '~/' + STATE_TRANSITION_ACTION,
            execute_callback=None,
            goal_callback=self._handle_state_goal,
            handle_accepted_callback=self._handle_state_accepted,
            cancel_callback=self._handle_state_cancel,
        )

        # Service: ~/packml_mode_transition
        self._mode_transition_srv = self.create_service(
            ModeTransition,
            '~/' + MODE_TRANSITION_SERVICE,
            self._handle_mode_transition,
        )

        # Subscription: packml_status (from manager). The manager publishes status
        # latched (TRANSIENT_LOCAL + RELIABLE) so a node that (re)starts after the
        # manager has already published immediately receives the current state.
        # Match that here (parity with the C++ PackmlNodeInterface).
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Sensor-style QoS (best-effort) reused for the heartbeat publisher below.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._status_sub = self.create_subscription(
            Status,
            STATUS_TOPIC,
            self._handle_status,
            status_qos,
        )

        # --- Heartbeat publisher (mirrors C++ PackmlNodeInterface) ---
        heartbeat_interval_ms = self.declare_parameter(
            PARAM_HEARTBEAT_INTERVAL_MS, 1000).value
        heartbeat_interval_ms = heartbeat_interval_ms if heartbeat_interval_ms > 0 else 1000

        self._protocol.heartbeat.init(self.get_name(), heartbeat_interval_ms)

        # Serializes post_event() against the periodic publisher: the C++
        # latch itself is mutex-protected (no torn reads), but without this
        # lock a periodic tick that has already snapshotted a clear latch can
        # be overtaken by a full post_event(fault), then publish its stale
        # HEALTHY with a HIGHER sequence number — which the manager reads as
        # an immediate (false) clear of the fault it just raised.
        # SYNC: keep in lockstep with heartbeat_publish_mutex_ in
        # packml_ros/include/packml_ros/interface/packml_interface.hpp — both
        # guard the identical race on the shared protocol_.heartbeat.
        self._heartbeat_lock = threading.Lock()

        # Sensor-style QoS (best-effort, keep-last) — must match the manager's
        # heartbeat subscription, or QoS-incompatibility silently drops delivery.
        self._heartbeat_pub = self.create_publisher(NodeHeartbeat, '~/' + HEARTBEAT_TOPIC, sensor_qos)
        # Its OWN callback group, not the node's default one. The default group is
        # mutually exclusive, and _handle_state_accepted() deliberately runs
        # on it synchronously (see its docstring: the local accept decision has to
        # beat the manager's status echo, so it cannot be moved to another hop).
        # A subclass whose on_state_transition_request() does blocking work -- e.g.
        # two bounded round trips to external hardware -- therefore held the
        # heartbeat off the wire for the whole hook, and the manager's health
        # monitor read that silence as a dead node and ABORTed the machine. Exactly
        # the false timeout the hook was supposed to avoid. Nothing here mutates
        # state the other callbacks own, so running it concurrently with them is
        # safe; it publishes a health snapshot.
        #
        # Needs a MultiThreadedExecutor to have any effect. PackmlNode does not own
        # the executor, so a single-threaded host still serialises everything and
        # keeps the old behaviour.
        self._heartbeat_timer = self.create_timer(
            heartbeat_interval_ms / 1000.0,
            self._publish_heartbeat,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info('PackmlNode initialized')

    # ─── Properties ──────────────────────────────────────────────────────

    @property
    def current_state(self) -> State:
        """Current PackML state as reported by the manager."""
        return State(int(self._protocol.transitions.current_state))

    @property
    def current_mode(self) -> int:
        """Current PackML mode as reported by the manager."""
        return self._protocol.transitions.current_mode

    # ─── Callbacks for subclasses to override ────────────────────────────

    def on_state_transition_request(self, target_state: State) -> bool:
        """Called when the manager requests a state transition.

        Override in subclass. Return True to accept, False to reject.
        """
        return True

    def on_mode_transition_request(self, target_mode: int) -> bool:
        """Called when the manager requests a mode transition.

        Override in subclass. Return True to accept, False to reject.
        """
        return True

    def on_packml_status_changed(self, status: Status) -> None:
        """Called when the manager publishes a new status.

        Override in subclass to react to system-wide state/mode updates.
        """
        pass

    def defers_completion(self, state: State) -> bool:
        """Return True for states where your own commanded work finishes later than the
        transition is approved -- the states you implement on_deferred_work() for.

        The default, for every state, is to complete the moment
        on_state_transition_request() approves the transition. Override in subclass. This is how
        a node declares -- in its own code, not manager-side configuration -- that it
        participates in coordinated completion for a given state.

        Must be a static, state-shape answer (does this node defer THIS state at all), not a
        per-request decision: __init__ probes it once per state to decide which
        deferred_completion_timeout_ms.<STATE> overrides to declare.
        """
        return False

    # ─── Health / heartbeat API ───────────────────────────────────────────

    def get_health_status(self) -> NodeHealth:
        """Return the current health of this Equipment Module.

        Override in subclass to report real conditions.
        The base implementation returns HEALTHY / NONE.
        """
        msg = NodeHealth()
        msg.status = NodeHealth.HEALTHY
        msg.action = NodeHealth.NONE
        return msg

    def _set_heartbeat_active(self, active: bool) -> None:
        """Pause or resume heartbeat publishing.

        Intended for derived test/demo Equipment Modules to simulate a crashed or
        silent node (no heartbeat = timeout in the HealthMonitor) — not part of the
        public API (mirrors the protected `set_heartbeat_active` in the C++
        PackmlNodeInterface).
        """
        self._protocol.heartbeat.set_active(active)

    def _make_heartbeat(self, health: NodeHealth) -> NodeHeartbeat:
        """Assemble a NodeHeartbeat with the standard header (node_name, next sequence,
        interval) and the given health.  Single source for both post_event() and the
        periodic timer; calls next_sequence() exactly once per published heartbeat
        (mirrors the C++ PackmlNodeInterface::make_heartbeat)."""
        hb = NodeHeartbeat()
        hb.node_name = self._protocol.heartbeat.node_name
        hb.sequence_number = self._protocol.heartbeat.next_sequence()
        hb.heartbeat_interval_ms = self._protocol.heartbeat.interval_ms
        hb.health = health
        return hb

    def post_event(self, health: NodeHealth) -> None:
        """Immediately publish a heartbeat with the given health state.

        Bypasses the periodic timer.  Use for safety-critical events (e.g. E-stop)
        where waiting up to heartbeat_interval_ms for the next tick is unacceptable.
        The event is *latched*: the periodic publisher repeats this health on every
        subsequent tick (instead of calling get_health_status()), so a transient
        getter cannot flap the alarm/state.  Posting a healthy/NONE event clears it.
        """
        with self._heartbeat_lock:
            if health.action == NodeHealth.NONE:
                self._protocol.heartbeat.clear_latch()
            else:
                self._protocol.heartbeat.set_latch(
                    health.status, health.action, health.error_code, health.message,
                    health.instance_id)
            self._heartbeat_pub.publish(self._make_heartbeat(health))

    # ─── Dedicated completion-signal API ──────────────────────────────────

    def on_deferred_work(self, state: State, completion: DeferredCompletion) -> None:
        """Start this node's own commanded work for `state`, and resolve `completion` when that
        work has genuinely finished or definitively failed.

        Called once per accepted goal, for exactly the states defers_completion() returns True
        for. Override in subclass.

        Runs on the executor thread and must not block: put long work on a thread of your own and
        capture `completion` into it. Reporting inline, before returning, is also fine for work
        that is already done.

        The handle is per-goal, which is the point of the shape: identity travels with the work
        instead of being re-derived when the report arrives. Work that outlives its own goal -- a
        homing motion already issued to hardware, still running after an operator's ABORT
        cancelled the goal that asked for it -- reports into a record nothing is waiting on, and
        can see that coming via completion.abandoned. It cannot resolve whatever goal is deferring
        by then, not even when that goal is for the same state.

        Structurally separate from health/heartbeat reporting (post_event()): completion is a
        progress fact, not a health fact. Mirrors the C++
        PackmlNodeInterface::on_deferred_work().
        """
        self.get_logger().error(
            f'defers_completion({state.name}) returned True but on_deferred_work() is not '
            'implemented, so nothing can ever complete this state. Implement it, or stop '
            'deferring this state.')
        completion.report(
            False, StateTransitionAction.Result.INVALID_STATE_REQUEST,
            f'Node defers completion for {state.name} but implements no on_deferred_work()')

    def _publish_heartbeat(self) -> None:
        """Periodic heartbeat callback — called by the internal timer."""
        if not self._protocol.heartbeat.is_active:
            return
        # The lock makes snapshot → getter → publish atomic against a
        # concurrent post_event() (see _heartbeat_lock); the snapshot itself
        # is additionally torn-read-safe via the C++ latch mutex, matching
        # the C++ periodic publisher.
        with self._heartbeat_lock:
            latch = self._protocol.heartbeat.latch_snapshot()
            if latch.active:
                # A post_event() fault is latched — repeat it, don't poll the getter.
                health = NodeHealth()
                health.status = latch.status
                health.action = latch.action
                health.error_code = latch.error_code
                health.message = latch.message
                health.instance_id = latch.instance_id
            else:
                health = self.get_health_status()
            self._heartbeat_pub.publish(self._make_heartbeat(health))

    # ─── Internal action / service handlers ──────────────────────────────

    def _handle_state_goal(self, goal_request):
        """Handle a new ~/packml_state_transition goal request.

        Pure admission control -- always accept. Business-logic accept/reject (the
        on_state_transition_request() hook) happens in _handle_state_accepted() below
        as a normal, fast succeeded/aborted result, so a rejection still carries a
        proper Result message (error_code/message) rather than the goal simply
        vanishing with no result at all.

        admit_state() runs here, not in _handle_state_accepted(): this callback's return
        value is what the SendGoal service response is built from
        (rclpy.action.server._execute_goal_request sends that response immediately after
        calling this), and that response cannot reach the manager -- unblocking its own
        bounded acceptance wait, which gates publish_status() -- before this call returns.
        So current_state here is still this goal's PRE-echo view, and it is the only moment
        at which "am I already in the requested state" can be answered without the answer
        being poisoned by the very request being answered.
        """
        self._protocol.transitions.admit_state(State(goal_request.state.val))
        return GoalResponse.ACCEPT

    def _handle_state_cancel(self, goal_handle):
        """Wake a deferred wait immediately rather than only via its own timeout."""
        del goal_handle
        with self._completion_signal.deferral_progress_cv:
            self._completion_signal.deferral_progress_cv.notify_all()
        return CancelResponse.ACCEPT

    def _handle_state_accepted(self, goal_handle):
        """Handle a newly-accepted ~/packml_state_transition goal.

        Runs synchronously: rclpy's ActionServer calls this immediately after the
        goal-acceptance response has already been sent to the manager (see
        rclpy.action.ActionServer._execute_goal_request), on the same executor thread
        -- not inside execute_callback's separate task-scheduling hop. Doing the fast
        accept/reject/instant-complete decision here, with no further hop, keeps this
        node's local processing (no network round trip) reliably ahead of the
        manager's own status-topic publish, which needs a full round trip (this
        node's goal-response ack reaching the manager, then its status update
        reaching back here) to arrive -- see fanout_state_transition()'s own comment
        in packml_interface.hpp for why that ordering matters: a node that saw status
        first would hit its own already_there shortcut and skip
        on_state_transition_request()/defers_completion() entirely. Only the
        (potentially long) deferred wait is moved onto its own thread below.
        """
        target = State(goal_handle.request.state.val)
        # `claimed` owns the arm; releasing it is explicit on every path below rather than
        # left to the collector, because a live traceback or a reference cycle can delay
        # collection and the next goal would be refused for as long as it did.
        claimed = self._protocol.transitions.claim_state(target)
        result = claimed.result

        if result.already_there:
            claimed.arm.release()
            self.get_logger().info(f'Already in state: {target.name}')
            action_result = StateTransitionAction.Result()
            action_result.success = True
            goal_handle.executing()
            goal_handle.succeed(action_result)
            return

        if result.error:
            self.get_logger().warn(result.error)

        # defers_completion() is a static, state-shape answer (does this node defer THIS state at
        # all), not a per-request decision, so asking it here costs nothing. Computing it even
        # when the transition is about to be rejected below is harmless -- it is simply unused.
        # Exceptions from either subclass hook release the claim arm and abort the goal.
        # This permits subsequent goals and gives the manager an immediate result.
        try:
            will_defer = result.accepted and self.defers_completion(target)
            approved = result.accepted and self.on_state_transition_request(target)
        except Exception:
            claimed.arm.release()
            error_msg = f'Node hook raised handling the {target.name} transition'
            self.get_logger().error(f'{error_msg}:\n{traceback.format_exc()}')
            action_result = StateTransitionAction.Result()
            action_result.success = False
            action_result.error_code = StateTransitionAction.Result.INVALID_STATE_REQUEST
            action_result.message = error_msg
            goal_handle.executing()
            goal_handle.abort(action_result)
            return

        if not approved:
            claimed.arm.release()
            error_msg = 'Node rejected state transition'
            self.get_logger().warn(error_msg)
            action_result = StateTransitionAction.Result()
            action_result.success = False
            action_result.error_code = StateTransitionAction.Result.INVALID_STATE_REQUEST
            action_result.message = error_msg
            goal_handle.executing()
            goal_handle.abort(action_result)
            return

        self.get_logger().info(f'Approved state transition to {target.name}')

        if not will_defer:
            claimed.arm.release()
            action_result = StateTransitionAction.Result()
            action_result.success = True
            goal_handle.executing()
            goal_handle.succeed(action_result)
            self._mark_state_locally_reached(target)
            return

        # This goal's own completion record, and the only handle that can reach it. Nothing else
        # shares it, so a report from an earlier goal's still-running work has no path to this
        # wait.
        deferral = Deferral(self._completion_signal, target)

        # Declare and cache the override before dispatching the hook. A hook may use the same
        # parameter to keep its own deadline inside this deferred-completion safety net.
        self._deferred_completion_timeout_s_for(target)

        # Synchronously, for the same reason the accept decision above runs here: this is what
        # dispatches the node's own commanded work, and the work has to be under way before the
        # manager's status echo can arrive. Reporting from inside the hook is fine -- the record
        # already exists, so an instant report is simply already there when the wait below starts.
        # An exception from the dispatch hook releases the arm and aborts the goal;
        # no deferred wait is started without a completion report source.
        try:
            self.on_deferred_work(target, DeferredCompletion(deferral, self.get_logger()))
        except Exception:
            claimed.arm.release()
            error_msg = f'on_deferred_work() raised dispatching {target.name}'
            self.get_logger().error(f'{error_msg}:\n{traceback.format_exc()}')
            action_result = StateTransitionAction.Result()
            action_result.success = False
            action_result.error_code = StateTransitionAction.Result.INVALID_STATE_REQUEST
            action_result.message = error_msg
            goal_handle.executing()
            goal_handle.abort(action_result)
            return

        # Deferred: hand off to a background thread that only waits for that report -- the one
        # part of this that can legitimately take a long time.
        # The arm moves with the work: the transition is still in flight until that thread
        # resolves the goal, so releasing it here would let a second goal in while the first
        # is still waiting.
        goal_handle.executing()
        threading.Thread(
            target=self._wait_for_deferred_completion,
            args=(goal_handle, deferral, claimed),
            daemon=True,
        ).start()

    def _deferred_completion_timeout_s_for(self, state: State) -> float:
        """Per-state deferred-completion timeout, declaring its parameter on first use.

        See __init__ for why the declaration is lazy: the eager version had to ask the subclass's
        defers_completion() override before the subclass constructor body had run.
        """
        cached = self._deferred_completion_timeout_s_by_state.get(state)
        if cached is not None:
            return cached
        state_timeout_param = f'{PARAM_DEFERRED_COMPLETION_TIMEOUT_MS}.{state.name}'
        if not self.has_parameter(state_timeout_param):
            self.declare_parameter(
                state_timeout_param, self._deferred_completion_timeout_ms_default)
        timeout_s = self.get_parameter(state_timeout_param).value / 1000.0
        self._deferred_completion_timeout_s_by_state[state] = timeout_s
        return timeout_s

    def _wait_for_deferred_completion(self, goal_handle, deferral, claimed=None):
        """Bounded so a subclass that never reports can't hang the goal forever.

        A manager-issued cancel (_handle_state_cancel, above) also wakes this wait promptly rather
        than only via this timeout.
        """
        state = deferral.state
        timeout_s = self._deferred_completion_timeout_s_for(state)

        # Re-checked on an interval rather than waited on once, matching
        # wait_for_deferred_completion() in packml_interface.hpp and for the same reason: rclpy
        # runs the cancel callback BEFORE it moves the goal to CANCELING (ActionServer accepts the
        # callback's response, then updates the goal state), so _handle_state_cancel()'s notify
        # arrives while is_cancel_requested is still False. A single wait_for() re-evaluates a
        # predicate that is still entirely false, parks again, and -- since nothing notifies a
        # second time -- sleeps until the full timeout. The interval is a re-check cadence, not a
        # deadline: a report still wakes this instantly and the overall bound is still timeout_s.
        deadline = time.monotonic() + timeout_s
        with self._completion_signal.deferral_progress_cv:
            while not (deferral.reported or goal_handle.is_cancel_requested):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._completion_signal.deferral_progress_cv.wait(
                    min(_CANCEL_RECHECK_INTERVAL_S, remaining))
            reported = deferral.reported
            success = deferral.success
            error_code = deferral.error_code
            message = deferral.message
            if not reported:
                # Nothing waits on this goal from here on. Marking it says so to a report that
                # arrives later -- work that kept running past its own cancellation -- which is
                # discarded with a warning instead of silently going nowhere. It cannot reach any
                # OTHER goal's wait: this record is the only thing its handle refers to.
                deferral.abandoned = True

        action_result = StateTransitionAction.Result()

        # Filled in BEFORE the cancel branch, so a report that landed in the same wakeup as the
        # cancel still reaches the client. Matches C++, which populates the result inside the
        # locked section and hands that same object to canceled(); constructing a fresh empty
        # Result on the cancel path would throw the report away.
        if reported:
            action_result.success = success
            action_result.error_code = error_code
            action_result.message = message

        # The claim arm is released, and on success the local state adopted, BEFORE the
        # goal handle resolves. The manager fans out the next state the moment the result
        # arrives, and the round-closing node receives that goal within the same
        # millisecond -- a claim still held (or a guard still waiting for the status
        # echo) at that instant rejects it as "already in progress" and fails the whole
        # round. Leaving the release to the collector when this thread dies is the exact
        # delay the claim's own contract warns against.
        if goal_handle.is_cancel_requested:
            if claimed is not None:
                claimed.arm.release()
            goal_handle.canceled(action_result)
            return

        if not reported:
            error_msg = (
                f'Deferred completion for {state.name} timed out with no report')
            self.get_logger().warn(error_msg)
            action_result.success = False
            action_result.error_code = StateTransitionAction.Result.INVALID_STATE_REQUEST
            action_result.message = error_msg
            if claimed is not None:
                claimed.arm.release()
            goal_handle.abort(action_result)
            return

        action_result.success = success
        action_result.error_code = error_code
        action_result.message = message
        if success:
            self._mark_state_locally_reached(state)
            if claimed is not None:
                claimed.arm.release()
            goal_handle.succeed(action_result)
        else:
            if claimed is not None:
                claimed.arm.release()
            goal_handle.abort(action_result)

    def _mark_state_locally_reached(self, state: State) -> None:
        """Update this node's own view of its current state the moment it locally
        commits to succeeding a transition, rather than waiting for the manager's
        status-topic echo to arrive.

        The transitions guard's "already in progress" rejection is otherwise only
        cleared by on_status_update() -- when the manager moves through two
        coordinated states in quick succession (e.g. the IDLE waypoint on the way to
        STARTING), this node's own next goal can arrive before that echo does, and
        get rejected as a conflicting in-flight request even though nothing actually
        conflicts: this node already knows, locally, that it just finished the
        previous one. Reuses on_status_update() (rather than adding a new
        TransitionGuard method) since the effect needed -- clear the waiting flag,
        adopt the new current state -- is exactly what it already does for a real
        echo; a later, genuinely-redundant echo of the same state is then just a
        no-op.
        """
        self._protocol.transitions.on_status_update(state, self._protocol.transitions.current_mode)

    def _handle_mode_transition(self, request, response):
        """Handle ~/packml_mode_transition service call from manager."""
        target_mode = int(request.mode.val)
        result = self._protocol.transitions.request_mode(target_mode)

        if result.already_there:
            self.get_logger().info(f'Already in mode: {target_mode}')
            response.success = True
            return response

        if result.error:
            self.get_logger().warn(result.error)

        # Ask subclass whether to accept
        if result.accepted and self.on_mode_transition_request(target_mode):
            self.get_logger().info(f'Approved mode transition to {target_mode}')
            response.success = True
        else:
            error_msg = 'Node rejected mode transition'
            response.success = False
            response.message = error_msg
            self.get_logger().warn(error_msg)

        return response

    def _handle_status(self, msg: Status):
        """Handle packml_status subscription message from manager."""
        changed = self._protocol.transitions.on_status_update(State(msg.state.val), int(msg.mode.val))

        if changed:
            self.get_logger().debug(
                f'Status updated - state: {State(msg.state.val).name}, mode: {msg.mode.val}'
            )
            self.on_packml_status_changed(msg)
