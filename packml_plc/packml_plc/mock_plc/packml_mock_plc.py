# Mock PackML PLC OPC UA server for testing
# ============================================
#
# This mock server simulates a PackML OPC UA interface.
# Interface is a little inspired by: https://support.industry.siemens.com/cs/document/49970441/simatic-omac-packml-v3-machine-and-unit-states-(lpmlv30)
# The behavior follows a practical PackML state-machine model for testing.
#
# Structure: ns=3;s="PackML_Status"."EM00"."Unit"."<name>"
#
# State names per PackML:
#   Stopped, Idle, Starting, Execute, Completing, Complete, Resetting,
#   Holding, Held, Unholding, Suspending, Suspended, Unsuspending,
#   Aborting, Aborted, Clearing, Stopping
#
# Command names (Cmd_ prefix):
#   Cmd_Reset, Cmd_Start, Cmd_Stop, Cmd_Hold, Cmd_Unhold,
#   Cmd_Suspend, Cmd_Unsuspend, Cmd_Abort, Cmd_Clear
#

import threading
import time
from opcua import Server, ua

# PackML state names (17 states)
PACKML_STATES = [
    'Stopped', 'Idle', 'Starting', 'Execute', 'Completing', 'Complete',
    'Clearing', 'Suspended', 'Aborting', 'Aborted', 'Holding', 'Held',
    'Unholding', 'Suspending', 'Unsuspending', 'Resetting', 'Stopping'
]

# State number mapping used by the mock
# Numbers aligned with packml spec and packml_sm library
STATE_NUMBERS = {
    'Undefined': 0, 'Clearing': 1, 'Stopped': 2, 'Starting': 3, 'Idle': 4,
    'Suspended': 5, 'Execute': 6, 'Stopping': 7, 'Aborting': 8, 'Aborted': 9,
    'Holding': 10, 'Held': 11, 'Unholding': 12, 'Suspending': 13,
    'Unsuspending': 14, 'Resetting': 15, 'Completing': 16, 'Complete': 17
}

# PackML command names
PACKML_COMMANDS = [
    'Cmd_Reset', 'Cmd_Start', 'Cmd_Stop', 'Cmd_Hold', 'Cmd_Unhold',
    'Cmd_Suspend', 'Cmd_Unsuspend', 'Cmd_Abort', 'Cmd_Clear'
]

# Acting states: must complete via SC signal or timeout
ACTING_STATES = [
    'Starting', 'Completing', 'Resetting', 'Holding', 'Unholding',
    'Suspending', 'Unsuspending', 'Stopping', 'Aborting', 'Clearing'
]

# Wait states: machine waits for command
WAIT_STATES = ['Stopped', 'Idle', 'Execute', 'Complete', 'Suspended', 'Held', 'Aborted']


class PackMLMockPLC:
    """Mock PackML OPC UA PLC server for testing and demos.

    Simulates a PackML function block with PackML state machine.

    Features:
    - All 17 PackML states with boolean outputs
    - SC (State Complete) input for acting state transitions
    - Configurable transition delays
    - Transition validation
    """

    def __init__(self, endpoint='opc.tcp://0.0.0.0:4840', transition_delay=0.0, verbose=False):
        """Initialize the mock PLC.

        Args:
            endpoint: OPC UA endpoint URL
            transition_delay: Delay (seconds) for acting states. 0=instant
            verbose: If True, print state changes to console
        """
        self.endpoint = endpoint
        self.transition_delay = transition_delay
        self.verbose = verbose
        self._server = Server()
        self._server.set_endpoint(self.endpoint)
        # Register namespace index 2 (placeholder) and 3 (PackML_Status)
        self._server.register_namespace("placeholder")  # ns=2
        self._namespace = self._server.register_namespace("PackML_Status")  # ns=3
        self._objects = self._server.get_objects_node()

        # Build hierarchy:
        # ns=3;s="PackML_Status"."EM00"."Unit"
        packml_nodeid = ua.NodeId('"PackML_Status"', self._namespace)
        self._packml = self._objects.add_object(packml_nodeid, "PackML_Status")
        em00_nodeid = ua.NodeId('"PackML_Status"."EM00"', self._namespace)
        self._em00 = self._packml.add_object(em00_nodeid, "EM00")
        unit_nodeid = ua.NodeId('"PackML_Status"."EM00"."Unit"', self._namespace)
        self._unit = self._em00.add_object(unit_nodeid, "Unit")

        # Add all 17 PackML state variables (boolean)
        # Format: ns=3;s="PackML_Status"."EM00"."Unit"."Stopped"
        self._states = {}
        for state in PACKML_STATES:
            nodeid = ua.NodeId(f'"PackML_Status"."EM00"."Unit"."{state}"', self._namespace)
            var = self._unit.add_variable(nodeid, state, False)
            var.set_writable()
            self._states[state] = var

        # Add SC (State Complete) input variable
        nodeid = ua.NodeId('"PackML_Status"."EM00"."Unit"."SC"', self._namespace)
        self._sc = self._unit.add_variable(nodeid, "SC", False)
        self._sc.set_writable()

        # Add PackML command variables (boolean)
        # Format: ns=3;s="PackML_Status"."EM00"."Unit"."Cmd_Reset"
        self._commands = {}
        for cmd in PACKML_COMMANDS:
            nodeid = ua.NodeId(f'"PackML_Status"."EM00"."Unit"."{cmd}"', self._namespace)
            var = self._unit.add_variable(nodeid, cmd, False)
            var.set_writable()
            self._commands[cmd] = var

        # Initialize to STOPPED state
        self._states['Stopped'].set_value(True)
        self._running = False
        self._thread = None
        self._acting_state_start = None  # Timestamp when acting state started
        self._last_state = 'Stopped'  # Track for change detection

    def start(self):
        """Start the mock PLC server."""
        self._server.start()
        self._running = True
        self._thread = threading.Thread(target=self._main_loop, daemon=True)
        self._thread.start()
        print(f"PackML Mock PLC OPC UA server started at {self.endpoint}")
        if self.transition_delay > 0:
            print(f"  Transition delay: {self.transition_delay}s for acting states")

    def stop(self):
        """Stop the mock PLC server."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self._server.stop()
        print("PackML Mock PLC OPC UA server stopped.")

    def set_state(self, state_name, silent=False):
        """Directly set the PLC to a specific state (for testing).

        Args:
            state_name: One of PACKML_STATES (e.g., 'Stopped', 'Idle', 'Execute')
            silent: If True, don't print state change (for internal transitions)
        """
        if state_name not in self._states:
            raise ValueError(f"Invalid state: {state_name}. Must be one of {PACKML_STATES}")

        old_state = self._last_state
        for s, s_var in self._states.items():
            s_var.set_value(s == state_name)

        # Reset acting state timer
        if state_name in ACTING_STATES:
            self._acting_state_start = time.time()
        else:
            self._acting_state_start = None

        # Print state change if verbose and state actually changed
        if self.verbose and not silent and state_name != old_state:
            print(f"[PackML] {old_state} -> {state_name} (#{STATE_NUMBERS.get(state_name, 0)})")

        self._last_state = state_name

    def get_state(self):
        """Get the current state name."""
        for s, s_var in self._states.items():
            if s_var.get_value():
                return s
        return None

    def get_state_number(self):
        """Get the current state as integer."""
        state_name = self.get_state()
        if state_name is None:
            return STATE_NUMBERS['Undefined']
        return STATE_NUMBERS.get(state_name, STATE_NUMBERS['Undefined'])

    def _main_loop(self):
        """Main PLC loop - processes commands and updates states."""
        # PackML command -> (transitional_state, final_state)
        # Acting states are entered first, then auto-complete to final state
        cmd_transitions = {
            'Cmd_Reset':     ('Resetting', 'Idle'),
            'Cmd_Start':     ('Starting', 'Execute'),
            'Cmd_Stop':      ('Stopping', 'Stopped'),
            'Cmd_Hold':      ('Holding', 'Held'),
            'Cmd_Unhold':    ('Unholding', 'Execute'),
            'Cmd_Suspend':   ('Suspending', 'Suspended'),
            'Cmd_Unsuspend': ('Unsuspending', 'Execute'),
            'Cmd_Abort':     ('Aborting', 'Aborted'),
            'Cmd_Clear':     ('Clearing', 'Stopped'),
        }

        # Acting state -> final state (when SC or timeout)
        acting_to_final = {
            'Resetting': 'Idle',
            'Starting': 'Execute',
            'Stopping': 'Stopped',
            'Holding': 'Held',
            'Unholding': 'Execute',
            'Suspending': 'Suspended',
            'Unsuspending': 'Execute',
            'Aborting': 'Aborted',
            'Clearing': 'Stopped',
            'Completing': 'Complete',
        }

        # PackML valid transitions (from_state -> allowed_commands)
        # Allowed transitions by current state
        valid_transitions = {
            'Stopped': ['Cmd_Reset', 'Cmd_Abort'],
            'Idle': ['Cmd_Start', 'Cmd_Stop', 'Cmd_Abort'],
            'Starting': ['Cmd_Stop', 'Cmd_Abort'],
            'Execute': ['Cmd_Hold', 'Cmd_Suspend', 'Cmd_Stop', 'Cmd_Abort'],
            'Completing': ['Cmd_Stop', 'Cmd_Abort'],
            'Complete': ['Cmd_Reset', 'Cmd_Stop', 'Cmd_Abort'],
            'Resetting': ['Cmd_Stop', 'Cmd_Abort'],
            'Holding': ['Cmd_Stop', 'Cmd_Abort'],
            'Held': ['Cmd_Unhold', 'Cmd_Stop', 'Cmd_Abort'],
            'Unholding': ['Cmd_Stop', 'Cmd_Abort'],
            'Suspending': ['Cmd_Stop', 'Cmd_Abort'],
            'Suspended': ['Cmd_Unsuspend', 'Cmd_Stop', 'Cmd_Abort'],
            'Unsuspending': ['Cmd_Stop', 'Cmd_Abort'],
            'Stopping': ['Cmd_Abort'],
            'Aborting': [],
            'Aborted': ['Cmd_Clear'],
            'Clearing': [],
        }

        while self._running:
            current_state = self.get_state()

            # Check for SC (State Complete) signal for acting states
            if current_state in acting_to_final:
                sc_triggered = self._sc.get_value()
                timeout_triggered = (
                    self.transition_delay > 0 and
                    self._acting_state_start and
                    (time.time() - self._acting_state_start) >= self.transition_delay
                )

                if sc_triggered or timeout_triggered or self.transition_delay == 0:
                    # Transition to final state
                    final_state = acting_to_final[current_state]
                    self.set_state(final_state)
                    self._sc.set_value(False)  # Clear SC
                    continue

            # Process commands
            for cmd, var in self._commands.items():
                if var.get_value():
                    allowed = valid_transitions.get(current_state, [])
                    if cmd in allowed:
                        trans_state, final_state = cmd_transitions.get(cmd, (None, None))
                        if trans_state:
                            if self.transition_delay > 0:
                                # Go through transitional state
                                self.set_state(trans_state)
                            else:
                                # Instant mode (for tests): skip to final state
                                self.set_state(final_state)
                    # Always clear the command after processing
                    var.set_value(False)

            time.sleep(0.05)


def main():
    """Main entry point for running the mock PLC standalone."""
    import argparse
    parser = argparse.ArgumentParser(description='PackML Mock PLC OPC UA Server')
    parser.add_argument('--delay', type=float, default=1.0,
                        help='Transition delay for acting states (default: 1.0s)')
    parser.add_argument('--port', type=int, default=4840,
                        help='OPC UA port (default: 4840)')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Suppress state change messages')
    args = parser.parse_args()

    endpoint = f'opc.tcp://0.0.0.0:{args.port}'
    plc = PackMLMockPLC(endpoint=endpoint, transition_delay=args.delay, verbose=not args.quiet)
    plc.start()
    print(f"\nPackML Mock PLC running. Press Ctrl+C to stop.")
    print(f"Initial state: {plc.get_state()} (#{plc.get_state_number()})")
    print("Waiting for OPC UA commands...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print()
        plc.stop()


if __name__ == '__main__':
    main()
