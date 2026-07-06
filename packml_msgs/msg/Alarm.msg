# PackML Node (Equipment Module) alarm event message — mirrors the PackML Admin.Alarm tag structure.
#
# Published to the `packml_alarms` topic whenever a PackML Node alarm
# is raised, updated, or cleared.  One message per event.  Self-contained: a
# consumer needs no other message to interpret an alarm.

# Severity scale — mirrors the PackML action levels (NodeHealth.action WARN..ABORT).
uint8 WARN=1
uint8 HOLD=2
uint8 SUSPEND=3
uint8 ABORT=4

# True = alarm activated/updated; false = alarm cleared.
bool trigger

# Timestamp of this alarm event.
builtin_interfaces/Time stamp

# PackML Node (Equipment Module) that reported this alarm.
string node_name

# Severity of the condition (WARN..ABORT above). On a clear event, this is the
# severity the alarm had before it cleared (not reset to a neutral value).
uint8 severity

# User-defined condition id (matches the NodeHealth.error_code that raised this
# alarm); 0 = unspecified. On a clear event, this is the id of the alarm that
# cleared. Not a sentinel — see is_timeout for the heartbeat-timeout case.
uint32 error_code

# True if the manager itself synthesized this alarm from a heartbeat timeout
# (the node stopped reporting, rather than reporting a health condition).
# Also true on the event that clears a timeout alarm.
bool is_timeout

# Human-readable description supplied by the PackML Node. Empty on clear.
string message
