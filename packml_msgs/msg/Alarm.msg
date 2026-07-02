# PackML Node (Equipment Module) alarm event message — mirrors the PackML Admin.Alarm tag structure.
#
# Published to the `packml_alarms` topic whenever a PackML Node alarm
# is raised, updated, or cleared.  One message per event.

# True = alarm activated/updated; false = alarm cleared.
bool trigger

# Timestamp of this alarm event.
builtin_interfaces/Time stamp

# PackML Node (Equipment Module) that reported this alarm.
string node_name

# Health snapshot that caused this alarm event.
# health.error_code — alarm identifier; 0 is reserved for heartbeat-timeout alarms
# health.action     — severity level (WARN=1 .. ABORT=4)
# health.message    — human-readable description supplied by the PackML Node
NodeHealth health
