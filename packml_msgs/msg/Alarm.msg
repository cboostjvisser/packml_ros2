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

# Global (manager-assigned) fault number resolved from the machine error
# catalog; 0 if no catalog is loaded or no entry matched this node/error_code.
uint32 global_code

# Instance identifier for a per-location/per-unit fault (e.g. "cell_north"),
# propagated from NodeHealth.instance_id; empty if this fault is not instanced.
string instance_id

# PackML machine state at the moment this alarm was generated.
State state

# Identifies which "stop episode" this alarm belongs to; 0 = none. The manager
# mints a new id on the first alarm raised while the health gate is closed, and
# every alarm raised or cleared before the gate reopens carries that same id —
# so the earliest timestamp sharing an id is the first fault of that episode.
# This is a single-manager grouping id: it does not correlate alarms across
# multiple machines on the same line (that needs its own durable key and
# lifecycle, not yet designed).
uint64 stop_event_id

# Durable identifier for the fault's source (source package + symbolic name),
# for cross-machine correlation independent of this manager's global_code
# numbering (which collides once a line has more than one machine). Currently
# always empty: the per-node error catalog does not yet carry a package
# identity to build this from. Reserved here so adding it later needs no
# second breaking wire change.
string source_key
