# PackML Equipment Module health status.
# Reported in each NodeHeartbeat and passed to HealthMonitor callbacks.

# ------------------------------------------------------------------
# Status constants
# ------------------------------------------------------------------
int32 UNKNOWN=0
int32 HEALTHY=1
int32 DEGRADED=2
int32 ERROR=3

# ------------------------------------------------------------------
# Recommended PackML action constants
# ------------------------------------------------------------------
int32 NONE=0
int32 WARN=1
int32 HOLD=2
int32 SUSPEND=3
int32 ABORT=4

# ------------------------------------------------------------------
# Fields
# ------------------------------------------------------------------

# Current health level of the node
int32 status

# Recommended PackML action based on this health condition
int32 action

# Human-readable description of the condition
string message

# User-defined error code (0 = no error)
int32 error_code
