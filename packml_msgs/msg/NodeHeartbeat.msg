# Periodic health heartbeat published by every PackML Node (Equipment Module).
# The HealthMonitor inside the manager subscribes to these to detect node
# failures and route PackML actions (Hold / Suspend / Abort).

# Name of the publishing node (matches rclcpp::Node::get_name())
string node_name

# Monotonically increasing counter, starting at 1 (0 = unsequenced).
# Effectively unbounded — at any realistic heartbeat rate the uint64 range is
# never exhausted, so wrap is not handled.
uint64 sequence_number

# Nominal publishing interval advertised by this node (milliseconds).
# The HealthMonitor uses this to compute its per-node timeout:
#   timeout_ms = heartbeat_interval_ms * timeout_factor
uint32 heartbeat_interval_ms

# Current health status of this node
NodeHealth health
