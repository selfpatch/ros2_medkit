# ros2_medkit_fault_manager

Central fault manager node for the ros2_medkit fault management system.

## Overview

The FaultManager node provides a central point for fault aggregation and lifecycle management.
It receives fault reports from multiple sources, aggregates them by `fault_code`, and provides
query and clearing interfaces.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/report_fault` | `ros2_medkit_msgs/srv/ReportFault` | Report a fault occurrence |
| `~/get_faults` | `ros2_medkit_msgs/srv/GetFaults` | Query faults with filtering |
| `~/clear_fault` | `ros2_medkit_msgs/srv/ClearFault` | Clear/acknowledge a fault |

## Features

- **Multi-source aggregation**: Same `fault_code` from different sources creates a single fault
- **Occurrence tracking**: Counts total reports and tracks all reporting sources
- **Severity escalation**: Fault severity is updated if a higher severity is reported
- **Status lifecycle**: PENDING → CONFIRMED → CLEARED (automatic status transitions in Issue #6)
- **Persistent storage**: SQLite backend ensures faults survive node restarts

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `storage_type` | string | `"sqlite"` | Storage backend: `"sqlite"` for persistent storage, `"memory"` for in-memory |
| `database_path` | string | `"/var/lib/ros2_medkit/faults.db"` | Path to SQLite database file. Use `":memory:"` for in-memory SQLite |

### Storage Backends

**SQLite (default)**: Faults are persisted to disk and survive node restarts. The database directory is created automatically if it doesn't exist. Uses WAL mode for optimal performance.

**Memory**: Faults are stored in memory only. Useful for testing or when persistence is not required.

## Usage

### Launch

```bash
# Default (SQLite storage)
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# With custom database path
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p database_path:=/custom/path/faults.db

# With in-memory storage (no persistence)
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p storage_type:=memory
```

### Manual Testing

```bash
# Report a fault
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'MOTOR_OVERHEAT', severity: 2, description: 'Motor temp exceeded', source_id: '/motor_node'}"

# Get all faults (including PENDING)
ros2 service call /fault_manager/get_faults ros2_medkit_msgs/srv/GetFaults \
  "{filter_by_severity: false, severity: 0, statuses: ['PENDING', 'CONFIRMED']}"

# Clear a fault
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT'}"
```

## Building

```bash
colcon build --packages-select ros2_medkit_fault_manager
source install/setup.bash
```

## Testing

```bash
colcon test --packages-select ros2_medkit_fault_manager
colcon test-result --verbose
```

## License

Apache-2.0
