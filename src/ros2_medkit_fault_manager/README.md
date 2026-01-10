# ros2_medkit_fault_manager

Central fault manager node for the ros2_medkit fault management system.

## Overview

The FaultManager node provides a central point for fault aggregation and lifecycle management.
It receives fault reports from multiple sources, aggregates them by `fault_code`, and provides
query and clearing interfaces.

## Quick Start

By default, faults are confirmed immediately when reported - no additional configuration needed.

```bash
# Start the fault manager
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# Report a fault - it's immediately CONFIRMED
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'MOTOR_OVERHEAT', event_type: 0, severity: 2, description: 'Motor temp exceeded', source_id: '/motor_node'}"

# Query faults
ros2 service call /fault_manager/get_faults ros2_medkit_msgs/srv/GetFaults \
  "{statuses: ['CONFIRMED']}"

# Clear a fault
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT'}"
```

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
- **Persistent storage**: SQLite backend ensures faults survive node restarts
- **Debounce filtering** (optional): AUTOSAR DEM-style counter-based fault confirmation

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `storage_type` | string | `"sqlite"` | Storage backend: `"sqlite"` or `"memory"` |
| `database_path` | string | `"/var/lib/ros2_medkit/faults.db"` | Path to SQLite database file |
| `confirmation_threshold` | int | `-1` | Counter value at which faults are confirmed |
| `healing_enabled` | bool | `false` | Enable automatic healing via PASSED events |
| `healing_threshold` | int | `3` | Counter value at which faults are healed |
| `auto_confirm_after_sec` | double | `0.0` | Auto-confirm PREFAILED faults after timeout (0 = disabled) |

### Storage Backends

**SQLite (default)**: Faults are persisted to disk and survive node restarts. Uses WAL mode for optimal performance.

**Memory**: Faults are stored in memory only. Useful for testing or when persistence is not required.

## Usage

### Launch

```bash
# Default (SQLite storage, immediate confirmation)
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# With custom database path
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p database_path:=/custom/path/faults.db

# With in-memory storage (no persistence)
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p storage_type:=memory
```

## Advanced: Debounce Filtering

For systems that need to filter transient faults, enable debounce filtering by setting a lower `confirmation_threshold`.

### Configuration

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p confirmation_threshold:=-3 \
  -p healing_enabled:=true \
  -p healing_threshold:=3
```

### How It Works

The fault manager uses an AUTOSAR DEM-style debounce model:

- **FAILED events** (fault detected): Decrement the internal counter
- **PASSED events** (fault cleared): Increment the internal counter
- Fault becomes **CONFIRMED** when counter reaches `confirmation_threshold`
- Fault becomes **HEALED** when counter reaches `healing_threshold` (if enabled)

### Fault Lifecycle with Debounce

```
     FAILED events          PASSED events
          |                      |
          v                      v
   [counter--]             [counter++]
          |                      |
          v                      v
PREFAILED -----> CONFIRMED -----> HEALED (retained)
  (counter     (counter <=     (counter >=
   < 0)         threshold)      healing)
                    |
                    v
                CLEARED (manual via ~/clear_fault)
```

### Status Reference

| Status | Description |
|--------|-------------|
| `PREFAILED` | Debounce counter < 0, not yet confirmed |
| `CONFIRMED` | Fault is active and verified |
| `HEALED` | Resolved via PASSED events (if healing enabled) |
| `CLEARED` | Manually acknowledged via `~/clear_fault` |

### Testing Debounce

```bash
# Report FAILED events (need 3 to confirm with threshold=-3)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 0, severity: 2, description: 'Sensor timeout', source_id: '/sensor'}"

# Report PASSED event (fault condition cleared)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 1, severity: 0, description: '', source_id: '/sensor'}"

# Query all statuses including PREFAILED
ros2 service call /fault_manager/get_faults ros2_medkit_msgs/srv/GetFaults \
  "{statuses: ['PREFAILED', 'CONFIRMED', 'HEALED']}"
```

Event types: `0` = EVENT_FAILED, `1` = EVENT_PASSED

### Immediate Confirmation

CRITICAL severity faults bypass debounce and are immediately CONFIRMED, regardless of threshold.

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
