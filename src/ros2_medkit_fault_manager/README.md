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
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED']}"

# Clear a fault
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT'}"
```

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/report_fault` | `ros2_medkit_msgs/srv/ReportFault` | Report a fault occurrence |
| `~/list_faults` | `ros2_medkit_msgs/srv/ListFaults` | Query faults with filtering |
| `~/clear_fault` | `ros2_medkit_msgs/srv/ClearFault` | Clear/acknowledge a fault |
| `~/get_snapshots` | `ros2_medkit_msgs/srv/GetSnapshots` | Get topic snapshots for a fault |

## Features

- **Multi-source aggregation**: Same `fault_code` from different sources creates a single fault
- **Occurrence tracking**: Counts total reports and tracks all reporting sources
- **Severity escalation**: Fault severity is updated if a higher severity is reported
- **Persistent storage**: SQLite backend ensures faults survive node restarts
- **Debounce filtering** (optional): AUTOSAR DEM-style counter-based fault confirmation
- **Snapshot capture**: Captures topic data when faults are confirmed for debugging (snapshots are deleted when fault is cleared)
- **Fault correlation** (optional): Root cause analysis with symptom muting and auto-clear

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `storage_type` | string | `"sqlite"` | Storage backend: `"sqlite"` or `"memory"` |
| `database_path` | string | `"/var/lib/ros2_medkit/faults.db"` | Path to SQLite database file |
| `confirmation_threshold` | int | `-1` | Counter value at which faults are confirmed |
| `healing_enabled` | bool | `false` | Enable automatic healing via PASSED events |
| `healing_threshold` | int | `3` | Counter value at which faults are healed |
| `auto_confirm_after_sec` | double | `0.0` | Auto-confirm PREFAILED faults after timeout (0 = disabled) |

### Snapshot Parameters

Snapshots capture topic data when faults are confirmed for post-mortem debugging.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `snapshots.enabled` | bool | `true` | Enable/disable snapshot capture |
| `snapshots.background_capture` | bool | `false` | Use background subscriptions (caches latest message) vs on-demand capture |
| `snapshots.timeout_sec` | double | `1.0` | Timeout waiting for topic message (on-demand mode) |
| `snapshots.max_message_size` | int | `65536` | Maximum message size in bytes (larger messages skipped) |
| `snapshots.default_topics` | string[] | `[]` | Topics to capture for all faults |
| `snapshots.config_file` | string | `""` | Path to YAML config for `fault_specific` and `patterns` |

**Topic Resolution Priority:**
1. `fault_specific` - Exact match for fault code (configured via YAML config file)
2. `patterns` - Regex pattern match (configured via YAML config file)
3. `default_topics` - Fallback for all faults

**Example YAML config file** (`snapshots.yaml`):
```yaml
fault_specific:
  MOTOR_OVERHEAT:
    - /joint_states
    - /motor/temperature
patterns:
  "MOTOR_.*":
    - /joint_states
    - /cmd_vel
```

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
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['PREFAILED', 'CONFIRMED', 'HEALED']}"
```

Event types: `0` = EVENT_FAILED, `1` = EVENT_PASSED

### Immediate Confirmation

CRITICAL severity faults bypass debounce and are immediately CONFIRMED, regardless of threshold.

## Advanced: Fault Correlation

Fault correlation reduces noise by identifying relationships between faults. When enabled, symptom faults
(effects of a root cause) can be muted and auto-cleared when the root cause is resolved.

### Correlation Modes

**Hierarchical**: Defines explicit root cause → symptoms relationships. When a root cause fault occurs,
subsequent matching symptom faults within a time window are correlated and optionally muted.

**Auto-Cluster**: Automatically groups related faults that match a pattern within a time window.
Useful for detecting "storms" of related faults (e.g., communication errors).

### Configuration

Enable correlation by providing a YAML configuration file:

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p correlation.config_file:=/path/to/correlation.yaml
```

### Correlation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `correlation.config_file` | string | `""` | Path to correlation YAML config (empty = disabled) |
| `correlation.cleanup_interval_sec` | double | `5.0` | Interval for cleaning up expired pending correlations (seconds) |

### Configuration File Format

```yaml
correlation:
  enabled: true
  default_window_ms: 500  # Default time window for symptom detection

  # Reusable fault patterns (supports wildcards with *)
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    drive_faults:
      codes: ["DRIVE_*"]
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]

  rules:
    # Hierarchical rule: E-Stop causes motor and drive faults
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001", "ESTOP_002"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000           # Symptoms within 1s of root cause
      mute_symptoms: true       # Don't publish symptom events
      auto_clear_with_root: true # Clear symptoms when root cause clears

    # Auto-cluster rule: Group communication errors
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3              # Need 3 faults to form cluster
      window_ms: 500            # Within 500ms
      show_as_single: true      # Only show representative fault
      representative: highest_severity  # first | most_recent | highest_severity
```

### Pattern Wildcards

Patterns support `*` wildcard matching:
- `MOTOR_*` matches `MOTOR_COMM`, `MOTOR_TIMEOUT`, `MOTOR_DRIVE_FAULT`
- `*_COMM_*` matches `MOTOR_COMM_FL`, `SENSOR_COMM_TIMEOUT`
- `*_TIMEOUT` matches `MOTOR_TIMEOUT`, `SENSOR_TIMEOUT`

### Querying Correlation Data

Use `include_muted` and `include_clusters` to retrieve correlation information:

```bash
# Get faults with muted fault details
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED'], include_muted: true, include_clusters: true}"
```

Response includes:
- `muted_count`: Number of muted symptom faults
- `cluster_count`: Number of active fault clusters
- `muted_faults[]`: Details of muted faults (when `include_muted=true`)
- `clusters[]`: Details of active clusters (when `include_clusters=true`)

### REST API (via Gateway)

Query parameters for GET `/api/v1/faults`:
- `include_muted=true`: Include muted fault details in response
- `include_clusters=true`: Include cluster details in response

Response fields:
```json
{
  "faults": [...],
  "count": 5,
  "muted_count": 2,
  "cluster_count": 1,
  "muted_faults": [
    {
      "fault_code": "MOTOR_COMM_FL",
      "root_cause_code": "ESTOP_001",
      "rule_id": "estop_cascade",
      "delay_ms": 50
    }
  ],
  "clusters": [
    {
      "cluster_id": "comm_storm_1",
      "rule_id": "comm_storm",
      "rule_name": "Communication Storm",
      "representative_code": "SENSOR_TIMEOUT",
      "representative_severity": "CRITICAL",
      "fault_codes": ["MOTOR_COMM_FL", "SENSOR_TIMEOUT", "DRIVE_COMM_ERR"],
      "count": 3,
      "first_at": 1705678901.123,
      "last_at": 1705678901.456
    }
  ]
}
```

When clearing a root cause fault, `auto_cleared_codes` lists symptoms that were auto-cleared:
```json
{
  "status": "success",
  "fault_code": "ESTOP_001",
  "message": "Fault cleared",
  "auto_cleared_codes": ["MOTOR_COMM_FL", "MOTOR_COMM_FR", "DRIVE_FAULT"]
}
```

### Example: E-Stop Cascade

1. E-Stop is triggered → `ESTOP_001` fault reported
2. Motors lose power → `MOTOR_COMM_FL`, `MOTOR_COMM_FR` faults reported
3. Correlation engine detects motor faults are symptoms of E-Stop
4. Motor faults are muted (not published as events, but stored)
5. Dashboard shows only `ESTOP_001` (root cause)
6. When E-Stop is cleared → Motor faults are auto-cleared

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
