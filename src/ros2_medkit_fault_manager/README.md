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

## Usage

### Launch

```bash
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py
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
