# ros2_medkit_diagnostic_bridge

Bridge node that converts ROS2 `/diagnostics` messages to FaultManager faults.

## Overview

This package provides backwards compatibility with existing ROS2 diagnostic infrastructure.
It subscribes to the standard `/diagnostics` topic and forwards diagnostic status messages
to the FaultManager as faults.

## Severity Mapping

| DiagnosticStatus Level | Fault Severity | Action |
|------------------------|----------------|--------|
| OK (0) | - | Sends PASSED event (healing) |
| WARN (1) | WARN (1) | Reports fault |
| ERROR (2) | ERROR (2) | Reports fault |
| STALE (3) | CRITICAL (3) | Reports fault |

## Quick Start

```bash
# Start FaultManager first
ros2 run ros2_medkit_fault_manager fault_manager_node

# Start DiagnosticBridge
ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge_node
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `diagnostics_topic` | string | `/diagnostics` | Topic to subscribe to |
| `auto_generate_codes` | bool | `true` | Auto-generate fault codes from diagnostic names |
| `name_to_code.<name>` | string | - | Custom mapping from diagnostic name to fault code |

### Example Configuration

```yaml
diagnostic_bridge:
  ros__parameters:
    diagnostics_topic: "/diagnostics"
    auto_generate_codes: true

    # Custom mappings (optional)
    # Format: "name_to_code.<diagnostic_name>": "<FAULT_CODE>"
    "name_to_code.motor_controller: Temperature": "MOTOR_OVERHEAT"
    "name_to_code.battery_monitor: Voltage": "BATTERY_LOW"
```

### Fault Code Generation

When `auto_generate_codes` is enabled, diagnostic names are converted to fault codes:

| Diagnostic Name | Generated Fault Code |
|-----------------|---------------------|
| `motor temp` | `MOTOR_TEMP` |
| `motor: Status` | `MOTOR_STATUS` |
| `/robot/sensor` | `ROBOT_SENSOR` |

Custom mappings in `name_to_code` take priority over auto-generation.

## Launch

```bash
# Using launch file with default config
ros2 launch ros2_medkit_diagnostic_bridge diagnostic_bridge.launch.py

# With custom config
ros2 launch ros2_medkit_diagnostic_bridge diagnostic_bridge.launch.py \
    config_file:=/path/to/custom_config.yaml
```

## Integration with FaultManager

For full healing support, configure FaultManager with:

```yaml
fault_manager:
  ros__parameters:
    healing_enabled: true
    healing_threshold: 1
```

When a diagnostic transitions from ERROR/STALE to OK, the bridge sends a PASSED event,
allowing the fault to heal in FaultManager.

## Building

```bash
colcon build --packages-select ros2_medkit_diagnostic_bridge
```

## Testing

```bash
colcon test --packages-select ros2_medkit_diagnostic_bridge
colcon test-result --verbose
```

## License

Apache-2.0
