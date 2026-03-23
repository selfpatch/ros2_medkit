# ros2_medkit_fault_reporter

Client library for easy fault reporting to the central FaultManager.

## Overview

The FaultReporter library provides a simple API for ROS 2 nodes to report faults.
Just call `report()` when something goes wrong - the fault is immediately confirmed.

## Quick Start

```cpp
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

class MyNode : public rclcpp::Node {
 public:
  MyNode() : Node("my_node") {
    reporter_ = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(
        shared_from_this(), get_fully_qualified_name());
  }

  void check_sensor() {
    if (sensor_error_detected()) {
      reporter_->report("SENSOR_FAILURE",
                        ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                        "Sensor communication timeout");
    }
  }

 private:
  std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
};
```

That's it! The fault will be immediately confirmed in FaultManager.

## Features

- **Simple API**: Just call `report()` to report a fault
- **Local Filtering** (optional): Suppress repeated faults until threshold is met
- **Per-fault tracking**: Each fault_code has independent filtering
- **Severity bypass**: High-severity faults bypass local filtering

## API

### `report(fault_code, severity, description)`

Report a fault occurrence. With default FaultManager settings, the fault is immediately confirmed.

```cpp
reporter_->report("MOTOR_OVERHEAT",
                  ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                  "Motor temperature exceeded safe limit");
```

### `report_passed(fault_code)` (Advanced)

Report that a fault condition has cleared. Use this when FaultManager is configured with debounce filtering and healing enabled.

```cpp
reporter_->report_passed("MOTOR_OVERHEAT");
```

## Local Filtering

The reporter includes optional local filtering to reduce noise from repeated fault occurrences.
Only forwards the fault to FaultManager after threshold is reached within a time window.

### Configuration

```yaml
my_node:
  ros__parameters:
    fault_reporter:
      local_filtering:
        enabled: true           # Enable local filtering (default: true)
        default_threshold: 3    # Reports needed before forwarding
        default_window_sec: 10.0  # Time window in seconds
        bypass_severity: 2      # ERROR and CRITICAL bypass filtering
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fault_reporter.local_filtering.enabled` | bool | true | Enable/disable local filtering |
| `fault_reporter.local_filtering.default_threshold` | int | 3 | Reports needed before forwarding |
| `fault_reporter.local_filtering.default_window_sec` | double | 10.0 | Time window in seconds |
| `fault_reporter.local_filtering.bypass_severity` | int | 2 | Severity level that bypasses filtering |

## Advanced: Debounce Integration

When FaultManager is configured with debounce filtering (`confirmation_threshold < -1`),
you can use `report_passed()` to signal that a fault condition has cleared:

```cpp
void check_sensor() {
  if (sensor_error_detected()) {
    // Fault condition detected
    reporter_->report("SENSOR_FAILURE",
                      ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                      "Sensor timeout");
  } else if (was_sensor_failing()) {
    // Fault condition cleared - helps with healing
    reporter_->report_passed("SENSOR_FAILURE");
  }
}
```

See [ros2_medkit_fault_manager README](../ros2_medkit_fault_manager/README.md) for debounce configuration details.

## Building

```bash
colcon build --packages-select ros2_medkit_fault_reporter
```

## Testing

```bash
colcon test --packages-select ros2_medkit_fault_reporter
colcon test-result --verbose
```

## License

Apache-2.0
