# ros2_medkit_fault_reporter

Client library for easy fault reporting with local filtering support.

## Overview

The FaultReporter library provides a simple API for ROS 2 nodes to report faults to the
central `ros2_medkit_fault_manager`. It includes optional local filtering to reduce noise
by only forwarding faults after a configurable threshold is reached within a time window.

## Features

- **Simple API**: `report()` for FAILED events, `report_passed()` for PASSED events
- **Local Filtering** (default ON): Suppress repeated FAILED events until threshold is met
- **Per-fault tracking**: Each fault_code has independent threshold/window tracking
- **Severity bypass**: High-severity faults can bypass filtering
- **PASSED bypass**: PASSED events always bypass local filtering
- **Configurable**: Via ROS 2 parameters

## Usage

### Basic Example

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
      // Report FAILED event - fault condition detected
      reporter_->report("SENSOR_FAILURE",
                        ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                        "Sensor communication timeout");
    } else if (was_sensor_failing()) {
      // Report PASSED event - fault condition cleared
      reporter_->report_passed("SENSOR_FAILURE");
    }
  }

 private:
  std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
};
```

### API Methods

- **`report(fault_code, severity, description)`**: Report a FAILED event (fault detected)
- **`report_passed(fault_code)`**: Report a PASSED event (fault condition cleared)

### Configuration

Parameters can be set via YAML or command line:

```yaml
my_node:
  ros__parameters:
    fault_reporter:
      local_filtering:
        enabled: true
        default_threshold: 3
        default_window_sec: 10.0
        bypass_severity: 2  # ERROR and CRITICAL bypass filtering
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fault_reporter.local_filtering.enabled` | bool | true | Enable/disable local filtering |
| `fault_reporter.local_filtering.default_threshold` | int | 3 | Reports needed before forwarding |
| `fault_reporter.local_filtering.default_window_sec` | double | 10.0 | Time window in seconds |
| `fault_reporter.local_filtering.bypass_severity` | int | 2 | Severity level that bypasses filtering |

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
