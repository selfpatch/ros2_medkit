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

### Using with a lifecycle node

`FaultReporter` also works inside an `rclcpp_lifecycle::LifecycleNode` — construct it from the node
directly (no `shared_from_this()` needed), typically in `on_configure()`:

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  MyLifecycleNode() : LifecycleNode("my_node") {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    reporter_ = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(
        *this, get_fully_qualified_name());
    return CallbackReturn::SUCCESS;
  }

 private:
  std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
};
```

### Constructors

`FaultReporter` can be built from whichever handle you have:

- `FaultReporter(rclcpp::Node & node, source_id[, service_name])`
- `FaultReporter(rclcpp::Node::SharedPtr node, source_id[, service_name])`
- `FaultReporter(rclcpp_lifecycle::LifecycleNode & node, source_id[, service_name])`
- `FaultReporter(rclcpp_lifecycle::LifecycleNode::SharedPtr node, source_id[, service_name])`
- `FaultReporter(node_base, node_graph, node_services, node_params, logger, source_id[, service_name])`
  — the interface-based constructor the others delegate to, for custom wiring.

## Features

- **Simple API**: Just call `report()` to report a fault
- **Lifecycle node support**: Construct from a regular `Node` or an `rclcpp_lifecycle::LifecycleNode`
- **Local Filtering** (optional): Suppress repeated faults until threshold is met
- **Per-fault tracking**: Each fault_code has independent filtering
- **Severity bypass**: High-severity faults bypass local filtering
- **ROS-free core**: `ReporterCore` (filtering and report building) builds without ROS, see below

## API

### `report(fault_code, severity, description)`

Report a fault occurrence. With default FaultManager settings, the fault is immediately confirmed.

```cpp
reporter_->report("MOTOR_OVERHEAT",
                  ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                  "Motor temperature exceeded safe limit");
```

### `report_passed(fault_code)` (Advanced)

Report that a fault condition has cleared. PASSED events are filtered by the same threshold and window as FAILED events. Use this when FaultManager is configured with debounce filtering and healing enabled.

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

A `default_threshold` above `INT_MAX` or a `bypass_severity` outside `[0, 255]` is replaced by its
default and logged as a warning. A `default_threshold` below 1 becomes 1. A `default_window_sec` that
is NaN, infinite or not positive becomes 1.0. These last two are not logged.

## ROS-free core

`ros2_medkit_fault_reporter::core::ReporterCore` holds the filter and builds a
`ros2_medkit::fault_detection::FaultReport` (types from `ros2_medkit_fault_detection`). It hands each
report that passed the filter to a sink function. `FaultReporter` is the ROS adapter: its sink sends
the `ReportFault` request. The CMake target `fault_reporter_core` is defined in
`cmake/fault_reporter_core.cmake`, which uses plain CMake (3.16, C++17). The core compiles and is
tested without ROS, with no DDS setup. `scripts/check_cores_without_ros/run.sh` builds it on
Ubuntu 20.04 with gcc 9, the oldest compiler the cores are checked with.

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
