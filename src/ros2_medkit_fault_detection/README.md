# ros2_medkit_fault_detection

Shared, protocol-agnostic fault-detection model for medkit gateway plugins.
A single evaluator maps a raw value read from any source (OPC UA, S7, Modbus,
ADS, ...) to the set of faults it implies, so every protocol plugin detects
faults the same way.

## Detection modes

Composable per mapped point in a plugin's node map:

- **threshold** (`ThresholdRule`) - numeric above/below a setpoint. A boolean
  point is alarm-on-true.
- **status word** (`StatusWordRule`) - decode named bits of an integer status
  register; one fault per configured bit.
- **fault enum** (`EnumMapRule`) - map a fault-code register value to a fault
  code + text; one fault per configured code, with an `ok_value` meaning
  "no fault".

## API

```cpp
#include "ros2_medkit_fault_detection/fault_detection.hpp"
namespace fd = ros2_medkit::fault_detection;

// Pure evaluator: value + rule -> set of (active/inactive) fault signals.
std::vector<fd::FaultSignal> signals = fd::evaluate(value, rule);

// Stateful raise/clear edge detector over successive evaluations.
fd::FaultTransitionTracker tracker;
std::vector<fd::FaultSignal> transitions = tracker.apply(signals);  // raises + clears
```

`evaluate` reports the full set of faults a rule governs (each flagged active
or inactive). `FaultTransitionTracker` keeps the last-known state per
`fault_code` and emits only the raise/clear edges, which a plugin forwards to
the fault manager as report/clear.

## Shared fault types

`fault_types.hpp` holds the std-only types that the ROS-free fault reporter core and fault manager core
exchange: `ReportKind`, `FaultReport`, `FaultRecord`, `FaultEventRecord`, and the
constants for severity, fault status and fault event type. They mirror `Fault.msg`, `FaultEvent.msg`
and `ReportFault.srv` in `ros2_medkit_msgs`. Times are `int64_t` nanoseconds since the Unix epoch;
0 means never. The ROS adapters of both packages convert to and from the messages at their edge.

`cmake/fault_reporter_core.cmake` and `cmake/fault_manager_core.cmake` define an INTERFACE target
`ros2_medkit_fault_detection` for `include/` of this package when the ament target is absent. That lets
the cores build with plain CMake. See the READMEs of `ros2_medkit_fault_reporter` and
`ros2_medkit_fault_manager`.

## Placement and packaging

This is a **header-only INTERFACE package**: the detection logic is compiled
directly into each consuming plugin. Protocol plugins are built as MODULE
libraries that resolve gateway symbols from the host process at load time
(`-Wl,--unresolved-symbols=ignore-all`); a separately linked shared library
would not be present in that host, so the logic is compiled in instead. The
header has no ROS or protocol dependencies, which keeps the evaluator a pure,
trivially unit-testable function.

It lives in the open `ros2_medkit` repo so both the open OPC UA plugin
(`ros2_medkit_opcua`) and future protocol plugins can depend on it.

## Consumers

`ros2_medkit_opcua` lowers its `alarm:`, `status_bits:` and `fault_enum:`
node-map blocks onto `DetectionRule` and evaluates them through this module.
