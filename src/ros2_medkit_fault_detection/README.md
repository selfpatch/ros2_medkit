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
