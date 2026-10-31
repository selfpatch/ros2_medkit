ros2_medkit_fault_detection
===========================

This section contains design documentation for the ros2_medkit_fault_detection package.

Overview
--------

The ``ros2_medkit_fault_detection`` package provides a single, protocol-agnostic
model for turning a raw value read from any source (OPC UA, S7, Modbus, ADS, ...)
into the set of faults that value implies. Every protocol plugin lowers its own
configuration onto this shared model, so a threshold, a status-word bit, or a
fault-code register is mapped to a SOVD fault identically regardless of the
transport it came from.

The package is a header-only INTERFACE library (``cxx_std_17``). Protocol
plugins are built as ``MODULE`` libraries that resolve gateway symbols from the
host process at load time (``--unresolved-symbols=ignore-all``); a separately
linked shared library would not be present in that host, so the detection logic
is compiled directly into each plugin instead.

Detection modes
---------------

A single mapped point carries exactly one of three composable detection rules,
expressed as a ``DetectionRule`` variant:

``ThresholdRule``
    Numeric above/below a setpoint. A boolean point honours the direction, so a
    normally-closed contact or a watchdog-OK bit can alarm on ``false``.

``StatusWordRule``
    Decode named bits of an integer status register, one fault per bit. An
    optional source-register ``width`` masks the raw value to that many low bits
    before decode, so a signed register read whose sign bit is set does not light
    up spurious high bits.

``EnumMapRule``
    Map a fault-code register value to at most one fault code and text. An
    optional catch-all keeps a non-ok value that matches no configured code
    visible rather than reading as healthy.

Evaluator contract
------------------

``evaluate(value, rule)`` is a pure function of its inputs with no ROS or
protocol dependencies, which makes it trivially unit-testable. It returns the
full set of faults the rule governs, each flagged active or inactive, so a
transition tracker can clear a previous code and raise a new one in the same
cycle.

Undecidable input holds state. A value that cannot be coerced for the rule - a
non-finite double (NaN / inf from a disconnected analog sensor or a div-by-zero
on the PLC), a string, or a failed numeric conversion - yields an empty result.
The evaluator emits no signal, so a transition tracker preserves the prior state
instead of clearing a standing fault. A bad read must never mask a real alarm.

Transition tracking and global uniqueness
-----------------------------------------

``FaultTransitionTracker`` layers stateful raise/clear edge detection on top of
the evaluator. It returns only the signals whose active state changed since the
last call, which a plugin forwards to the fault manager as report / clear.

The tracker is keyed by ``fault_code`` alone, matching the fault manager, which
also keys and clears faults by code alone. A single tracker may therefore be
shared across many points only if every ``fault_code`` is globally unique; two
points emitting the same code would alternately raise and clear it each cycle.
Consumers that share one tracker (for example the OPC UA poller across all
node-map entries and event alarms) must enforce that uniqueness at
config-load time and reject a colliding configuration before anything runs.

Raise qualifier
---------------

One active evaluation is enough to raise a fault. A bridge reports a raise as one
``EVENT_FAILED``, so the fault manager's event-counting debounce has nothing to count, and
the filter has to sit where the samples are.

``RaiseQualifier`` sets how long a condition must hold before a rule raises: ``min_samples``
new samples in a row, ``min_duration`` without a break, or both. The default raises on the
first sample.

``RaiseQualifierFilter`` sits between the evaluator and the tracker, so the evaluator
stays pure. While a fault waits, its signal goes out as inactive, and the tracker raises
nothing. Once the qualifier is met the signal passes through until the condition goes away.
Only the raise waits; the first inactive sample clears at once, and the next raise has to
qualify again.

The caller says whether each call carries a new sample (``fresh``). Only fresh samples
count toward ``min_samples``. A call that is not fresh can still end ``min_duration``, so a
consumer that is told about changes only can call again on a timer. A value that could not
be read is unknown, not unchanged, and is not passed at all. An undecidable value, for which
the evaluator returns nothing, keeps the wait as it is, the same way it keeps a raised
fault.

``reset_pending()`` drops the faults that are still waiting, for a consumer that loses
sight of its source. Faults already raised stay raised. The filter has the same
``fault_code`` uniqueness contract as the tracker.
