^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-22)
------------------
* Initial release: shared, protocol-agnostic fault-detection model for medkit
  gateway plugins. A single header-only evaluator maps a raw value read from any
  source (OPC UA, S7, Modbus, ADS, ...) into the set of faults it implies, using
  one of three composable detection modes: ``ThresholdRule`` (numeric
  above/below a setpoint), ``StatusWordRule`` (decode named bits of an integer
  status register, with optional source-width masking to drop sign-extended high
  bits), and ``EnumMapRule`` (map a fault-code register value to a fault code +
  text, with an optional catch-all for unmapped values)
  (`#481 <https://github.com/selfpatch/ros2_medkit/issues/481>`_).
* ``evaluate(value, rule)`` is a pure function with no ROS / protocol
  dependencies, so it is trivially unit-testable and safe to compile into a
  dlopen-loaded plugin MODULE. Undecidable input (a non-finite double, a string,
  a failed numeric conversion) yields an empty result so a transition tracker
  holds the prior state instead of clearing a standing fault - a bad read never
  masks a real alarm.
* ``FaultTransitionTracker`` layers stateful raise/clear edge detection on top,
  keyed by ``fault_code`` alone; consumers that share one tracker across many
  points must enforce global fault-code uniqueness at config-load time.
* Shipped as a header-only INTERFACE library (``cxx_std_17``); the ``OPC UA``
  plugin is the first consumer and migrates its threshold / status-bit / enum
  detection onto this module.
* Contributors: @mfaferek93, @bburda
