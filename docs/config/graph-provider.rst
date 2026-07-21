Graph Provider Configuration
==============================

Configuration reference for the ``ros2_medkit_graph_provider`` plugin, which serves
the ``x-medkit-graph`` vendor extension. For the ``/diagnostics`` producer contract,
plugin loading, and a worked example, see :doc:`/tutorials/graph-provider` first -
this page only covers the tunable thresholds.

.. contents:: Table of Contents
   :local:
   :depth: 2

Configuration Keys
-------------------

All keys are set under the ``plugins.graph_provider.`` prefix in
``gateway_params.yaml``, alongside ``plugins.graph_provider.path`` (see
:doc:`/tutorials/plugin-system`):

.. list-table::
   :header-rows: 1
   :widths: 32 12 56

   * - Parameter
     - Default
     - Description
   * - ``expected_frequency_hz_default``
     - ``30.0``
     - Fallback expected frequency (Hz) used to compute an edge's frequency
       ratio when no per-message ``expected_frequency`` was stamped into
       ``/diagnostics`` and no per-function override applies.
   * - ``degraded_frequency_ratio``
     - ``0.5``
     - An edge is ``degraded`` when ``measured_frequency_hz / expected_frequency_hz``
       falls below this ratio.
   * - ``drop_rate_percent_threshold``
     - ``5.0``
     - An edge is ``degraded`` when ``drop_rate_percent`` exceeds this value.
   * - ``freshness_floor_sec``
     - ``5.0``
     - Minimum freshness window in seconds, regardless of expected frequency.
       Keeps slow producers (e.g. a 1 Hz reference monitor) from flapping
       between ``active`` and ``error`` on ordinary sample-to-sample jitter.
   * - ``freshness_headroom_factor``
     - ``3.0``
     - Multiplier applied to the expected message interval when computing the
       freshness window (see formula below).

All five keys are validated on load: ``expected_frequency_hz_default``,
``degraded_frequency_ratio``, ``freshness_headroom_factor``, and
``freshness_floor_sec`` must be strictly positive; ``drop_rate_percent_threshold``
must be non-negative (zero is a valid threshold). An invalid value is rejected,
logged as a warning naming the offending key and value, and the plugin's own
hardcoded default is used instead - it never silently disables degradation or
staleness detection by falling through to a non-positive divisor.

Freshness Window
-----------------

An edge's freshness window - how long a merged sample stays ``active`` before
the edge flips to ``error``/``metrics_stale`` if nothing newer arrives - is
derived from the resolved expected frequency, not a single fixed timeout:

.. code-block:: text

   freshness_window_sec = max(
       freshness_floor_sec,
       freshness_headroom_factor / expected_frequency_hz
   )

A topic expected at 30 Hz with the defaults gets a window of
``max(5.0, 3.0 / 30.0)`` = ``5.0`` seconds (the floor dominates). A topic
overridden to 2 Hz gets ``max(5.0, 3.0 / 2.0)`` = ``5.0`` seconds too, still
floor-dominated; only quite slow expected rates (below ``freshness_headroom_factor
/ freshness_floor_sec`` = 0.6 Hz with the defaults) push the window above the
floor.

Example: raising the floor for a deliberately slow diagnostic producer:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["graph_provider"]
       plugins.graph_provider.freshness_floor_sec: 15.0

Per-Function Overrides
------------------------

Every threshold above can be overridden for a specific Function using
``plugins.graph_provider.function_overrides.<function_id>.<field>``. This lets
different subsystems run different health baselines - a camera pipeline
expected at 30 Hz next to a LiDAR pipeline expected at 10 Hz, in the same
deployment.

.. important::

   The override field names are **not** the same strings as the global default
   keys. Most notably: the global default is
   ``expected_frequency_hz_default``, but the per-function override field is
   ``expected_frequency_hz`` (no ``_default`` suffix). The other four fields
   keep the same name in both places:

   .. list-table::
      :header-rows: 1
      :widths: 50 50

      * - Global default key
        - Per-function override field
      * - ``expected_frequency_hz_default``
        - ``expected_frequency_hz``
      * - ``degraded_frequency_ratio``
        - ``degraded_frequency_ratio``
      * - ``drop_rate_percent_threshold``
        - ``drop_rate_percent_threshold``
      * - ``freshness_headroom_factor``
        - ``freshness_headroom_factor``
      * - ``freshness_floor_sec``
        - ``freshness_floor_sec``

A function with no override for a given field inherits that field from the
resolved global defaults, not from a separate hardcoded fallback. An override
value is validated with the same positive/non-negative rules as its global
counterpart; a rejected override falls back to whatever the function's config
already resolved to (an earlier valid override for the same field, or the
validated global default) - it can never smuggle in a value the global
defaults would themselves reject (e.g. a zero ``expected_frequency_hz``).

Worked Example
~~~~~~~~~~~~~~

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["graph_provider"]
       plugins.graph_provider.expected_frequency_hz_default: 30.0
       plugins.graph_provider.degraded_frequency_ratio: 0.5

       # camera-pipeline Function stays at the 30 Hz global default (no override needed)

       # lidar-pipeline Function: 10 Hz sensor, wider drop-rate tolerance
       plugins.graph_provider.function_overrides.lidar-pipeline.expected_frequency_hz: 10.0
       plugins.graph_provider.function_overrides.lidar-pipeline.drop_rate_percent_threshold: 8.0

       # engine-monitoring Function: slow 2 Hz sensor
       plugins.graph_provider.function_overrides.engine-monitoring.expected_frequency_hz: 2.0

Each dotted ``plugins.graph_provider.function_overrides.<function_id>.<field>``
ROS 2 parameter is delivered to the plugin as a nested JSON object (the
gateway builds nested objects from dotted parameter names for every plugin,
not just this one). For the ``lidar-pipeline`` override above,
``configure()`` receives:

.. code-block:: json

   {
     "expected_frequency_hz_default": 30.0,
     "degraded_frequency_ratio": 0.5,
     "function_overrides": {
       "lidar-pipeline": {
         "expected_frequency_hz": 10.0,
         "drop_rate_percent_threshold": 8.0
       },
       "engine-monitoring": {
         "expected_frequency_hz": 2.0
       }
     }
   }

Only numeric values are accepted for override fields; a non-numeric value
(e.g. a string) or an unrecognized field name is silently ignored rather than
crashing configuration load.

See Also
--------

- :doc:`/tutorials/graph-provider` - Prerequisites, plugin loading, and a worked walkthrough
- :doc:`/api/rest` - Full ``x-medkit-graph`` schema reference
- :doc:`/tutorials/plugin-system` - General plugin configuration mechanism
