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
   * - ``stale_grace_sec``
     - ``2.0``
     - Extra seconds an edge may remain outside its freshness window before
       ``metrics_status`` flips to ``error``/``metrics_stale``. Absorbs a single
       late DDS/executor-jitter ``/diagnostics`` sample so ``pipeline_status``
       does not flap to ``broken`` and back on the very next on-time sample.
       ``0.0`` disables the grace (report stale exactly at the window
       boundary - today's behavior before this option existed). See
       :ref:`graph-provider-stale-grace` below.
   * - ``multi_publisher_rate``
     - ``"annotate"``
     - Policy for edges whose DATA topic has more than one live publisher:
       ``"annotate"`` always shows the measured rate and flags the ambiguity;
       ``"suppress"`` hides the (untrustworthy) rate instead. See
       :ref:`graph-provider-multi-publisher-rate` below.

All seven keys are validated on load: ``expected_frequency_hz_default``,
``degraded_frequency_ratio``, ``freshness_headroom_factor``, and
``freshness_floor_sec`` must be strictly positive; ``drop_rate_percent_threshold``
and ``stale_grace_sec`` must be non-negative (zero is a valid value for both -
a zero threshold and a zero grace are both meaningful, not "disabled"); and
``multi_publisher_rate`` must be exactly ``"annotate"`` or ``"suppress"``. An
invalid value is rejected, logged as a warning naming the offending key and
value, and the plugin's own hardcoded default is used instead - it never
silently disables degradation or staleness detection by falling through to a
non-positive divisor.

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

.. _graph-provider-stale-grace:

Stale-Grace Debounce
----------------------

An edge is not reported ``metrics_stale`` the instant its age crosses the
freshness window above - it must stay outside that window continuously for
more than ``stale_grace_sec`` before ``metrics_status`` flips to ``error``:

.. code-block:: text

   metrics_stale iff age_sec > freshness_window_sec + stale_grace_sec

This is a stateless function of the sample's age, the resolved freshness
window, and the grace period - there is no separate onset/tick state to
maintain, so it always uses whichever window the edge actually resolved (its
own per-function config, never a different function's or the global default).
The default, ``2.0`` seconds, absorbs ordinary DDS/executor jitter; raise it
for a noisier network or executor, or set it to ``0.0`` for immediate,
point-in-time detection (the plugin's original behavior before this option
existed). ``stale_grace_sec`` can be overridden per Function like every other
threshold on this page.

.. _graph-provider-multi-publisher-rate:

Multi-Publisher Rate Policy
------------------------------

``frequency_hz`` (mapped from the producer's ``frame_rate_msg``/``frame_rate_node``,
see :doc:`/tutorials/graph-provider`) is a **topic-level arrival rate measured on
the subscriber side of the ``/diagnostics`` producer, summed across every live
publisher on that topic name** - the graph provider has no way to attribute an
incoming sample to one specific publisher. If a second, leftover, or duplicate
publisher ends up on the same topic, its messages inflate the summed rate, and a
genuinely slow or stalled primary publisher can read as healthy.

To make that ambiguity visible, every edge whose DATA topic (queried live from
the ROS graph, independent of and in addition to ``/diagnostics``) resolves more
than one live publisher always gets ``metrics.publisher_count`` (the live count)
and ``metrics.rate_ambiguous: true`` in its response, regardless of policy - see
:doc:`/api/rest`'s field notes. ``multi_publisher_rate`` controls what happens to
``frequency_hz`` itself once that is true:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Value
     - Behavior
   * - ``"annotate"`` (default)
     - Keep showing the measured ``frequency_hz`` and let it drive the degraded
       ratio as usual; ``publisher_count``/``rate_ambiguous`` warn the operator
       that the number may be inflated, but the plugin still trusts it.
   * - ``"suppress"``
     - Omit ``frequency_hz`` entirely and never let it drive a ``degraded``
       verdict for this edge. Freshness/``metrics_status`` is unaffected -
       suppression is only about the rate number, never about whether the
       edge is stale.

Choose ``"annotate"`` when you want visibility without losing data. Choose
``"suppress"`` for a safety-critical deployment that would rather show no rate
than an untrustworthy one when more than one publisher is present. Suppression
removes the misleading number, not the edge's health verdict: ``rate_ambiguous``
flags the ambiguity and ``pipeline_status`` is unchanged. ``multi_publisher_rate``
can be overridden per Function like every other setting on this page.

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
   ``expected_frequency_hz`` (no ``_default`` suffix). The other six fields
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
      * - ``stale_grace_sec``
        - ``stale_grace_sec``
      * - ``multi_publisher_rate``
        - ``multi_publisher_rate``

A function with no override for a given field inherits that field from the
resolved global defaults, not from a separate hardcoded fallback. A numeric
override value is validated with the same positive/non-negative rules as its
global counterpart, and a ``multi_publisher_rate`` override is validated
against the same ``{"annotate", "suppress"}`` enum as its global counterpart; a
rejected override falls back to whatever the function's config already
resolved to (an earlier valid override for the same field, or the validated
global default) - it can never smuggle in a value the global defaults would
themselves reject (e.g. a zero ``expected_frequency_hz``).

.. warning::

   **Per-function overrides only take effect via a params YAML loaded with**
   ``config_file``. Two other ways an operator might expect to set one either
   silently do nothing or fail outright:

   - A bare ``ros2 launch ros2_medkit_gateway gateway.launch.py
     plugins.graph_provider.function_overrides.<function_id>.<field>:=value``
     is dropped without warning: ``gateway.launch.py`` only forwards a fixed
     set of launch arguments into the node's parameters (``server_host``,
     ``server_port``, ``refresh_interval_ms``, ``cors_allowed_origins``,
     ``config_file``) - there is no passthrough for arbitrary plugin
     parameters.
   - ``ros2 run ... --ros-args -p
     plugins.graph_provider.function_overrides.<function_id>.<field>:=value``
     fails to parse whenever ``<function_id>`` contains a hyphen (e.g.
     ``lidar-pipeline``, ``engine-monitoring``) - rcl's parameter-value lexer
     rejects the hyphen there, and the command errors out before the gateway
     starts.

   The only reliable path is a YAML params file with the flat, dotted keys
   (see the worked example below) loaded via ``config_file``:

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py \
        config_file:=/path/to/graph-provider-overrides.yaml

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

Only numeric values are accepted for the six numeric override fields, and only
a string value for ``multi_publisher_rate``. A non-numeric value for a numeric
field, a non-string value for ``multi_publisher_rate``, or an unrecognized
field name never crashes configuration load, but it also no longer applies
silently: each case logs a warning naming the offending function, field, and
value, and the override is skipped in favor of whatever that function's config
already resolved to.

See Also
--------

- :doc:`/tutorials/graph-provider` - Prerequisites, plugin loading, and a worked walkthrough
- :doc:`/api/rest` - Full ``x-medkit-graph`` schema reference
- :doc:`/tutorials/plugin-system` - General plugin configuration mechanism
