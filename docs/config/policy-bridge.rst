Policy Bridge Configuration
===========================

The ``ros2_medkit_policy_bridge`` node watches a learned robot policy (ACT,
diffusion policy, VLA, ...) through the
``ros2_medkit_policy_msgs/PolicyIntrospection`` messages it publishes, and
reports faults when its predictions disagree or its inference is slow. A robot can be healthy while
the policy driving it is not; without this bridge, that shows up only as a
failed task.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The policy publishes one ``PolicyIntrospection`` per inference: the action
chunk it predicted, the episode and step, and the inference time. A ROS-free
core turns them into two signals:

- **Chunk spread.** A policy that predicts chunks of future actions predicts
  the same steps again at its next inference. When it knows what to do, the new
  chunk agrees with the older ones on the steps they share; when it does not,
  they drift apart. This measures disagreement between chunks, not a
  confidence score. The spread is the mean, over the last ``spread_history``
  chunks, of the RMS difference on the shared steps (at most
  ``spread_horizon``), each joint divided by its ``joint_scale``. Two chunks
  share steps only when the policy infers more often than every
  ``chunk_length`` steps. ACT with temporal ensembling infers every step; ACT
  without it infers every ``n_action_steps`` (the whole chunk) and cannot be
  checked this way. When 10 valid inferences in a row share no step, the bridge
  logs a warning that names the step gap and ``chunk_length``; it warns again
  only after the spread was computed 10 times in a row.
- **Inference time**, from ``inference_ms``. 0 (the message default), a
  negative or a non-finite value means not measured, and is not a reading.

``raise_after`` readings over the limit in a row raise the fault.
``clear_after`` readings within it, or a new episode, pass it. A missing
reading (a dropped message, an unmeasured time) neither counts nor resets a
count. Both checks are off until their threshold is set, because the right
threshold depends on the policy: measure the spread on successful runs and take
a high percentile.

Fault codes are ``<code_prefix>_CHUNK_DISAGREEMENT`` and
``<code_prefix>_SLOW_INFERENCE``. They are reported for the node that publishes
the introspection topic, unless ``source_id`` names another entity. The bridge
waits up to 5 s for discovery to name that node, then reports under the topic
name.

The fault manager keys faults by code, so every bridge needs its own
``code_prefix``: two bridges with the same prefix share their faults, and the
first one to pass a fault passes it for both.

Delivery to the FaultManager
----------------------------

The bridge keeps the state each fault should be in and sends the difference
whenever the FaultManager service can be reached, checked every
``retry_period_sec``:

- At start it reports PASSED for both codes once, so a fault left by an
  earlier run of the bridge is passed (it heals with the settings below).
- A fault raised while the FaultManager is not running is reported when it
  starts. When a raise has waited 10 s, the bridge logs one warning, and one
  info line when the FaultManager is reachable again.
- When a FaultManager appears, the current state of both codes is sent again,
  so a raise a restarted FaultManager lost is reported again. A new
  FaultManager is seen by its service becoming ready, or by a new publisher on
  ``/fault_manager/events``: a FaultManager killed and started again at once
  keeps looking ready until the dead one's DDS lease ends. Measured with a
  SIGKILL and an immediate restart (memory storage): the raise was back
  2.8 s later. With persistent storage the fault is still CONFIRMED there, and
  the repeat only moves its ``last_occurred``.
- A fault that is raised and passed while the FaultManager cannot be reached
  is not reported.
- A fault cleared over REST while the chunks still disagree is raised again
  only after it has passed.
- A report the FaultManager client gives up on (no answer after its
  ``fault_reporter.delivery`` attempts, while the service still looks ready)
  is not sent again until the state changes or a new FaultManager appears.

A PASSED heals the fault only when the FaultManager heals on a single PASSED
event: ``healing_enabled: true`` and ``healing_threshold: 0``. With the
packaged ``fault_manager.yaml`` (healing off) a passed fault stays CONFIRMED
until it is cleared.

The bridge declares ``fault_reporter.local_filtering.enabled: false``: it sends
one report per change of state, which the reporter's local occurrence filter
would hold back. A value set for it is kept.

Parameters
----------

Every parameter below accepts an integer where a number is expected. A value
of the wrong type or out of range is named in a warning and replaced by its
default or clamped; the node keeps running. An ``introspection_topic`` that is
not a valid topic name stops the node with an error. The ``fault_reporter.*``
parameters belong to the fault reporter library and keep its own checks.

.. code-block:: yaml

   policy_bridge:
     ros__parameters:
       introspection_topic: "/policy/introspection"
       source_id: ""
       code_prefix: "POLICY"
       spread_threshold: 0.0       # 0 = off
       spread_history: 4
       spread_horizon: 20
       # joint_scale: [0.3, 0.5, 0.4]
       max_inference_ms: 0.0       # 0 = off
       raise_after: 3
       clear_after: 0
       clear_on_new_episode: true
       chunk_disagreement_severity: 1  # SEVERITY_WARN
       slow_inference_severity: 1
       retry_period_sec: 0.1

.. list-table::
   :header-rows: 1
   :widths: 30 14 14 42

   * - Parameter
     - Default
     - Range
     - Description
   * - ``introspection_topic``
     - ``/policy/introspection``
     - topic name
     - Topic the policy publishes ``PolicyIntrospection`` on.
   * - ``source_id``
     - ``""``
     -
     - Entity the faults are reported for. Empty: the node that publishes the
       introspection topic.
   * - ``code_prefix``
     - ``POLICY``
     -
     - Prefix of the two fault codes. One per watched policy.
   * - ``spread_threshold``
     - ``0.0``
     - >= 0
     - Chunk spread above which a reading counts as disagreeing. ``0`` turns the
       check off; a negative value turns it off with a warning.
   * - ``spread_history``
     - ``4``
     - 1 to 64
     - Older chunks each new chunk is compared with.
   * - ``spread_horizon``
     - ``20``
     - 1 to 10000
     - Shared steps compared per pair of chunks, at most.
   * - ``joint_scale``
     - unset (``1`` per joint)
     - each > 0
     - Per-joint divisor, for example each joint's spread in the training data.
       A value that is not positive makes every joint use 1. A list of another
       length than ``action_dim`` is warned once; joints past its end use 1.
   * - ``max_inference_ms``
     - ``0.0``
     - >= 0
     - Inference time above which a reading counts as slow. ``0`` turns the
       check off.
   * - ``raise_after``
     - ``3``
     - 1 to 100000
     - Readings over the limit in a row before the fault is raised.
   * - ``clear_after``
     - ``0``
     - 0 to 100000
     - Readings within the limit in a row before the fault is passed. ``0``:
       only a new episode passes it. With ``clear_on_new_episode: false`` too, a
       raised fault is never passed, and the bridge warns at start.
   * - ``clear_on_new_episode``
     - ``true``
     -
     - Pass both faults when the episode number changes.
   * - ``chunk_disagreement_severity``, ``slow_inference_severity``
     - ``1`` (WARN)
     - 0 to 3
     - Severity of each fault: 0 INFO, 1 WARN, 2 ERROR, 3 CRITICAL.
   * - ``retry_period_sec``
     - ``0.1``
     - 0.01 to 10
     - How often the bridge checks the FaultManager service and sends what is
       due.

Messages the bridge does not use
--------------------------------

A message is not used, and logged with the reason, when ``chunk_length`` or
``action_dim`` is 0, when ``chunk`` does not hold ``chunk_length x action_dim``
values, when that is more than 65536 values (such a chunk is not copied), when
the chunk holds NaN or inf, or when its step repeats the last one. The first
message of each kind is logged with its reason; later ones of the same kind are
counted, and the count is logged at most once a minute. The inference time of
a message with a bad chunk is still a reading.

At the limits (``spread_history`` 64, chunks of 65536 values) the history holds
64 x 256 KiB = 16 MiB, and each message is compared with 64 chunks on the
executor thread.

Run it
------

The bridge is not started by ``bringup.launch.py``. Start it with a parameter
file that sets at least one threshold:

.. code-block:: bash

   ros2 run ros2_medkit_policy_bridge policy_bridge_node --ros-args --params-file my_policy.yaml

``policy_bridge.launch.py`` starts it with the packaged ``policy_bridge.yaml``,
where both checks are off.
