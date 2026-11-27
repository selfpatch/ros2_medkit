# ros2_medkit_policy_bridge

Bridge node that watches a learned robot policy (ACT, diffusion policy, VLA,
...) and reports faults when its predictions disagree or its inference is
slow. The policy publishes
`ros2_medkit_policy_msgs/msg/PolicyIntrospection` once per inference: the
action chunk it predicted, the episode and step, and the inference time.

A robot can be healthy while the policy driving it is not. Without this bridge
that shows up only as a failed task; with it, the fault manager records when
the policy stopped agreeing with itself or got too slow, with the usual
snapshots and recordings.

## Faults

| Code | Raised when | Passed when |
|---|---|---|
| `<PREFIX>_CHUNK_DISAGREEMENT` | the chunk spread is above `spread_threshold` for `raise_after` readings in a row | a new episode starts, or `clear_after` readings within the limit |
| `<PREFIX>_SLOW_INFERENCE` | `inference_ms` is above `max_inference_ms` for `raise_after` readings in a row | the same |

Both checks are off until their threshold is set. The right values depend on
the policy: measure them on successful runs.

**Chunk spread.** A policy that predicts chunks of future actions predicts the
same steps again at its next inference. When it knows what to do the new chunk
agrees with the older ones on the steps they share; when it does not, they
drift apart. This measures disagreement between chunks, not a confidence
score. The spread needs inferences closer than `chunk_length` steps: ACT
with temporal ensembling infers every step, ACT without it infers once per
chunk and cannot be checked this way (the bridge warns once when 10 inferences
in a row share no step, and again only after the spread was computed 10 times
in a row).

**Inference time.** `inference_ms` 0, the message default, means not measured
and is not a reading.

## Delivery

The bridge keeps the state each fault should be in and sends the difference
whenever the FaultManager can be reached:

- at start both codes are reported PASSED once, so a fault left by an earlier
  run of the bridge is passed;
- a raise made while the FaultManager is not running is reported when it starts;
- when a FaultManager appears (its service becomes ready, or a new publisher
  shows up on `/fault_manager/events` after a crash and restart), both codes
  are sent again, so a raise it lost is reported again;
- a fault raised and passed while the FaultManager cannot be reached is not
  reported.

A PASSED heals the fault only with `healing_enabled: true` and
`healing_threshold: 0` in the fault manager. With the packaged
`fault_manager.yaml` (healing off) a passed fault stays CONFIRMED until it is
cleared.

The fault manager keys faults by code: give every bridge its own
`code_prefix`.

## Run

The bridge is not part of `bringup.launch.py`.

```bash
ros2 run ros2_medkit_policy_bridge policy_bridge_node --ros-args --params-file my_policy.yaml
```

Parameters, ranges and defaults: `config/policy_bridge.yaml` and the
[configuration reference](../../docs/config/policy-bridge.rst).

## Code

- `core/policy_monitor.hpp`: the signals and the raise/pass rules, no ROS.
- `core/fault_sync.hpp`: what each fault should be and what was sent, no ROS.
- `policy_bridge_node`: parameters in, messages in, reports out. One thread
  (`main.cpp` runs a single-threaded executor).
