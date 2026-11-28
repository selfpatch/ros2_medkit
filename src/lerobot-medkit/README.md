# lerobot-medkit

Reports what a LeRobot policy does to ros2_medkit, from inside LeRobot: slow
inference, frozen camera frames as the policy received them, and every
predicted action chunk (for `ros2_medkit_policy_bridge`, which turns chunks
that disagree into a fault). Used by `examples/lerobot-so101`. Not on PyPI.

Supported: ROS 2 Jazzy or newer with Python 3.12 (LeRobot 0.7 needs 3.12,
and rclpy and LeRobot must share one interpreter).

## How it hooks in

`attach()` wraps three methods every LeRobot policy (`PreTrainedPolicy`) has
and every LeRobot loop calls (`lerobot-rollout`'s inference engines,
`lerobot-record`, `lerobot-eval`, `predict_action`):

| Method | Means | Checked |
|---|---|---|
| `reset()` | a new episode | |
| `select_action(batch)` | one control step | step time, camera frames |
| `predict_action_chunk(batch)` | one model inference | inference time, chunk published |

The wrappers call the original first and return its result unchanged. The
actions were bit-identical with and without the monitor for the example's ACT
checkpoint (CPU and an RTX 3060 GPU) and for a randomly initialized diffusion
policy (GPU), over two episodes each. Each check runs on its own: an error
inside one is logged once (and counted in a warning at `close()`), and
reaches neither the other checks nor the policy's caller.

In lerobot 0.7, 23 policies call `predict_action_chunk` from `select_action`
(read from their source; only ACT and diffusion were run). Four do not:
SmolVLA, gaussian_actor, lingbot_va and longwam run their model from
`select_action` directly. For them no chunk is published and `measure: chunk`
sees nothing; use `measure: step` with a window of at least `n_action_steps`.
The monitor warns once when `select_action` has run `2 * n_action_steps`
times without reaching `predict_action_chunk`.

lerobot's chunk and RTC inference engines call `predict_action_chunk`
directly, from their own thread, and execute the chunk elsewhere. There the
monitor checks frames and inference time, but cannot tell which step a chunk
belongs to, so it publishes no chunks (one warning). `SyncInferenceEngine`,
`predict_action` and the example go through `select_action` and get
everything.

## Use

```python
import lerobot_medkit

monitor = lerobot_medkit.attach(policy, 'medkit.yaml', postprocess=postprocessor)
...                       # run the policy as usual
monitor.close()
```

A bad configuration (unknown key, wrong type, value out of range) raises
`ValueError` at `attach()`. With an rclpy node of your own, pass `node=`:
reports go through it and you spin it. Without one, the plugin creates a
node (`node_name`) and spins it on a background thread.

With `lerobot-rollout`:

```bash
lerobot-medkit-rollout --medkit.config=medkit.yaml <lerobot-rollout arguments>
```

It attaches the monitor right after `lerobot-rollout` loads the policy, and
publishes chunks through the policy's postprocessor (robot units). Remote
inference has no local policy and is not monitored. Run once against
`lerobot-rollout` 0.7 with the so101-nexus simulated follower
(`--robot.type=sim_so_follower`, sync inference, 60 s): the monitor attached to
the ACT policy, published chunks at steps 0, 50, 100 and 150, and its faults
reached the fault manager. Not yet run on a real robot. `lerobot-rollout`
needs LeRobot's `dataset` extra, and the sim follower its `serial-motors`
extra.

## Faults

| Code | Raised | Passed |
|---|---|---|
| `<PREFIX>_INFERENCE_SLOW` | mean time over `window` readings above `max_ms` | mean below `clear_ratio * max_ms` |
| `<PREFIX>_CAMERA_<NAME>_FROZEN` | `identical_frames` identical frames in a row on `observation.images.<name>` | first frame that differs |

Both checks are off until their threshold is set (`config/medkit.yaml` lists
every key); introspection is on by default. Each check also reports its first
state when it is fine (the first full window, the first frame), so a process
that restarts passes a fault the previous one left raised.

- **Slow inference.** `measure: chunk` times each model inference. `measure:
  step` times each `select_action`, which is what the control loop waits for.
  With a chunked policy most steps only pop a queue: a step mean shorter than
  `n_action_steps` rises and falls with each inference, so the fault comes and
  goes (the monitor warns at `attach()`). Use `chunk` for a slow model, `step`
  with a window of at least `n_action_steps` for a slow loop.
- **Frozen camera.** A camera slower than the control loop repeats frames on
  purpose (15 fps at 30 Hz: 2 in a row): set `identical_frames` well above
  that ratio. Two image keys that map to the same code (`wrist-left`,
  `wrist_left`) are refused with an error; the second is not checked.
- **Cost.** Every frame of every step is copied and compared with the last one.
  Measured on a Ryzen 7 5800H CPU, identical frames (the worst case): +0.36 ms
  per step with 2 cameras at 320x240, +3.2 ms with 3 at 640x480 (float32). On
  an RTX 3060 with 2 cameras at 320x240, the median step took 0.18 ms longer;
  each comparison waits for the device.
- **Several policies.** The fault manager keys faults by code: give each policy
  its own `code_prefix`, `node_name` and `introspection.topic`.

## Transport

`Transport` has four methods: `report_fault`, `clear_fault`,
`publish_introspection` and `close`. The monitor calls them from the control
loop, so a backend must not block.

- `rclpy` (`RclpyTransport`): `ros2_medkit_msgs/srv/ReportFault` to
  `/fault_manager/report_fault`, and `PolicyIntrospection` on
  `/policy/introspection`, stamped with the time the inference started. Plain
  rclpy, so any rmw works (DDS, Zenoh, ...). Reports are queued and sent in
  order, so faults seen while the fault manager is down are reported late, not
  lost:
  - a refused report (`accepted` false) is sent again every `retry_sec`, at
    most `max_attempts` times, then dropped with a warning;
  - a report still unanswered when the service disappears goes to the next
    fault manager. One unanswered for `ack_timeout_sec` (30 s) is sent again:
    if the first copy was only late, the event is counted twice;
  - the queue keeps the newest `max_pending` reports;
  - each problem is logged once when it starts and once when it ends.

  rclpy and the messages come from a sourced ROS 2 workspace, not from pip.
  Without `ros2_medkit_policy_msgs`, no chunks are published (one warning).
- `none`: drops everything.

## Tests

```bash
pip install -e .[test]
pytest tests
```

The rclpy tests need a sourced workspace with `ros2_medkit_msgs` (and
`ros2_medkit_policy_msgs` for the introspection test); without it they skip.
CI (`.github/workflows/lerobot-medkit.yml`, `scripts/ci_lerobot_medkit.sh`)
builds those messages in `ros:jazzy-ros-base`, lints the package and fails if
any test is skipped.
