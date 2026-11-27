# LeRobot SO-101 example

A simulated SO-101 arm runs a LeRobot ACT policy over ROS 2: it picks up a cube
and lifts it. ros2_medkit, with **stock packages and this example's
configuration**, watches the robot and the policy and reports what goes wrong
through its REST API.

Without `src/lerobot-medkit`, everything here is plain ROS 2 plus stock
ros2_medkit. The Dockerfile installs the plugin, and the policy node then also
reports from inside LeRobot: see "With lerobot-medkit".

```
 so101_sim (MuJoCo)  --images, joint states-->  lerobot_policy (ACT)
        ^                                              |
        +------------------ joint command -------------+
 camera_monitor --/diagnostics-->  diagnostic_bridge --+
 lerobot_policy --/diagnostics (no plugin)-->          +--> fault_manager --> gateway (REST, :8080)
 lerobot_policy --ReportFault (lerobot-medkit)-------->+        |              + graph_watchdog plugin
 episode_runner --ReportFault (EPISODE_FAILED)-------->+        |
 lerobot_policy --/policy/introspection--> policy_bridge (optional) --> fault_manager
                                                       rosbag of each failed attempt
```

## What is learned and what is scripted

- **Learned:** the policy that drives the arm, ACT trained with LeRobot on 200 demonstrations.
  It lifts the cube in about a third of the attempts (see "The checkpoint"): failed
  attempts are part of what the example shows, not something it hides.
- **Scripted:** the demonstrations. `training/expert.py` reads the cube pose from the
  simulator and uses inverse kinematics, recomputed every step from the measured joints.
  While recording, noise is added to the command the arm executes, and the dataset keeps
  the expert's correction, so the demonstrations include recoveries. It only generates
  training data and never runs in the example.
- **Injected:** every fault in the scenarios below is injected on purpose. The example shows
  that each one is detected and reported; it does not show how often such faults happen.

## Faults

| Fault code | Raised by | When |
|---|---|---|
| `EPISODE_FAILED` | `episode_runner`, `ReportFault` service | An attempt ends without the cube lifted 5 cm. Saves a rosbag of the attempt. |
| `CAMERA_WRIST_FROZEN`, `CAMERA_OVERHEAD_FROZEN` | `camera_monitor` on `/diagnostics`, via `diagnostic_bridge` | 10 identical frames in a row |
| `POLICY_INFERENCE_SLOW` | with lerobot-medkit: the plugin, through `ReportFault`; without: `lerobot_policy` on `/diagnostics`, via `diagnostic_bridge` | with lerobot-medkit: two model inferences in a row above 300 ms; without: mean step time over 10 steps above 100 ms |
| `POLICY_CAMERA_WRIST_FROZEN`, `POLICY_CAMERA_OVERHEAD_FROZEN` | lerobot-medkit only, through `ReportFault` | 10 identical frames in a row, as the policy received them |
| `GRAPH_NODE_DISAPPEARED` | gateway `graph_watchdog` plugin (`node_death`) | The policy process dies |
| `POLICY_CHUNK_DISAGREEMENT` | optional `policy_bridge`, from `/policy/introspection` | Successive action chunks disagree (see "Optional: policy bridge") |

Measured on a laptop (AMD Ryzen 7 5800H, no GPU in the container), one run each:

| Scenario | Raised | Healed |
|---|---|---|
| Overhead camera frozen from step 40 | 10 frames after the freeze; in the fault manager 19 ms after the monitor saw it | 4 s after `restore-normal.sh` |
| +500 ms per policy step | 3.2 s after the injection | 6.9 s after `restore-normal.sh` |
| Policy killed (`SIGKILL`) mid-attempt | 5.9 s after the kill | 11.2 s after the kill (respawned node back in the graph) |

A clean campaign (no injection) raised no camera or inference fault. The failed attempt
with a frozen camera kept one recording covering the whole attempt (104.6 s attempt,
134.5 s recording starting 29 s before it).

The graph watchdog runs only its `node_death` detector here
(`config/gateway.yaml`): the scripts change node parameters on purpose, which
`param_drift` would report. The container also uses a 3 s DDS liveliness lease
(`config/fastdds.xml`). With Fast DDS's default of 20 s, a process killed with `SIGKILL`
stays in the graph until the launch file has already restarted it, and the watchdog
never sees it gone. A real robot needs the same setting to report a crashed process
quickly.

The camera and inference faults are configured never to confirm
(`config/fault_thresholds.yaml`): a confirmation in the middle of an attempt would start
its own recording and cut the start off the `EPISODE_FAILED` recording of the same
attempt. They are still reported and listed (status `PREFAILED`), and they heal on the
first OK. That is why the nodes publish a failing status only once, when it starts, and
the bridge's local filter is off (`config/diagnostic_bridge.yaml`).

`GRAPH_NODE_DISAPPEARED` does confirm and records its own rosbag. The attempt it hits is
aborted, not reported as `EPISODE_FAILED`, so no attempt recording is cut. It heals when
the respawned policy node is back in the graph.

`EPISODE_FAILED` heals at the start of every attempt, so each failed attempt has its own
recording. Its `occurrence_count` stays 1: count failed attempts in
`results/attempts.jsonl` or the recordings list, not in that field.

### With lerobot-medkit

`lerobot_policy` attaches `lerobot_medkit` (`src/lerobot-medkit`,
`config/lerobot_medkit.yaml`) to the ACT policy. The plugin wraps the policy's
`reset`, `select_action` and `predict_action_chunk`, the calls LeRobot itself
makes, so the checks run on what the policy actually got and did, not on the
ROS topics around it. It then replaces the node's `/diagnostics` path:

- `POLICY_INFERENCE_SLOW`: timed on each model inference
  (`predict_action_chunk`, `measure: chunk`), raised when two in a row take
  more than 300 ms, and reported straight to the fault manager. The
  `/diagnostics` path times whole steps instead (preprocessing, `select_action`,
  postprocessing; 100 ms mean over 10 steps). The policy re-plans every 50
  steps, so a mean over steps rises and falls with each inference: the plugin
  warns when its window is shorter than that. With the plugin
  `inference_delay_ms` sleeps inside each inference.
- `POLICY_CAMERA_<NAME>_FROZEN`: identical frames in the batch the policy
  received. `camera_monitor` still watches the image topics. Both faults
  appear for a frozen camera, one for each side of the policy.
- The action chunks on `/policy/introspection` for the policy bridge below.

Set the node parameter `use_lerobot_medkit: false`, or leave the plugin out of
the image, to get the plain ROS 2 behaviour back.

Measured in the container (plugin and policy bridge built in, CPU), one run each:

| Scenario | What the fault manager showed |
|---|---|
| 2 clean attempts | no fault |
| +500 ms per inference | `POLICY_INFERENCE_SLOW` 15.6 s after the injection (the second slow inference); healed 32 s after `restore-normal.sh` |
| Overhead camera frozen from step 40 | `POLICY_CAMERA_OVERHEAD_FROZEN` together with `CAMERA_OVERHEAD_FROZEN` |
| Policy killed while that fault was raised, camera restored | the respawned process passed it on its first frame, 9 s after the respawn |
| `lerobot_medkit_config` pointing to a missing file | an error naming the file, then the `/diagnostics` path |

One ACT inference took 79-90 ms on this CPU. An earlier campaign of 10 clean
attempts, with the slow check timing steps instead of inferences, raised no
plugin fault. Neither the plugin's faults nor the bridge's confirm mid-attempt,
so the `EPISODE_FAILED` recording still covers the whole attempt.

### Optional: policy bridge

When `ros2_medkit_policy_bridge` is in the build (`src/ros2_medkit_policy_bridge`; the
Dockerfile picks it up if present), the launch file also starts it, and `lerobot_policy`
publishes each predicted action chunk on `/policy/introspection`. The bridge raises
`POLICY_CHUNK_DISAGREEMENT` when a new chunk disagrees with the older ones on the steps
they share (`config/policy_bridge.yaml`; turn it off with `policy_bridge:=false`). Like the
camera faults it never confirms, and it heals when the next attempt starts.

The threshold (0.1 rad, twice in a row) comes from 30 unseen seeds and 15 runs with a
frozen overhead camera, on the host GPU:

| Runs | Flagged | Median step |
|---|---|---|
| Successful, clean | 0 of 11 | |
| Failed, clean | 13 of 19 | 150 |
| Overhead camera frozen from step 40 | 14 of 15 | 150 |

`EPISODE_FAILED` comes at step 300. Successful attempts end by about step 160, after 2 or
3 readings, so 0 of 11 is a weak bound on false alarms. In the container (CPU): a clean
campaign of 6 attempts flagged 1 of 3 failures (seed 100002, flagged on the host too)
and none of the 3 successes; with the frozen camera, 6 of 6 attempts were flagged. This
fault is about the policy, not the robot: a frozen camera also raises
`CAMERA_OVERHEAD_FROZEN`, but a failure with healthy sensors raises only this one.

## Run it

You need Docker and a trained checkpoint in `policy/` (next section).

```bash
cd examples/lerobot-so101
docker compose up --build          # gateway on http://localhost:8080
./scripts/campaign.sh 10           # 10 attempts, appended to results/attempts.jsonl
./scripts/check-faults.sh
```

The gateway runs without authentication, so its port (and the web UI's) is
published on 127.0.0.1 only.

Scenarios (each lasts until `./scripts/restore-normal.sh`):

```bash
./scripts/inject-camera-fault.sh overhead 40   # freeze the overhead camera from step 40
./scripts/inject-slow-inference.sh 500         # +500 ms per policy step
./scripts/kill-policy.sh                       # SIGKILL the policy; the launch restarts it
```

Web UI: `docker compose --profile ui up`, then http://localhost:3000.

## The checkpoint

The example mounts `policy/` (or `$POLICY_DIR`) at `/opt/policy`. To train it yourself on
one GPU (about 6 GB is enough):

```bash
cd training
python3 -m venv .venv && . .venv/bin/activate
pip install torch==2.15.0 torchvision==0.29.2 -r ../requirements.txt "lerobot[dataset,training]==0.7.0"
./train.sh ./outputs
cp -r outputs/train/checkpoints/last/pretrained_model ../policy
```

`train.sh` records 200 successful demonstrations from the scripted expert (seeds 0 and up),
trains ACT for 30,000 steps, and evaluates it on 50 episodes with seeds from 100000, which
training never sees. The example runs the policy with `n_action_steps` 50 (it re-plans
every 50 steps; set in the checkpoint's `config.json`).

Measured for the checkpoint used here:

| Run | Success |
|---|---|
| `eval_policy.py`, 50 unseen seeds, GPU | 16 / 50 (32 %) |
| Same, re-planning every 25 / 100 steps | 13 / 50, 12 / 50 |
| Clean campaign in the container (CPU), 20 attempts | 8 / 20 (40 %) |

Most failures close the gripper about 1 cm away from the cube: the cube can appear
anywhere in a 29 x 55 cm area, and 200 demonstrations do not teach the policy to place
the gripper more precisely than that. A smaller spawn area or more demonstrations would
raise the rate; the example does not need a better policy to show the faults.

## Simulation

[so101-nexus](https://github.com/johnsutor/so101-nexus) 0.12.1 (Apache-2.0),
`MuJoCoPickLift-v1`, based on the MuJoCo Menagerie SO-101 model. The topics are the ones a
real SO-101 driver would offer, in radians, so the policy node does not depend on the
simulator:

| Topic | Type |
|---|---|
| `/so101/so101_sim/joint_states` | `sensor_msgs/JointState` (6 joints) |
| `/so101/so101_sim/joint_command` | `sensor_msgs/JointState` (6 position targets) |
| `/so101/so101_sim/camera/{wrist,overhead}/image_raw` | `sensor_msgs/Image` rgb8 320x240 (+ `/compressed`) |
| `/so101/so101_sim/task_status` | `std_msgs/String` JSON |

The simulation runs in lockstep: it steps only when the command for its last observation
arrives. A slow policy therefore makes attempts slower, but cannot change their outcome.

## Limitations

- Simulation only. Faults are injected, not natural; real-hardware runs are a later step.
- Lockstep: timing faults cannot change an outcome here.
- The success rate of the policy (about a third) is measured on simulated episodes only.
