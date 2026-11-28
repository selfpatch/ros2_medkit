# Copyright 2026 ahmedsleem109
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A LeRobot policy (ACT) as a ROS 2 node.

Pairs the two camera images and the joint states that carry the same stamp (one
observation), runs the policy on them the way `lerobot-rollout` does
(`predict_action`: preprocessor, `select_action`, postprocessor) and publishes
the action as a joint command with that same stamp.

ROS topics carry radians; the policy was trained on LeRobot's SO-101 units
(body joints in degrees, gripper 0-100 %), so the node converts both ways.

Without lerobot-medkit, health goes out on the standard `/diagnostics` topic,
which ros2_medkit's diagnostic bridge turns into faults:

  "lerobot_policy: inference"  WARN when the mean step time over the last
                               `inference_window` steps exceeds `inference_ms_max`,
                               OK again below `inference_clear_ratio` of it

A status goes out when it changes; OK is repeated every `diagnostics_period_sec`.
WARN is not repeated: the bridge forwards every message, so a WARN repeated each
second would need as many OKs to heal as the fault lasted seconds.

When lerobot-medkit (src/lerobot-medkit) is installed and `use_lerobot_medkit`
is true, the node does none of that itself: it attaches the plugin to the policy
(`lerobot_medkit_config`), which reports POLICY_INFERENCE_SLOW and
POLICY_CAMERA_<NAME>_FROZEN straight to the fault manager, from the calls
LeRobot makes, and publishes the chunks described below. If the plugin cannot
be attached (a bad configuration, missing messages), the node logs why and
uses `/diagnostics`.

`inference_delay_ms` adds a sleep to inject a timing fault: inside each model
inference (`predict_action_chunk`) with lerobot-medkit, which times those;
inside every `select_action` call without it, where the node times steps.

Without the plugin, when `ros2_medkit_policy_msgs` is installed, every inference is also published
as a `PolicyIntrospection` on `introspection_topic`: the whole predicted chunk
(in radians, like the joint command), the episode (one per policy reset), the
step it ran at, and the inference time. ros2_medkit_policy_bridge turns that
into CHUNK_DISAGREEMENT and SLOW_INFERENCE faults. The chunk is taken with a
forward hook on the ACT model, so the policy still runs exactly as LeRobot
runs it.
"""

import collections
import os
import time

from ament_index_python.packages import get_package_share_directory

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from so101_nexus.lerobot_dataset import dataset_row_to_sim_qpos, sim_qpos_to_dataset_row
from std_srvs.srv import Trigger
import torch

try:
    import lerobot_medkit
except ImportError:  # src/lerobot-medkit not installed: plain /diagnostics
    lerobot_medkit = None

try:
    from ros2_medkit_policy_msgs.msg import PolicyIntrospection
except ImportError:  # ros2_medkit_policy_msgs not built: no introspection
    PolicyIntrospection = None

CAMERAS = ('wrist', 'overhead')
DIAG_NAME = 'lerobot_policy: inference'


def _key(stamp):
    return (stamp.sec, stamp.nanosec)


class LeRobotPolicyNode(Node):

    def __init__(self):
        super().__init__('lerobot_policy')
        self.declare_parameter('policy_path', os.environ.get('POLICY_PATH', '/opt/policy'))
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('task', 'Pick up the red cube and lift it.')
        self.declare_parameter('robot_ns', '/so101/so101_sim')
        self.declare_parameter('inference_ms_max', 100.0)
        self.declare_parameter('inference_clear_ratio', 0.8)
        self.declare_parameter('inference_window', 10)
        self.declare_parameter('inference_delay_ms', 0)
        self.declare_parameter('diagnostics_period_sec', 5.0)
        self.declare_parameter('introspection_topic', '/policy/introspection')
        self.declare_parameter('use_lerobot_medkit', True)
        self.declare_parameter(
            'lerobot_medkit_config',
            os.path.join(
                get_package_share_directory('lerobot_so101_example'),
                'config',
                'lerobot_medkit.yaml',
            ),
        )

        path = self.get_parameter('policy_path').value
        device = self.get_parameter('device').value
        torch.set_num_threads(max(1, (os.cpu_count() or 2) // 2))
        config = PreTrainedConfig.from_pretrained(path)
        config.device = device
        # The checkpoint holds every weight: do not fetch the ImageNet backbone.
        if hasattr(config, 'pretrained_backbone_weights'):
            config.pretrained_backbone_weights = None
        self._policy = get_policy_class(config.type).from_pretrained(path, config=config)
        self._policy.to(device).eval()
        self._pre, self._post = make_pre_post_processors(
            config,
            pretrained_path=path,
            preprocessor_overrides={'device_processor': {'device': device}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}},
        )
        self._device = torch.device(device)
        self._monitor = None
        if lerobot_medkit is not None and self.get_parameter('use_lerobot_medkit').value:
            self._attach_monitor()
        if self._monitor is None:
            self._inject_delay('select_action')

        # Introspection: the ACT model runs only when the action queue is empty
        # and returns (chunk, latent); keep the chunk of the last run.
        self._chunk = None
        self._episode = 0
        self._step = 0
        self._introspection_pub = None
        if (
            self._monitor is None
            and PolicyIntrospection is not None
            and hasattr(self._policy, 'model')
        ):
            self._policy.model.register_forward_hook(self._on_model_output)
            self._introspection_pub = self.create_publisher(
                PolicyIntrospection, self.get_parameter('introspection_topic').value, 10
            )

        ns = self.get_parameter('robot_ns').value
        self._pending = {}  # stamp -> {'joints': msg, 'wrist': msg, 'overhead': msg}
        self._command_pub = self.create_publisher(JointState, f'{ns}/joint_command', 10)
        self.create_subscription(
            JointState, f'{ns}/joint_states', lambda m: self._on_part('joints', m), 10
        )
        for camera in CAMERAS:
            self.create_subscription(
                Image,
                f'{ns}/camera/{camera}/image_raw',
                lambda m, c=camera: self._on_part(c, m),
                10,
            )
        self.create_service(Trigger, '~/reset', self._on_reset)

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._step_ms = collections.deque(maxlen=int(self.get_parameter('inference_window').value))
        self._slow = False
        if self._monitor is None:
            self.create_timer(
                float(self.get_parameter('diagnostics_period_sec').value), self._repeat_ok
            )
        self.get_logger().info(f'{config.type} policy loaded from {path} on {device}')

    def _attach_monitor(self):
        # The injected delay goes inside what lerobot-medkit times (each model
        # inference, config/lerobot_medkit.yaml), so it is put on first.
        self._inject_delay('predict_action_chunk')
        path = self.get_parameter('lerobot_medkit_config').value
        try:
            self._monitor = lerobot_medkit.attach(
                self._policy, path, postprocess=self._to_radians, node=self
            )
        except Exception as error:  # noqa: B902 - any error: run without the plugin
            del self._policy.predict_action_chunk
            self.get_logger().error(
                f'lerobot-medkit not attached ({type(error).__name__}: {error}); '
                'reporting through /diagnostics instead'
            )
            return
        self.get_logger().info(f'lerobot-medkit attached to the policy ({path})')

    def _inject_delay(self, method):
        # `inference_delay_ms` sleeps inside `method` of the policy, so whatever
        # times that call (this node or lerobot-medkit) sees the delay.
        original = getattr(self._policy, method)

        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            delay_ms = int(self.get_parameter('inference_delay_ms').value)
            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)
            return result

        setattr(self._policy, method, delayed)

    def close(self):
        if self._monitor is not None:
            self._monitor.close()
            self._monitor = None

    def _to_radians(self, chunk):
        # The model works in normalized units: the postprocessor turns the chunk
        # into LeRobot SO-101 units, then into radians like the joint command.
        rows = self._post(chunk).squeeze(0).float().cpu().numpy().astype(np.float64)
        return np.stack([dataset_row_to_sim_qpos(row) for row in rows])

    def _on_model_output(self, module, inputs, output):
        self._chunk = output[0].detach()

    def _on_reset(self, request, response):
        self._policy.reset()
        self._episode += 1
        self._step = 0
        self._chunk = None
        self._pending.clear()
        self._step_ms.clear()
        response.success = True
        response.message = 'policy reset'
        return response

    def _on_part(self, part, msg):
        key = _key(msg.header.stamp)
        parts = self._pending.setdefault(key, {})
        parts[part] = msg
        if len(parts) < 1 + len(CAMERAS):
            return
        del self._pending[key]
        # An older observation can never complete now: drop it.
        self._pending = {k: v for k, v in self._pending.items() if k > key}
        self._act(parts)

    def _act(self, parts):
        joints = parts['joints']
        observation = {
            'observation.state': sim_qpos_to_dataset_row(
                np.asarray(joints.position, dtype=np.float64)
            ).astype(np.float32)
        }
        for camera in CAMERAS:
            image = parts[camera]
            if image.encoding != 'rgb8':
                self.get_logger().error(f'unsupported image encoding {image.encoding}')
                return
            observation[f'observation.images.{camera}'] = np.frombuffer(
                image.data, dtype=np.uint8
            ).reshape(image.height, image.width, 3)

        start = time.monotonic()
        self._chunk = None
        action = predict_action(
            observation,
            self._policy,
            self._device,
            self._pre,
            self._post,
            use_amp=False,
            task=self.get_parameter('task').value,
            robot_type='so101',
        )
        step_ms = (time.monotonic() - start) * 1000.0
        if self._monitor is None:
            self._step_ms.append(step_ms)
            self._check_inference()
        if self._chunk is not None and self._introspection_pub is not None:
            self._publish_introspection(joints.header.stamp, step_ms)
        self._step += 1

        target = dataset_row_to_sim_qpos(
            action.squeeze(0).float().cpu().numpy().astype(np.float64)
        )
        command = JointState()
        command.header.stamp = joints.header.stamp
        command.name = list(joints.name)
        command.position = [float(v) for v in target]
        self._command_pub.publish(command)

    def _publish_introspection(self, stamp, inference_ms):
        chunk = self._to_radians(self._chunk)
        msg = PolicyIntrospection()
        msg.header.stamp = stamp
        msg.episode = self._episode
        msg.step = self._step
        msg.chunk_length, msg.action_dim = chunk.shape
        msg.chunk = chunk.astype(np.float32).ravel().tolist()
        msg.inference_ms = float(inference_ms)
        self._introspection_pub.publish(msg)

    def _check_inference(self):
        if len(self._step_ms) < self._step_ms.maxlen:
            return
        mean = float(np.mean(self._step_ms))
        limit = float(self.get_parameter('inference_ms_max').value)
        if self._slow:
            slow = mean >= limit * float(self.get_parameter('inference_clear_ratio').value)
        else:
            slow = mean > limit
        if slow != self._slow:
            self._slow = slow
            message = f'mean step time {mean:.0f} ms (limit {limit:.0f} ms)'
            # One call site per severity: rclpy refuses to change it between calls.
            if slow:
                self.get_logger().warning(f'{message}: slow')
            else:
                self.get_logger().info(f'{message}: back to normal')
            self._publish_diagnostics()

    def _repeat_ok(self):
        if not self._slow:
            self._publish_diagnostics()

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = DIAG_NAME
        status.hardware_id = self.get_fully_qualified_name()
        limit = float(self.get_parameter('inference_ms_max').value)
        mean = float(np.mean(self._step_ms)) if self._step_ms else 0.0
        if self._slow:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f'mean step time {mean:.0f} ms over the last {len(self._step_ms)} '
                f'steps, limit {limit:.0f} ms'
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'inference within limit'
        status.values = [
            KeyValue(key='mean_step_ms', value=f'{mean:.1f}'),
            KeyValue(key='limit_ms', value=f'{limit:.1f}'),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main():
    rclpy.init()
    node = None
    try:
        # Inside the try: loading the checkpoint takes seconds, and a SIGINT
        # then (docker compose stop) must end the process cleanly too.
        node = LeRobotPolicyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
