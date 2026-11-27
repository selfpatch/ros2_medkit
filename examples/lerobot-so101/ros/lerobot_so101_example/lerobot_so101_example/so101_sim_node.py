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

"""The robot: a simulated SO-101 arm (so101-nexus MuJoCoPickLift-v1) as ROS 2 topics.

The topics are the ones a real SO-101 driver would offer, in ROS units
(radians), so the policy node does not know whether it drives the simulation
or the real arm:

  ~/joint_states                         sensor_msgs/JointState, 6 joints
  ~/joint_command                        sensor_msgs/JointState, 6 position targets
  ~/camera/{wrist,overhead}/image_raw    sensor_msgs/Image rgb8 (+ /compressed JPEG)
  ~/task_status                          std_msgs/String JSON (step, success, done)
  ~/reset                                std_srvs/Trigger, starts the next episode

Lockstep: after a reset the node publishes one observation (both images and the
joint states with the same stamp) and steps the simulation only when a command
with that stamp arrives. A slow or dead policy pauses the robot instead of
letting it run on old commands, and each episode is reproducible from its seed.

A camera fault is injected at run time, for example:

  ros2 param set /so101/so101_sim camera_fault frozen

Every stamp is on the wall clock, the clock the fault manager uses.
"""

import io
import json

import gymnasium as gym
import numpy as np
from PIL import Image as PILImage
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState
from so101_nexus.config import PickConfig, RenderConfig, SO101_JOINT_NAMES
import so101_nexus.mujoco  # noqa: F401  registers the MuJoCo environments
from so101_nexus.observations import JointPositions, OverheadCamera, WristCamera
from so101_nexus.rendering import RenderingQuality
from std_msgs.msg import String
from std_srvs.srv import Trigger

CAMERAS = ('wrist', 'overhead')
CAMERA_FAULTS = ('none', 'frozen')
# Must match training/record.py: the policy saw images rendered this way. Cheap enough
# for software rendering in a container (no GPU needed).
QUALITY = RenderingQuality(msaa_samples=4, shadows=True, shadow_map_size=2048)


class So101SimNode(Node):

    def __init__(self):
        super().__init__('so101_sim')
        self.declare_parameter('env_id', 'MuJoCoPickLift-v1')
        self.declare_parameter('base_seed', 100000)
        self.declare_parameter('max_steps', 300)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('camera_fault', 'none')
        self.declare_parameter('camera_fault_camera', 'overhead')
        self.declare_parameter('camera_fault_start_step', 40)

        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        self._base_seed = int(self.get_parameter('base_seed').value)
        self._max_steps = int(self.get_parameter('max_steps').value)
        config = PickConfig(
            obs_mode='visual',
            render=RenderConfig(quality=QUALITY),
            observations=[
                JointPositions(),
                WristCamera(width=width, height=height),
                OverheadCamera(width=width, height=height),
            ],
        )
        env_id = self.get_parameter('env_id').value
        self._env = gym.make(
            env_id, config=config, control_mode='pd_joint_pos', render_mode='rgb_array'
        )
        self._low = self._env.action_space.low
        self._high = self._env.action_space.high

        self._fault = 'none'
        self._fault_camera = 'overhead'
        self._fault_start = 40
        self._frozen = None
        self._set_fault(
            self.get_parameter('camera_fault').value,
            self.get_parameter('camera_fault_camera').value,
            self.get_parameter('camera_fault_start_step').value,
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._image_pubs = {
            c: self.create_publisher(Image, f'~/camera/{c}/image_raw', 10) for c in CAMERAS
        }
        self._jpeg_pubs = {
            c: self.create_publisher(CompressedImage, f'~/camera/{c}/image_raw/compressed', 10)
            for c in CAMERAS
        }
        self._joint_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self._status_pub = self.create_publisher(String, '~/task_status', 10)
        self.create_subscription(JointState, '~/joint_command', self._on_command, 10)
        self.create_service(Trigger, '~/reset', self._on_reset)

        self._episode = -1
        self._step = 0
        self._running = False
        self._obs_stamp = None
        self._max_lift = 0.0
        self.get_logger().info(f'{env_id} ready, waiting for a reset')

    # -- camera fault injection --
    def _set_fault(self, kind, camera, start_step):
        if kind not in CAMERA_FAULTS:
            raise ValueError(f'camera_fault must be one of {CAMERA_FAULTS}, got {kind!r}')
        if camera not in CAMERAS:
            raise ValueError(f'camera_fault_camera must be one of {CAMERAS}, got {camera!r}')
        self._fault, self._fault_camera, self._fault_start = kind, camera, int(start_step)
        self._frozen = None
        if kind == 'none':
            self.get_logger().info('camera fault cleared')
        else:
            self.get_logger().warning(
                f'camera fault injected: {camera} camera {kind} '
                f'from step {start_step} of each episode'
            )

    def _on_set_parameters(self, params):
        values = {p.name: p.value for p in params}
        keys = ('camera_fault', 'camera_fault_camera', 'camera_fault_start_step')
        if any(k in values for k in keys):
            try:
                self._set_fault(
                    values.get('camera_fault', self._fault),
                    values.get('camera_fault_camera', self._fault_camera),
                    values.get('camera_fault_start_step', self._fault_start),
                )
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _apply_fault(self, camera, pixels):
        if (
            self._fault != 'frozen'
            or camera != self._fault_camera
            or self._step < self._fault_start
        ):
            return pixels
        if self._frozen is None:
            self._frozen = pixels.copy()
        return self._frozen

    # -- episode control --
    def _on_reset(self, request, response):
        self._episode += 1
        self._frozen = None
        seed = self._base_seed + self._episode
        obs, _ = self._env.reset(seed=seed)
        self._step = 0
        self._max_lift = 0.0
        self._running = True
        self._publish_observation(obs)
        self._publish_status(success=False, done=False)
        response.success = True
        response.message = json.dumps({'episode': self._episode, 'seed': seed})
        self.get_logger().info(f'episode {self._episode} reset with seed {seed}')
        return response

    def _on_command(self, msg):
        if not self._running or self._obs_stamp is None or msg.header.stamp != self._obs_stamp:
            return  # not for the current observation: lockstep keeps waiting
        if len(msg.position) != len(SO101_JOINT_NAMES):
            self.get_logger().error(
                f'joint command has {len(msg.position)} values, expected {len(SO101_JOINT_NAMES)}'
            )
            return
        target = np.asarray(msg.position, dtype=np.float32)
        if not np.all(np.isfinite(target)):
            self.get_logger().error('joint command has a non-finite value, ignored')
            return
        obs, _, terminated, truncated, info = self._env.step(
            np.clip(target, self._low, self._high)
        )
        self._step += 1
        self._max_lift = max(self._max_lift, float(info.get('lift_height', 0.0)))
        success = bool(info.get('success', False))
        done = bool(terminated or truncated or success or self._step >= self._max_steps)
        if done:
            self._running = False
        else:
            self._publish_observation(obs)
        self._publish_status(success=success, done=done)
        if done:
            outcome = 'success' if success else 'failure'
            self.get_logger().info(
                f'episode {self._episode} ended after {self._step} steps: {outcome}'
            )

    def _publish_observation(self, obs):
        stamp = self.get_clock().now().to_msg()
        self._obs_stamp = stamp
        quality = int(self.get_parameter('jpeg_quality').value)
        for camera in CAMERAS:
            pixels = np.ascontiguousarray(
                self._apply_fault(camera, obs[f'{camera}_camera']), dtype=np.uint8
            )
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = f'{camera}_camera'
            image.height, image.width = int(pixels.shape[0]), int(pixels.shape[1])
            image.encoding = 'rgb8'
            image.step = image.width * 3
            image.data = pixels.tobytes()
            self._image_pubs[camera].publish(image)

            jpeg = io.BytesIO()
            PILImage.fromarray(pixels).save(jpeg, format='JPEG', quality=quality)
            compressed = CompressedImage()
            compressed.header = image.header
            compressed.format = 'jpeg'
            compressed.data = jpeg.getvalue()
            self._jpeg_pubs[camera].publish(compressed)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(SO101_JOINT_NAMES)
        joints.position = [float(v) for v in obs['state']]
        self._joint_pub.publish(joints)

    def _publish_status(self, success, done):
        self._status_pub.publish(
            String(
                data=json.dumps(
                    {
                        'episode': self._episode,
                        'step': self._step,
                        'max_lift_m': self._max_lift,
                        'success': success,
                        'done': done,
                    }
                )
            )
        )


def main():
    rclpy.init()
    node = So101SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
