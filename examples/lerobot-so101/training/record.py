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

"""Record closed-loop scripted-expert pick-lift demos (DART-style noise) as a LeRobot v3 dataset.

The executed command is the expert label plus noise; the dataset stores the clean label,
so the policy learns to correct deviations. 25 % of episodes are recorded noise-free.

Units follow LeRobot's SO-101 convention (body joints in degrees, gripper 0-100 %),
so a policy trained here sees the same feature layout as a real SO-101 follower.
Only successful episodes are kept.

usage: record.py ROOT N_EPISODES [FIRST_SEED]
"""

from pathlib import Path
import shutil
import sys
import time

from expert import ClosedLoopExpert, Perturber
import gymnasium as gym
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np
from so101_nexus.config import PickConfig, RenderConfig, SO101_JOINT_NAMES
from so101_nexus.lerobot_dataset import sim_qpos_to_dataset_row
from so101_nexus.observations import JointPositions, OverheadCamera, WristCamera
from so101_nexus.rendering import RenderingQuality


H, W = 240, 320
FPS = 50  # MuJoCoPickLift-v1 control_dt = 0.02 s
MAX_STEPS = 400
TASK = 'Pick up the red cube and lift it.'
# Cheap enough for software rendering in a container (about 4x faster than the default
# 8x MSAA with an 8192 px shadow map). Training and the example must use the same.
QUALITY = RenderingQuality(msaa_samples=4, shadows=True, shadow_map_size=2048)

root = Path(sys.argv[1])
n_target = int(sys.argv[2])
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
if root.exists():
    shutil.rmtree(root)

names = [f'{j}.pos' for j in SO101_JOINT_NAMES]
img = {'dtype': 'video', 'shape': (H, W, 3), 'names': ['height', 'width', 'channels']}
features = {
    'observation.state': {'dtype': 'float32', 'shape': (6,), 'names': names},
    'action': {'dtype': 'float32', 'shape': (6,), 'names': names},
    'observation.images.wrist': img,
    'observation.images.overhead': img,
}
ds = LeRobotDataset.create(
    'local/so101_picklift_scripted',
    fps=FPS,
    features=features,
    root=root,
    robot_type='so101',
    use_videos=True,
    image_writer_threads=4,
)

cfg = PickConfig(
    terminate_on_success=False,
    obs_mode='visual',
    render=RenderConfig(quality=QUALITY),
    observations=[
        JointPositions(),
        WristCamera(width=W, height=H),
        OverheadCamera(width=W, height=H),
    ],
)
env = gym.make(
    'MuJoCoPickLift-v1', config=cfg, control_mode='pd_joint_pos', render_mode='rgb_array'
)

lo, hi = env.action_space.low, env.action_space.high
kept, tried, t0 = 0, 0, time.time()
while kept < n_target:
    obs, info = env.reset(seed=seed)
    ex = ClosedLoopExpert(env, rng=np.random.default_rng(seed))
    pert = Perturber(np.random.default_rng(seed + 10**6))
    seed += 1
    tried += 1
    frames = 0
    while ex.ok and not ex.done and frames < MAX_STEPS:
        q = ex.act()
        q_exec = np.clip(pert(q, ex.phase, ex.k), lo, hi)
        state = sim_qpos_to_dataset_row(np.asarray(obs['state'], dtype=np.float64))
        action = sim_qpos_to_dataset_row(q.astype(np.float64))
        ds.add_frame(
            {
                'observation.state': state.astype(np.float32),
                'action': action.astype(np.float32),
                'observation.images.wrist': obs['wrist_camera'],
                'observation.images.overhead': obs['overhead_camera'],
                'task': TASK,
            }
        )
        obs, _, _, _, info = env.step(q_exec.astype(np.float32))
        frames += 1
    if frames and ex.done and info.get('success'):
        ds.save_episode()
        kept += 1
        if kept % 10 == 0:
            print(f'kept {kept}/{tried} tried, {time.time() - t0:.0f}s', flush=True)
    else:
        ds.clear_episode_buffer()
ds.finalize()
print(f'done: kept {kept} of {tried} (expert success {kept / tried:.1%}), next seed {seed}')
