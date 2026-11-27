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

"""Evaluate a trained LeRobot policy on MuJoCoPickLift-v1 over fixed seeds.

usage: eval_policy.py CHECKPOINT_DIR [N=50] [FIRST_SEED=100000] [DEVICE=cuda] [VIDEO_DIR]
Seeds start at 100000 so they never overlap the training seeds (0..).
"""

import json
import os
from pathlib import Path
import sys
import time

import gymnasium as gym
from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
import numpy as np
from so101_nexus.config import PickConfig, RenderConfig
from so101_nexus.lerobot_dataset import dataset_row_to_sim_qpos, sim_qpos_to_dataset_row
import so101_nexus.mujoco  # noqa: F401
from so101_nexus.observations import JointPositions, OverheadCamera, WristCamera
from so101_nexus.rendering import RenderingQuality
import torch


H, W = 240, 320
TASK = 'Pick up the red cube and lift it.'
# Cheap enough for software rendering in a container (about 4x faster than the default
# 8x MSAA with an 8192 px shadow map). Training and the example must use the same.
QUALITY = RenderingQuality(msaa_samples=4, shadows=True, shadow_map_size=2048)
MAX_STEPS = 300  # same as so101_sim_node max_steps

ckpt = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 50
seed0 = int(sys.argv[3]) if len(sys.argv) > 3 else 100000
device = sys.argv[4] if len(sys.argv) > 4 else 'cuda'
video_dir = Path(sys.argv[5]) if len(sys.argv) > 5 else None

cfg_p = PreTrainedConfig.from_pretrained(ckpt)
cfg_p.device = device
# Optional inference-time overrides: re-plan more often than every chunk.

if os.environ.get('N_ACTION_STEPS'):
    cfg_p.n_action_steps = int(os.environ['N_ACTION_STEPS'])
if os.environ.get('TEMPORAL_ENSEMBLE'):
    cfg_p.temporal_ensemble_coeff = float(os.environ['TEMPORAL_ENSEMBLE'])
    cfg_p.n_action_steps = 1
policy = get_policy_class(cfg_p.type).from_pretrained(ckpt, config=cfg_p).to(device).eval()
pre, post = make_pre_post_processors(
    cfg_p,
    pretrained_path=ckpt,
    preprocessor_overrides={'device_processor': {'device': device}},
    postprocessor_overrides={'device_processor': {'device': 'cpu'}},
)

env = gym.make(
    'MuJoCoPickLift-v1',
    config=PickConfig(
        obs_mode='visual',
        render=RenderConfig(quality=QUALITY),
        observations=[
            JointPositions(),
            WristCamera(width=W, height=H),
            OverheadCamera(width=W, height=H),
        ],
    ),
    control_mode='pd_joint_pos',
    render_mode='rgb_array',
)

results, infer_ms = [], []
for k in range(n):
    seed = seed0 + k
    obs, info = env.reset(seed=seed)
    policy.reset()
    pre.reset() if hasattr(pre, 'reset') else None
    frames, success, steps = [], False, 0
    for steps in range(1, MAX_STEPS + 1):
        o = {
            'observation.state': sim_qpos_to_dataset_row(
                np.asarray(obs['state'], np.float64)
            ).astype(np.float32),
            'observation.images.wrist': obs['wrist_camera'],
            'observation.images.overhead': obs['overhead_camera'],
        }
        t = time.perf_counter()
        a = predict_action(
            o,
            policy,
            torch.device(device),
            pre,
            post,
            use_amp=False,
            task=TASK,
            robot_type='so101',
        )
        infer_ms.append((time.perf_counter() - t) * 1e3)
        q = dataset_row_to_sim_qpos(a.squeeze(0).float().cpu().numpy().astype(np.float64))
        if video_dir is not None and k < 5:
            frames.append(np.concatenate([obs['overhead_camera'], obs['wrist_camera']], axis=1))
        obs, _, term, trunc, info = env.step(q.astype(np.float32))
        if info.get('success'):
            success = True
            break
        if term or trunc:
            break
    results.append(
        {
            'seed': seed,
            'success': success,
            'steps': steps,
            'lift_height': float(info.get('lift_height', 0.0)),
        }
    )
    print(f'seed {seed}: {"OK " if success else "FAIL"} steps {steps}', flush=True)
    if frames:
        import av

        video_dir.mkdir(parents=True, exist_ok=True)
        with av.open(str(video_dir / f'seed{seed}_{"ok" if success else "fail"}.mp4'), 'w') as c:
            s = c.add_stream('libx264', rate=50)
            s.width, s.height, s.pix_fmt = frames[0].shape[1], frames[0].shape[0], 'yuv420p'
            for f in frames:
                for p in s.encode(av.VideoFrame.from_ndarray(f, format='rgb24')):
                    c.mux(p)
            for p in s.encode():
                c.mux(p)

ok = sum(r['success'] for r in results)
summary = {
    'checkpoint': ckpt,
    'n_action_steps': cfg_p.n_action_steps,
    'temporal_ensemble_coeff': cfg_p.temporal_ensemble_coeff,
    'episodes': n,
    'first_seed': seed0,
    'success': ok,
    'success_rate': ok / n,
    'device': device,
    'infer_ms_median': float(np.median(infer_ms)),
    'infer_ms_p99': float(np.percentile(infer_ms, 99)),
    'results': results,
}
tag = f'nas{cfg_p.n_action_steps}' + (
    f'_te{cfg_p.temporal_ensemble_coeff}' if cfg_p.temporal_ensemble_coeff else ''
)
out = Path(ckpt).parent / f'eval_{seed0}_{n}_{device}_{tag}.json'
out.write_text(json.dumps(summary, indent=1))
print(
    f'SUCCESS {ok}/{n} = {ok / n:.0%}  infer median {summary["infer_ms_median"]:.1f} ms  -> {out}'
)
