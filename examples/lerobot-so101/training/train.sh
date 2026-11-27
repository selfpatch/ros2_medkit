#!/usr/bin/env bash
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

# Reproduce the example's ACT checkpoint on one GPU (tested on an RTX 3060 Laptop, 6 GB).
#
#   python3 -m venv .venv && . .venv/bin/activate
#   pip install torch==2.15.0 torchvision==0.29.2 -r ../requirements.txt "lerobot[dataset,training]==0.7.0"
#   ./train.sh [WORK_DIR]
#
# 1. record.py: 200 successful demos from the SCRIPTED closed-loop expert (expert.py),
#    seeds 0.., with DART-style noise on the executed command (labels stay clean)
# 2. lerobot-train: ACT on those demos
# 3. eval_policy.py: 50 episodes on seeds 100000.. (never used for training)
# The checkpoint lands in WORK_DIR/train/checkpoints/last/pretrained_model:
# point the example's POLICY_DIR at it.
set -euo pipefail
cd "$(dirname "$0")"
WORK=${1:-./outputs}
STEPS=${STEPS:-30000}
export MUJOCO_GL=${MUJOCO_GL:-egl}
export WANDB_MODE=${WANDB_MODE:-disabled}

python record.py "$WORK/dataset" 200 0
lerobot-train \
  --dataset.repo_id=local/so101_picklift_scripted --dataset.root="$WORK/dataset" \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --batch_size=8 --steps="$STEPS" --save_freq=10000 --log_freq=200 --num_workers=4 \
  --seed=1000 --output_dir="$WORK/train"
# The example runs the policy with n_action_steps=50 (re-plan every 50 steps); evaluate the same way.
N_ACTION_STEPS=50 python eval_policy.py "$WORK/train/checkpoints/last/pretrained_model" 50 100000 cuda "$WORK/eval_videos"
