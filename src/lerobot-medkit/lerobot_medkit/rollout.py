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

"""`lerobot-medkit-rollout`: `lerobot-rollout` with the monitor attached.

    lerobot-medkit-rollout --medkit.config=medkit.yaml <lerobot-rollout arguments>

Every other argument goes to `lerobot-rollout` unchanged. The monitor is
attached to the local policy right after `lerobot-rollout` loads it, and the
introspection chunk goes through the policy's postprocessor (robot units).
Remote inference has no local policy: the monitor is then not attached, with
a warning.
"""

import logging
import sys

logger = logging.getLogger('lerobot_medkit')
CONFIG_ARG = '--medkit.config'


def _split_args(argv):
    config, rest = None, []
    args = iter(argv)
    for arg in args:
        if arg == CONFIG_ARG:
            config = next(args, None)
        elif arg.startswith(CONFIG_ARG + '='):
            config = arg.split('=', 1)[1]
        else:
            rest.append(arg)
    return config, rest


def install(config_path, rollout_module=None):
    """Make `rollout_module.build_rollout_context` attach a monitor. Returns the holder."""
    from lerobot_medkit import attach, load

    if rollout_module is None:
        from lerobot.scripts import lerobot_rollout as rollout_module

    config = load(config_path)
    original = rollout_module.build_rollout_context
    holder = {}

    def build_rollout_context(*args, **kwargs):
        ctx = original(*args, **kwargs)
        policy = ctx.policy.policy
        if policy is None:
            logger.warning('lerobot_medkit: remote inference, no local policy to monitor')
            return ctx
        post = ctx.policy.postprocessor
        holder['monitor'] = attach(policy, config, postprocess=post)
        logger.info('lerobot_medkit: monitoring %s', type(policy).__name__)
        return ctx

    rollout_module.build_rollout_context = build_rollout_context
    return holder


def main(argv=None):
    config_path, rest = _split_args(sys.argv[1:] if argv is None else argv)
    if config_path is None:
        sys.exit(f'lerobot-medkit-rollout: {CONFIG_ARG}=<file.yaml> is required')
    holder = install(config_path)
    from lerobot.scripts import lerobot_rollout

    sys.argv = [sys.argv[0]] + rest
    try:
        lerobot_rollout.main()
    finally:
        monitor = holder.get('monitor')
        if monitor is not None:
            monitor.close()


if __name__ == '__main__':
    main()
