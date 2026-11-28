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


"""lerobot-medkit-rollout: argument split and attaching to a built rollout context."""

import types

from lerobot_medkit import MemoryTransport, rollout
import pytest

torch = pytest.importorskip('torch')

from test_monitor import batch, FakeACT  # noqa: E402, I100


@pytest.mark.parametrize(
    'argv, config, rest',
    [
        (['--medkit.config=m.yaml', '--robot.type=x'], 'm.yaml', ['--robot.type=x']),
        (['--robot.type=x', '--medkit.config', 'm.yaml', '--fps=30'], 'm.yaml',
         ['--robot.type=x', '--fps=30']),
        (['--robot.type=x'], None, ['--robot.type=x']),
    ],
)
def test_split_args(argv, config, rest):
    assert rollout._split_args(argv) == (config, rest)


def fake_rollout_module(policy, postprocessor=None):
    def build_rollout_context(cfg, shutdown_event):
        return types.SimpleNamespace(
            policy=types.SimpleNamespace(policy=policy, postprocessor=postprocessor),
            cfg=cfg,
        )

    return types.SimpleNamespace(build_rollout_context=build_rollout_context)


@pytest.fixture
def config_file(tmp_path):
    path = tmp_path / 'medkit.yaml'
    path.write_text('lerobot_medkit:\n  transport: none\n  code_prefix: ROLL\n')
    return str(path)


def test_install_attaches_after_the_policy_is_loaded(config_file, monkeypatch):
    transport = MemoryTransport()
    monkeypatch.setattr('lerobot_medkit.monitor.make_transport', lambda c, node=None: transport)
    policy = FakeACT()
    module = fake_rollout_module(policy, postprocessor=lambda c: c * 2)
    holder = rollout.install(config_file, rollout_module=module)
    assert 'monitor' not in holder  # nothing attached until the context is built
    ctx = module.build_rollout_context('cfg', None)
    assert ctx.cfg == 'cfg'
    assert holder['monitor'].attached
    policy.select_action(batch())
    assert float(transport.introspection[0][2][0, 0]) == 2.0  # through the postprocessor


def test_install_skips_remote_inference(config_file):
    module = fake_rollout_module(None)
    holder = rollout.install(config_file, rollout_module=module)
    module.build_rollout_context('cfg', None)
    assert holder == {}


def test_main_requires_the_config():
    with pytest.raises(SystemExit):
        rollout.main(['--robot.type=x'])
