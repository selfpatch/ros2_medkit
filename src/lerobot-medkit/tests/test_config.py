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


from lerobot_medkit import config
import pytest


def test_defaults_are_all_off():
    cfg = config.from_dict({})
    assert cfg.slow_inference.max_ms == 0
    assert cfg.frozen_camera.identical_frames == 0
    assert cfg.code('INFERENCE_SLOW') == 'LEROBOT_INFERENCE_SLOW'


def test_top_key_and_nested_values():
    cfg = config.from_dict(
        {'lerobot_medkit': {'code_prefix': 'POLICY', 'slow_inference': {'max_ms': 100}}}
    )
    assert cfg.code_prefix == 'POLICY'
    assert cfg.slow_inference.max_ms == 100
    assert cfg.slow_inference.window == 3
    assert cfg.slow_inference.measure == 'chunk'


@pytest.mark.parametrize(
    'values',
    [
        {'unknown': 1},
        {'slow_inference': {'max_sm': 1}},
        {'code_prefix': 'policy'},
        {'transport': 'zenoh'},
        {'slow_inference': {'measure': 'episode'}},
        {'frozen_camera': {'severity': 'fatal'}},
        {'introspection': 3},
        {'introspection': {'enabled': 'false'}},
        {'frozen_camera': {'identical_frames': 1}},
        {'frozen_camera': {'identical_frames': '3'}},
        {'frozen_camera': {'identical_frames': 2.5}},
        {'slow_inference': {'max_ms': -5}},
        {'slow_inference': {'max_ms': float('nan')}},
        {'slow_inference': {'max_ms': True}},
        {'slow_inference': {'window': 1e9}},
        {'slow_inference': {'window': 0}},
        {'slow_inference': {'clear_ratio': 0}},
        {'code_prefix': 3},
    ],
)
def test_rejects(values):
    with pytest.raises(ValueError):
        config.from_dict(values)


def test_packaged_file_loads(tmp_path):
    import pathlib

    path = pathlib.Path(__file__).parent.parent / 'config' / 'medkit.yaml'
    assert config.load(str(path)).code_prefix == 'LEROBOT'
