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

"""Configuration, from a YAML file or a dict. Unknown keys are an error."""

import dataclasses
import re

SEVERITIES = ('info', 'warn', 'error', 'critical')
MAX_WINDOW = 100000


def _expect(name, value, kind):
    if not isinstance(value, kind) or (kind is not bool and isinstance(value, bool)):
        raise ValueError(f'{name}: expected {kind.__name__}, got {value!r}')


def _integer(name, value, low, high):
    _expect(name, value, int)
    if not low <= value <= high:
        raise ValueError(f'{name}: {value} is not in [{low}, {high}]')


def _number(name, value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name}: expected a number, got {value!r}')
    if not low <= value <= high:
        raise ValueError(f'{name}: {value} is not in [{low}, {high}]')


@dataclasses.dataclass
class IntrospectionConfig:
    enabled: bool = True
    topic: str = '/policy/introspection'


@dataclasses.dataclass
class SlowInferenceConfig:
    max_ms: float = 0.0  # 0: off
    # 'step': time of each select_action call (what the control loop waits for).
    # 'chunk': time of each predict_action_chunk call (one model inference).
    measure: str = 'chunk'
    window: int = 3
    clear_ratio: float = 0.8
    severity: str = 'warn'


@dataclasses.dataclass
class FrozenCameraConfig:
    identical_frames: int = 0  # 0: off
    severity: str = 'error'


@dataclasses.dataclass
class MedkitConfig:
    code_prefix: str = 'LEROBOT'
    # Entity the faults are reported for. Empty: the transport's node.
    source_id: str = ''
    transport: str = 'rclpy'  # 'rclpy' or 'none'
    node_name: str = 'lerobot_medkit'
    report_service: str = '/fault_manager/report_fault'
    introspection: IntrospectionConfig = dataclasses.field(default_factory=IntrospectionConfig)
    slow_inference: SlowInferenceConfig = dataclasses.field(default_factory=SlowInferenceConfig)
    frozen_camera: FrozenCameraConfig = dataclasses.field(default_factory=FrozenCameraConfig)

    def __post_init__(self):
        for name in ('code_prefix', 'source_id', 'transport', 'node_name', 'report_service'):
            _expect(name, getattr(self, name), str)
        if not re.fullmatch(r'[A-Z][A-Z0-9_]*', self.code_prefix):
            raise ValueError(f'code_prefix {self.code_prefix!r}: use A-Z, 0-9 and _')
        if self.transport not in ('rclpy', 'none'):
            raise ValueError(f"transport {self.transport!r}: use 'rclpy' or 'none'")

        intro = self.introspection
        _expect('introspection.enabled', intro.enabled, bool)
        _expect('introspection.topic', intro.topic, str)

        slow = self.slow_inference
        _number('slow_inference.max_ms', slow.max_ms, 0.0, 1e7)
        if slow.measure not in ('step', 'chunk'):
            raise ValueError("slow_inference.measure: use 'step' or 'chunk'")
        _integer('slow_inference.window', slow.window, 1, MAX_WINDOW)
        _number('slow_inference.clear_ratio', slow.clear_ratio, 0.0, 1.0)
        if slow.clear_ratio == 0.0:
            raise ValueError('slow_inference.clear_ratio must be > 0')

        frozen = self.frozen_camera
        _integer('frozen_camera.identical_frames', frozen.identical_frames, 0, MAX_WINDOW)
        if frozen.identical_frames == 1:
            raise ValueError('frozen_camera.identical_frames: 0 (off) or at least 2')

        for name in ('slow_inference', 'frozen_camera'):
            severity = getattr(self, name).severity
            if severity not in SEVERITIES:
                raise ValueError(f'{name}.severity {severity!r}: use one of {SEVERITIES}')

    def code(self, suffix):
        return f'{self.code_prefix}_{suffix}'


def _build(cls, values, where):
    if not isinstance(values, dict):
        raise ValueError(f'{where}: expected a mapping')
    fields = {f.name: f for f in dataclasses.fields(cls)}
    unknown = sorted(set(values) - set(fields))
    if unknown:
        raise ValueError(f'{where}: unknown key(s) {unknown}')
    kwargs = {}
    for name, value in values.items():
        nested = fields[name].default_factory
        if dataclasses.is_dataclass(nested):
            kwargs[name] = _build(nested, value or {}, f'{where}.{name}')
        else:
            kwargs[name] = value
    return cls(**kwargs)


def from_dict(values):
    """Build a MedkitConfig from a dict, with or without a `lerobot_medkit:` top key."""
    values = values or {}
    if set(values) == {'lerobot_medkit'}:
        values = values['lerobot_medkit'] or {}
    return _build(MedkitConfig, values, 'lerobot_medkit')


def load(path):
    import yaml

    with open(path) as f:
        return from_dict(yaml.safe_load(f))
