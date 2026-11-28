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


"""PolicyMonitor on a small policy that behaves like LeRobot's ACT.

`select_action` pops a queue and calls `self.predict_action_chunk` when it is
empty, as `ACTPolicy` does in LeRobot 0.7.
"""

import collections
import inspect
import logging
import types

from lerobot_medkit import config, MemoryTransport, PolicyMonitor
from lerobot_medkit import monitor as monitor_module
import pytest

torch = pytest.importorskip('torch')

CHUNK, STEPS, DIM = 4, 2, 3


class Clock:
    """Advances by `tick` seconds per call; `advance()` adds time on demand."""

    def __init__(self):
        self.now = 0.0
        self.tick = 0.0005

    def __call__(self):
        self.now += self.tick
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeACT(torch.nn.Module):

    def __init__(self, clock=None):
        super().__init__()
        self.calls = 0
        self.queue = collections.deque()
        self.clock = clock
        self.inference_s = 0.0  # time one predict_action_chunk takes
        self.config = types.SimpleNamespace(n_action_steps=STEPS)

    def reset(self):
        self.queue.clear()

    def predict_action_chunk(self, batch):
        self.calls += 1
        if self.clock is not None:
            self.clock.advance(self.inference_s)
        return torch.full((1, CHUNK, DIM), float(self.calls))

    def select_action(self, batch):
        if not self.queue:
            chunk = self.predict_action_chunk(batch)[:, :STEPS]
            self.queue.extend(chunk.transpose(0, 1))
        return self.queue.popleft()


def batch(frame=0.0, top=None):
    return {
        'observation.state': torch.zeros(1, DIM),
        'observation.images.wrist': torch.full((1, 3, 4, 4), frame),
        'observation.images.top cam': torch.full((1, 3, 4, 4), frame if top is None else top),
    }


def make(values=None, postprocess=None, policy_cls=None):
    clock = Clock()
    policy = (policy_cls or FakeACT)(clock)
    transport = MemoryTransport()
    monitor = PolicyMonitor(
        policy,
        config.from_dict(values or {}),
        transport,
        postprocess=postprocess,
        clock=clock,
        wall_clock=lambda: 1000.0 + clock.now,
    ).attach()
    return policy, transport, monitor, clock


@pytest.fixture
def logs(monkeypatch):
    """Every line the monitor logs, as (level, message)."""
    lines = []

    def log(level, message, *args, exc_info=None):
        lines.append((level, message % args if args else message))

    monkeypatch.setattr(monitor_module.logger, 'log', log)
    monkeypatch.setattr(monitor_module.logger, 'warning',
                        lambda m, *a: lines.append((logging.WARNING, m % a if a else m)))
    return lines


def raised(transport, code=None):
    return [e[1] for e in transport.events if e[0] == 'raise' and code in (None, e[1])]


def cleared(transport, code=None):
    return [e[1] for e in transport.events if e[0] == 'clear' and code in (None, e[1])]


def test_attach_keeps_results_and_counts_steps_and_episodes():
    policy, transport, monitor, _ = make()
    reference = FakeACT()
    for _ in range(5):
        assert torch.equal(policy.select_action(batch()), reference.select_action(batch()))
    assert monitor.step == 5
    policy.reset()
    assert (monitor.episode, monitor.step) == (1, 0)


def test_introspection_once_per_chunk_with_step_episode_and_stamp():
    policy, transport, monitor, clock = make()
    for _ in range(5):
        policy.select_action(batch())
    policy.reset()
    policy.select_action(batch())
    got = [(e, s, c.shape, float(c[0, 0])) for e, s, c, *_ in transport.introspection]
    assert got == [(0, 0, (4, 3), 1.0), (0, 2, (4, 3), 2.0), (0, 4, (4, 3), 3.0),
                   (1, 0, (4, 3), 4.0)]
    assert transport.events == []  # every check is off by default


def test_chunk_is_stamped_when_the_inference_starts():
    starts = []
    original = FakeACT.predict_action_chunk

    def record_start(self, b):
        starts.append(1000.0 + clock.now)  # the wall clock as the inference begins
        return original(self, b)

    FakeACT.predict_action_chunk = record_start
    try:
        policy, transport, _, clock = make()
        policy.inference_s = 0.5
        for _ in range(3):
            policy.select_action(batch())
    finally:
        FakeACT.predict_action_chunk = original
    stamps = [stamp for *_, stamp in transport.introspection]
    assert len(stamps) == 2
    # Each stamp is from before the 0.5 s inference, not after it.
    assert all(start - 0.01 < stamp <= start for stamp, start in zip(stamps, starts))


class DictACT(FakeACT):
    """Returns a PolicyOutput (a dict with the actions under 'action')."""

    def predict_action_chunk(self, batch):
        return {'action': super().predict_action_chunk(batch)}

    def select_action(self, batch):
        if not self.queue:
            chunk = self.predict_action_chunk(batch)['action'][:, :STEPS]
            self.queue.extend(chunk.transpose(0, 1))
        return self.queue.popleft()


def test_policy_output_dict():
    policy, transport, _, _ = make(policy_cls=DictACT)
    policy.select_action(batch())
    assert transport.introspection[0][2].shape == (CHUNK, DIM)


def test_postprocess_applies_to_published_chunk():
    policy, transport, _, _ = make(postprocess=lambda c: c * 10)
    policy.select_action(batch())
    assert float(transport.introspection[0][2][0, 0]) == 10.0


SLOW_STEP = {'code_prefix': 'POLICY',
             'slow_inference': {'max_ms': 100, 'window': 2, 'measure': 'step'}}


def test_slow_step_raises_and_clears():
    policy, transport, _, clock = make(SLOW_STEP)
    clock.tick = 0.3  # every step takes 300 ms
    policy.select_action(batch())
    policy.select_action(batch())
    assert transport.events[-1][:3] == ('raise', 'POLICY_INFERENCE_SLOW', 'warn')
    clock.tick = 0.0005
    policy.select_action(batch())
    policy.select_action(batch())
    assert transport.events[-1] == ('clear', 'POLICY_INFERENCE_SLOW')
    assert len(transport.events) == 2


def test_first_good_window_passes_a_fault_left_by_an_earlier_run():
    # A respawned process: nothing raised yet, the first full window is fine.
    policy, transport, _, _ = make(SLOW_STEP)
    policy.select_action(batch())
    assert transport.events == []
    policy.select_action(batch())
    assert transport.events == [('clear', 'POLICY_INFERENCE_SLOW')]
    for _ in range(4):
        policy.select_action(batch())
    assert len(transport.events) == 1  # only the first state, then only changes


def test_reset_restarts_the_window():
    policy, transport, _, clock = make(SLOW_STEP)
    policy.select_action(batch())
    policy.select_action(batch())  # window [fast, fast]: passed
    policy.reset()
    clock.tick = 0.3
    policy.select_action(batch())
    # Without the reset, [fast, 300 ms] would average 150 ms and raise here.
    assert raised(transport) == []
    policy.select_action(batch())
    assert raised(transport) == ['POLICY_INFERENCE_SLOW']


def test_chunk_measure_stays_raised_through_cheap_steps():
    values = {'slow_inference': {'max_ms': 100, 'window': 1, 'measure': 'chunk'}}
    policy, transport, _, _ = make(values)
    policy.inference_s = 0.3
    for _ in range(STEPS * 3):  # 3 slow inferences, each followed by cheap pops
        policy.select_action(batch())
    assert [e[0] for e in transport.events] == ['raise']
    policy.inference_s = 0.0
    for _ in range(STEPS):
        policy.select_action(batch())
    assert [e[0] for e in transport.events] == ['raise', 'clear']


def test_step_measure_flaps_with_a_slow_model_and_warns(monkeypatch):
    warnings = []
    monkeypatch.setattr(monitor_module.logger, 'warning', lambda *a: warnings.append(a))
    policy = FakeACT()
    policy.config.n_action_steps = 50
    cfg = config.from_dict({'slow_inference': {'max_ms': 100, 'window': 10, 'measure': 'step'}})
    PolicyMonitor(policy, cfg, MemoryTransport()).attach()
    assert len(warnings) == 1 and 'n_action_steps' in warnings[0][0]
    warnings.clear()
    cfg = config.from_dict({'slow_inference': {'max_ms': 100, 'window': 10, 'measure': 'chunk'}})
    PolicyMonitor(FakeACT(), cfg, MemoryTransport()).attach()
    assert warnings == []


FROZEN = {'frozen_camera': {'identical_frames': 3}}
CODES = ['LEROBOT_CAMERA_TOP_CAM_FROZEN', 'LEROBOT_CAMERA_WRIST_FROZEN']


def test_frozen_camera_per_key():
    policy, transport, _, _ = make(FROZEN)
    policy.select_action(batch(0.0))
    assert sorted(cleared(transport)) == CODES  # first frame: a fault left behind is passed
    policy.select_action(batch(0.0))
    policy.select_action(batch(0.0))
    assert sorted(raised(transport)) == CODES
    policy.select_action(batch(1.0))
    assert sorted(cleared(transport)) == sorted(CODES * 2)


def test_frozen_camera_sees_a_reused_buffer():
    policy, transport, _, _ = make({'frozen_camera': {'identical_frames': 2}})
    b = batch(0.0)
    policy.select_action(b)
    b['observation.images.wrist'].fill_(1.0)  # the caller writes the next frame in place
    policy.select_action(b)
    assert raised(transport) == ['LEROBOT_CAMERA_TOP_CAM_FROZEN']  # control: top cam repeated


def test_camera_codes_that_collide_are_refused(logs):
    policy, transport, _, _ = make(FROZEN)
    b = {'observation.images.wrist-left': torch.zeros(2), 'observation.images.wrist_left':
         torch.ones(2)}
    policy.select_action(b)
    assert [level for level, m in logs if 'both map to' in m] == [logging.ERROR]
    assert cleared(transport) == ['LEROBOT_CAMERA_WRIST_LEFT_FROZEN']


def test_direct_chunk_calls_check_frames_but_publish_no_chunk(logs):
    # Chunk or RTC engine: predict_action_chunk without select_action.
    policy, transport, monitor, _ = make({'frozen_camera': {'identical_frames': 2}})
    for _ in range(3):
        policy.predict_action_chunk(batch())
    assert sorted(raised(transport)) == CODES
    assert transport.introspection == []
    assert len([m for _, m in logs if 'outside select_action' in m]) == 1


class SmolVLALike(FakeACT):
    """Runs its model from select_action without predict_action_chunk (SmolVLA)."""

    def _get_action_chunk(self, batch):
        if self.clock is not None:
            self.clock.advance(self.inference_s)
        return torch.zeros(1, CHUNK, DIM)

    def select_action(self, batch):
        if not self.queue:
            self.queue.extend(self._get_action_chunk(batch)[:, :STEPS].transpose(0, 1))
        return self.queue.popleft()


def test_policy_that_skips_predict_action_chunk_is_named_once(logs):
    policy, transport, _, _ = make({'slow_inference': {'max_ms': 10}}, policy_cls=SmolVLALike)
    policy.inference_s = 0.05
    for _ in range(2 * STEPS - 1):
        policy.select_action(batch())
    assert logs == []
    for _ in range(20):
        policy.select_action(batch())
    warned = [m for _, m in logs if 'without calling predict_action_chunk' in m]
    assert len(warned) == 1 and 'SmolVLALike' in warned[0] and 'measure: step' in warned[0]
    assert transport.introspection == [] and transport.events == []


def test_policy_that_skips_predict_action_chunk_works_with_measure_step(logs):
    values = {'slow_inference': {'max_ms': 10, 'window': STEPS, 'measure': 'step'}}
    policy, transport, _, _ = make(values, policy_cls=SmolVLALike)
    policy.inference_s = 0.05  # 50 ms every STEPS steps: mean 25 ms
    for _ in range(STEPS * 2):
        policy.select_action(batch())
    assert raised(transport) == ['LEROBOT_INFERENCE_SLOW']


def test_no_warning_for_a_policy_that_uses_predict_action_chunk(logs):
    policy, _, _, _ = make()
    for _ in range(10 * STEPS):
        policy.select_action(batch())
    assert logs == []


def test_one_failing_check_does_not_stop_the_others(monkeypatch, logs):
    def broken(a, b):
        raise RuntimeError('boom')

    monkeypatch.setattr(monitor_module, '_frames_equal', broken)
    values = dict(SLOW_STEP, frozen_camera={'identical_frames': 2})
    policy, transport, monitor, clock = make(values)
    clock.tick = 0.3
    for _ in range(4):
        policy.select_action(batch())
    assert raised(transport) == ['POLICY_INFERENCE_SLOW']
    assert monitor.step == 4
    assert [s for _, s, *_ in transport.introspection] == [0, 2]
    assert [m for _, m in logs] == ['lerobot_medkit: frozen_camera failed (logged once)']


def test_monitor_failure_does_not_reach_the_policy(logs):
    def broken(chunk):
        raise RuntimeError('boom')

    policy, transport, monitor, _ = make(postprocess=broken)
    for _ in range(5):  # three chunk inferences, all failing in the monitor
        policy.select_action(batch())
    assert len(logs) == 1
    monitor.close()  # the count of failures is reported at close
    assert logs[-1] == (logging.WARNING,
                        "lerobot_medkit: monitor failures: {'introspection': 3}")


def test_wrappers_keep_the_signature_lerobot_inspects():
    # lerobot checks inspect.signature(policy.predict_action_chunk) for RTC support.
    policy, _, _, _ = make()
    assert inspect.signature(policy.predict_action_chunk) == inspect.signature(
        FakeACT(None).predict_action_chunk)


def test_detach_puts_back_an_instance_attribute():
    policy = FakeACT()
    compiled = policy.predict_action_chunk  # torch.compile leaves one like this
    policy.predict_action_chunk = compiled
    monitor = PolicyMonitor(policy, config.from_dict({}), MemoryTransport()).attach()
    assert policy.predict_action_chunk is not compiled
    monitor.detach()
    assert vars(policy)['predict_action_chunk'] is compiled


def test_detach_restores_class_methods():
    policy, _, monitor, _ = make()
    monitor.detach()
    assert 'select_action' not in vars(policy)
    assert 'predict_action_chunk' not in vars(policy)
    with pytest.raises(RuntimeError):
        monitor.attach().attach()
