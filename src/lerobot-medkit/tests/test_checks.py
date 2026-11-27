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


from lerobot_medkit.checks import FrozenFrames, SlowInference
import numpy as np
import pytest


def test_slow_inference_reports_its_first_state_also_when_fine():
    check = SlowInference(max_ms=100, window=2)
    assert check.update(10) is None
    assert check.update(10) is False
    assert check.update(10) is None


def test_slow_inference_needs_a_full_window():
    check = SlowInference(max_ms=100, window=3)
    assert check.update(500) is None
    assert check.update(500) is None
    assert check.update(500) is True


def test_slow_inference_hysteresis():
    check = SlowInference(max_ms=100, window=2, clear_ratio=0.8)
    check.update(150)
    assert check.update(150) is True
    assert check.update(85) is None  # mean 117
    assert check.update(85) is None  # mean 85 >= 80: still slow
    assert check.update(70) is False  # mean 77.5


def test_slow_inference_off():
    check = SlowInference(max_ms=0, window=1)
    assert check.update(10000) is None
    assert not check.enabled


def test_slow_inference_reset_window_keeps_state():
    check = SlowInference(max_ms=100, window=2)
    check.update(200)
    check.update(200)
    assert check.slow
    check.reset_window()
    assert check.slow
    assert check.update(10) is None  # window not full again yet
    assert check.update(10) is False


@pytest.mark.parametrize('kwargs', [{'window': 0}, {'clear_ratio': 0.0}, {'clear_ratio': 1.5}])
def test_slow_inference_rejects_bad_values(kwargs):
    with pytest.raises(ValueError):
        SlowInference(max_ms=100, **kwargs)


def test_frozen_frames_raises_on_nth_identical_and_passes_on_change():
    check = FrozenFrames(3, np.array_equal)
    a, b = np.zeros((2, 2)), np.ones((2, 2))
    assert check.update(a) is False  # run 1: the first state is always reported
    assert check.update(a) is None  # run 2
    assert check.update(a) is True  # run 3
    assert check.update(a) is None  # still frozen
    assert check.update(b) is False
    assert check.update(b) is None


def test_frozen_frames_off_and_bad_values():
    assert FrozenFrames(0, np.array_equal).update(np.zeros(1)) is None
    with pytest.raises(ValueError):
        FrozenFrames(1, np.array_equal)
