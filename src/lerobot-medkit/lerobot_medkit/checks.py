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

"""The checks, without ROS, torch or LeRobot.

Each check takes one reading at a time and returns the new state when it
changes (True: raise the fault, False: pass it), else None. The first state a
check can tell is always returned, also when it is False: a process that
restarts then passes a fault an earlier run of it left raised.
"""

import collections


class SlowInference:
    """Mean time over the last `window` readings above `max_ms`.

    Raised when the mean goes above `max_ms`, passed when it drops below
    `clear_ratio * max_ms`. `max_ms` 0 turns the check off.
    """

    def __init__(self, max_ms, window=10, clear_ratio=0.8):
        if max_ms < 0:
            raise ValueError('max_ms must be >= 0')
        if window < 1:
            raise ValueError('window must be at least 1')
        if not 0.0 < clear_ratio <= 1.0:
            raise ValueError('clear_ratio must be in (0, 1]')
        self.max_ms = float(max_ms)
        self.clear_ratio = float(clear_ratio)
        self._times = collections.deque(maxlen=int(window))
        self.slow = None  # None: not known yet

    @property
    def enabled(self):
        return self.max_ms > 0.0

    @property
    def window(self):
        return self._times.maxlen

    @property
    def mean_ms(self):
        return sum(self._times) / len(self._times) if self._times else 0.0

    def update(self, ms):
        if not self.enabled:
            return None
        self._times.append(float(ms))
        if len(self._times) < self._times.maxlen:
            return None
        mean = self.mean_ms
        if self.slow:
            slow = mean >= self.max_ms * self.clear_ratio
        else:
            slow = mean > self.max_ms
        if slow == self.slow:
            return None
        self.slow = slow
        return slow

    def reset_window(self):
        """Forget the readings (a new episode), keep the state."""
        self._times.clear()


class FrozenFrames:
    """`identical_frames` identical frames in a row from one camera.

    Raised on the `identical_frames`-th identical frame, passed on the first
    frame that differs (and on the very first frame). `identical_frames` 0
    turns the check off. Frames are compared with `equal(previous, current)`,
    so the caller decides how (numpy or torch, on which device).
    """

    def __init__(self, identical_frames, equal):
        if identical_frames != 0 and identical_frames < 2:
            raise ValueError('identical_frames must be 0 (off) or at least 2')
        self.identical_frames = int(identical_frames)
        self._equal = equal
        self._previous = None
        self._run = 0  # frames in a row equal to the latest one
        self.frozen = None  # None: not known yet

    @property
    def enabled(self):
        return self.identical_frames > 0

    def update(self, frame):
        if not self.enabled:
            return None
        same = self._previous is not None and self._equal(self._previous, frame)
        self._previous = frame
        self._run = self._run + 1 if same else 1
        frozen = self._run >= self.identical_frames
        if frozen == self.frozen:
            return None
        self.frozen = frozen
        return frozen
