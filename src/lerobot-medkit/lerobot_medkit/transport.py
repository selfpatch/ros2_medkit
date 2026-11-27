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

"""Where reports go. One interface, several backends.

The monitor calls these from the control loop; a backend must not block it.
`rclpy_transport.RclpyTransport` is the ROS 2 backend. A non-ROS backend
implements the same four methods.
"""

import abc


class Transport(abc.ABC):

    @abc.abstractmethod
    def report_fault(self, code, severity, description):
        """Raise `code` (severity: 'info', 'warn', 'error' or 'critical')."""

    @abc.abstractmethod
    def clear_fault(self, code):
        """Pass `code`: the condition is gone."""

    def publish_introspection(self, episode, step, chunk, inference_ms, stamp=None):
        """Publish one inference.

        `chunk` is a float32 array (chunk_length, action_dim); `stamp` is the
        wall-clock time (seconds) the inference started, or None for now.
        """

    def close(self):
        """Release what the backend holds."""


class NullTransport(Transport):
    """Drops everything (transport: none)."""

    def report_fault(self, code, severity, description):
        pass

    def clear_fault(self, code):
        pass


class MemoryTransport(Transport):
    """Keeps every call, in order. For tests."""

    def __init__(self):
        self.events = []  # ('raise', code, severity, description) / ('clear', code)
        self.introspection = []  # (episode, step, chunk, inference_ms, stamp)

    def report_fault(self, code, severity, description):
        self.events.append(('raise', code, severity, description))

    def clear_fault(self, code):
        self.events.append(('clear', code))

    def publish_introspection(self, episode, step, chunk, inference_ms, stamp=None):
        self.introspection.append((episode, step, chunk, inference_ms, stamp))
