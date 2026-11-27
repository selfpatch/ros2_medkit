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

"""Helpers shared by the policy bridge launch tests."""

import os
import time

import rclpy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults
from ros2_medkit_policy_msgs.msg import PolicyIntrospection

TOPIC = '/policy/introspection'
LOW_CODE = 'POLICY_CHUNK_DISAGREEMENT'
SLOW_CODE = 'POLICY_SLOW_INFERENCE'
CHUNK, DIM = 10, 2
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]

# What the fault manager needs so that the bridge's single PASSED heals.
HEALING_FAULT_MANAGER = {
    'storage_type': 'memory',
    'confirmation_threshold': -1,
    'healing_enabled': True,
    'healing_threshold': 0,
}


def time_scale():
    """MEDKIT_TEST_TIME_SCALE as ros2_medkit_test_utils reads it: below 1 or unparseable is 1."""
    try:
        scale = float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1'))
    except ValueError:
        return 1.0
    return scale if scale >= 1.0 else 1.0


def message(episode, step, noise, inference_ms=0.0):
    """
    One inference of a synthetic policy.

    A confident policy predicts the same value for the same future step every
    time; `noise` shifts this whole chunk, so two chunks differ by the
    difference of their noise. inference_ms 0 is the message default: not
    measured.
    """
    msg = PolicyIntrospection()
    msg.episode = episode
    msg.step = step
    msg.chunk_length = CHUNK
    msg.action_dim = DIM
    msg.chunk = [0.01 * (step + t) + 0.1 * j + noise for t in range(CHUNK) for j in range(DIM)]
    msg.inference_ms = inference_ms
    return msg


class FaultReader:
    """Reads faults from a real fault manager through ListFaults."""

    def __init__(self, node):
        self.node = node
        self.client = node.create_client(ListFaults, '/fault_manager/list_faults')

    def wait_for_service(self, timeout):
        return self.client.wait_for_service(timeout_sec=timeout)

    def get(self, code):
        """Return the fault manager's record of `code` in any status, or None."""
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = ALL_STATUSES
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0 * time_scale())
        result = future.result()
        if result is None:
            return None
        return next((f for f in result.faults if f.fault_code == code), None)

    def wait_for(self, code, status, timeout, since=None):
        """
        Wait until `code` is in `status`.

        With `since` (wall-clock seconds), also until it last occurred at or
        after that time: the fault manager stamps faults with its wall clock,
        and a re-raise of a HEALED fault moves last_occurred, not
        occurrence_count.
        """
        deadline = time.monotonic() + timeout
        fault = None
        while time.monotonic() < deadline:
            fault = self.get(code)
            if fault is not None and fault.status == status and \
                    (since is None or stamp(fault.last_occurred) >= since):
                return fault
            time.sleep(0.2)
        return fault


def stamp(t):
    """Return a builtin_interfaces/Time as seconds."""
    return t.sec + t.nanosec * 1e-9
