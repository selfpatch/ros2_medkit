#!/usr/bin/env python3
# Copyright 2026 bburda
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

"""An entity's rosbag listing keeps a recording whatever state its fault is in.

A recording is made when its fault confirms and outlives that state. The
download of a recording checks only that the fault is in the entity's scope,
so the listing must not drop a recording when its fault is pending again
(PREFAILED or PREPASSED), cleared, healed or muted as a correlated symptom.

The debounce counter starts at -1, confirms at -2 and heals at +2; a
confirmed fault stays CONFIRMED until the counter reaches +2. A fault
reactivated after a clear starts again at -1, so two PASSED reports move it
to +1, PREPASSED. The fault manager mutes ``BAGSYM`` as a symptom of
``BAGROOT`` and keeps a recording when its fault is cleared
(``snapshots.rosbag.auto_cleanup`` off). Every
fault is reported from a path child of the ``/bagstates/bag_owner`` node,
which this process creates, so they all belong to its app.
"""

import os
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, FAULT_TIMEOUT, ROSBAG_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)

from std_msgs.msg import Float32


OWNER_APP = 'bag_owner'
SOURCE = '/bagstates/bag_owner/diag'
SIGNAL_TOPIC = '/bagstates/signal'

PREFAILED = 'STATE_PREFAILED'
PREPASSED = 'STATE_PREPASSED'
CLEARED = 'STATE_CLEARED'
HEALED = 'STATE_HEALED'
ROOT = 'BAGROOT'
MUTED = 'BAGSYM'

CORRELATION_YAML = """\
correlation:
  enabled: true
  default_window_ms: 30000
  rules:
    - id: bag_cascade
      name: Bag cascade
      mode: hierarchical
      root_cause:
        codes: ["BAGROOT"]
      symptoms:
        - codes: ["BAGSYM"]
      window_ms: 30000
      mute_symptoms: true
      auto_clear_with_root: false
"""


def _write_correlation_config():
    fd, path = tempfile.mkstemp(prefix='medkit_bag_states_', suffix='.yaml')
    os.write(fd, CORRELATION_YAML.encode('utf-8'))
    os.close(fd)
    return path


def generate_test_description():
    gateway = create_gateway_node(extra_params={'discovery.mode': 'runtime_only'})
    fault_manager = create_fault_manager_node(
        rosbag_topics=[SIGNAL_TOPIC],
        snapshot_topics=[SIGNAL_TOPIC],
        extra_params={
            'confirmation_threshold': -2,
            'healing_enabled': True,
            'healing_threshold': 2,
            # The default deletes a fault's recording when the fault is cleared.
            'snapshots.rosbag.auto_cleanup': False,
            'correlation.config_file': _write_correlation_config(),
        },
    )
    return (
        LaunchDescription([
            gateway,
            TimerAction(period=2.0, actions=[fault_manager]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestRosbagListFaultStates(GatewayTestCase):
    """The rosbag listing follows fault scope, not fault status."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {OWNER_APP}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._owner = Node(OWNER_APP, namespace='/bagstates')
        cls._report_client = cls._owner.create_client(ReportFault, '/fault_manager/report_fault')
        cls._clear_client = cls._owner.create_client(ClearFault, '/fault_manager/clear_fault')
        # The capture ring buffer needs data before a fault confirms, and the
        # fault manager resolves an explicit topic's type only at startup.
        signal = cls._owner.create_publisher(Float32, SIGNAL_TOPIC, 10)
        cls._owner.create_timer(0.05, lambda: signal.publish(Float32(data=1.0)))
        cls._executor = SingleThreadedExecutor()
        cls._executor.add_node(cls._owner)
        cls._spin_thread = threading.Thread(target=cls._executor.spin, daemon=True)
        cls._spin_thread.start()

        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=30.0), \
            'report_fault service not available'
        # A reply can be lost while the fault manager is still matching this
        # client, so the first round trip is retried until it is answered.
        deadline = time.monotonic() + FAULT_TIMEOUT
        while not cls._call(cls._report_client, cls._report_request(
                'BAGSTATES_HANDSHAKE', '/bagstates/handshake', passed=False), 2.0):
            assert time.monotonic() < deadline, 'report_fault never answered'

    @classmethod
    def tearDownClass(cls):
        cls._executor.shutdown()
        cls._spin_thread.join(timeout=5)
        cls._owner.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _report_request(code, source, passed):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = (ReportFault.Request.EVENT_PASSED if passed
                              else ReportFault.Request.EVENT_FAILED)
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'rosbag listing state test'
        request.source_id = source
        return request

    @staticmethod
    def _call(client, request, answer_within):
        future = client.call_async(request)
        deadline = time.monotonic() + answer_within
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _report(self, code, times=1, passed=False):
        for _ in range(times):
            answer = self._call(self._report_client,
                                self._report_request(code, SOURCE, passed), 10.0)
            self.assertTrue(answer is not None and answer.accepted, f'report {code} failed')

    def _clear(self, code):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = SOURCE
        answer = self._call(self._clear_client, request, 10.0)
        self.assertTrue(answer is not None and answer.success, f'clear {code} failed')

    def _status(self, code):
        """Status the fault manager holds for ``code``, read without a list filter."""
        detail = self.get_json(f'/apps/{OWNER_APP}/faults/{code}')
        return detail.get('x-medkit', {}).get('status_raw')

    def _bag_codes(self):
        data = self.get_json(f'/apps/{OWNER_APP}/bulk-data/rosbags')
        return {code for item in data.get('items', [])
                for code in item.get('x-medkit', {}).get('fault_codes', [])}

    def _wait_for_bags(self, codes):
        deadline = time.monotonic() + ROSBAG_TIMEOUT
        while time.monotonic() < deadline:
            listed = self._bag_codes()
            if codes <= listed:
                return
            time.sleep(0.5)
        self.fail(f'bags of {sorted(codes - listed)} never listed; listed {sorted(listed)}')

    # @verifies REQ_INTEROP_072
    def test_recordings_stay_listed_in_every_fault_state(self):
        """Pending, cleared, healed and muted faults keep their recordings listed."""
        for code in (PREFAILED, PREPASSED, CLEARED, HEALED, ROOT, MUTED):
            self._report(code, times=2)
        every = {PREFAILED, PREPASSED, CLEARED, HEALED, MUTED}
        self._wait_for_bags(every)

        # PREFAILED: a new outage after a clear, one FAILED short of confirming.
        self._clear(PREFAILED)
        self._report(PREFAILED)
        # PREPASSED: the same new outage, then two PASSED reports, one short of healing.
        self._clear(PREPASSED)
        self._report(PREPASSED)
        self._report(PREPASSED, times=2, passed=True)
        self._clear(CLEARED)
        self._report(HEALED, times=4, passed=True)

        # Preconditions: each fault is in the state this test is about. The
        # detail route does not go through the list status filter.
        self.assertEqual(
            {code: self._status(code) for code in (PREFAILED, PREPASSED, CLEARED, HEALED)},
            {PREFAILED: 'PREFAILED', PREPASSED: 'PREPASSED', CLEARED: 'CLEARED',
             HEALED: 'HEALED'})
        faults = self.get_json('/faults?status=all&include_muted=true')
        muted = {m.get('fault_code') for m in faults.get('x-medkit', {}).get('muted_faults', [])}
        self.assertIn(MUTED, muted)

        listed = self._bag_codes()
        self.assertTrue(every <= listed,
                        f'bags of {sorted(every - listed)} dropped; listed {sorted(listed)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
