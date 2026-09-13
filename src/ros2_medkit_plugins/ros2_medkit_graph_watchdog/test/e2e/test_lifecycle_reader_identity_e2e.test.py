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

"""
The gateway puts exactly one ``<gateway>_lifecycle_state_reader`` on the graph.

Reading a managed node's state needs a ROS node of its own, and that node is
named after the gateway. Two of them - the gateway's ``/status`` handler and
this plugin's lifecycle watcher - would therefore both claim the same fully
qualified name. ROS allows that and warns about it, and every graph query that
lists nodes then returns the name twice, which is a duplicate every reader of
the graph has to swallow. The reader is shared instead.

Counted with rclpy rather than through the gateway: ``GET /apps`` de-duplicates
by name, so it cannot see the second one. This test only asks the graph.
"""

import os
import sys
import time
import unittest

import launch_testing
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    create_watchdog_test_launch,
    wait_until_watchdog_armed,
)

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

PORT = get_test_port()

TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

READER_NODE_NAME = 'ros2_medkit_gateway_lifecycle_state_reader'

# The probe polls until the reader is on the graph at all, then keeps reading
# for a while: the second node, when there is one, is created by the plugin
# rather than by the REST server, so the two appear at different moments and a
# single read taken too early would miss the collision.
READER_VISIBLE_TIMEOUT_SEC = 60.0
SETTLE_SEC = 10.0
PROBE_INTERVAL_SEC = 0.5


def generate_test_description():
    return create_watchdog_test_launch(
        detector_params={
            'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
            'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        },
        # A managed node gives the plugin's lifecycle watcher something to read,
        # so the reader is exercised rather than merely constructed.
        demo_nodes=['managed_lifecycle'],
        port=PORT,
        # Without the __node:= remap. It is a global argument, so rclcpp applies
        # it to every node the gateway process creates and the reader would
        # answer to the gateway's own name instead of its suffixed one.
        gateway_name=None,
    )


class TestLifecycleReaderIdentityE2e(unittest.TestCase):
    """One gateway, one lifecycle-state reader node."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._probe = rclpy.create_node('lifecycle_reader_identity_probe')

    @classmethod
    def tearDownClass(cls):
        cls._probe.destroy_node()
        cls._probe = None
        rclpy.shutdown()

    def _node_names(self):
        return [name for name, _ in type(self)._probe.get_node_names_and_namespaces()]

    def _reader_count(self):
        return sum(1 for name in self._node_names() if name == READER_NODE_NAME)

    def test_reader_node_name_is_claimed_once(self):
        self.assertTrue(
            wait_until_watchdog_armed(PORT),
            'the watchdog plugin never armed, so its lifecycle watcher never ran',
        )

        deadline = time.monotonic() + READER_VISIBLE_TIMEOUT_SEC
        while time.monotonic() < deadline and self._reader_count() == 0:
            time.sleep(PROBE_INTERVAL_SEC)
        self.assertGreater(
            self._reader_count(), 0,
            f"'{READER_NODE_NAME}' never appeared on the graph, so this run proves "
            f'nothing; the graph held {sorted(self._node_names())}',
        )

        worst = 0
        settle_deadline = time.monotonic() + SETTLE_SEC
        while time.monotonic() < settle_deadline:
            worst = max(worst, self._reader_count())
            time.sleep(PROBE_INTERVAL_SEC)

        self.assertEqual(
            worst, 1,
            f"'{READER_NODE_NAME}' was on the graph {worst} times; the gateway's own "
            'reader and the plugin watcher must share one',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
