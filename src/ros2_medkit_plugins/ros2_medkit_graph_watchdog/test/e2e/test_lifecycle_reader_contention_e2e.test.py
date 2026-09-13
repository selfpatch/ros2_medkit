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
One lifecycle-state reader serves everybody, and nobody waits behind anybody.

The gateway's ``/status`` handler and this plugin's lifecycle watcher read
managed nodes through the same reader object. That is deliberate - the reader
owns a private ROS node named after the gateway, so a second one would collide
on the graph - but it puts an HTTP request and a plugin tick on the same object,
and the plugin's seed loop talks to nodes that may never answer.

The scenario builds exactly that: ``unreadable_lifecycle`` advertises
``get_state`` and never replies, so every seed against it spends the reader's
full timeout, and the watcher re-seeds it forever because its label never
becomes known. Meanwhile ``managed_lifecycle_active`` answers at once. A
``/status`` read of the answering node must not pay for the unanswering one.

Numbers, all measured on this branch on a developer box, one gateway, the
launch below:

- watcher holding its own reader, i.e. no sharing at all: max ``/status``
  latency 8 ms over 277 samples, median 4 ms
- one shared reader whose mutex spans the request and the inline spin: max
  489 ms over 190 samples, median 4 ms. That maximum is the reader's own 500 ms
  timeout, spent on a node the caller never asked about
- one shared reader whose mutex covers only client creation and destruction,
  five runs: maxima 6, 7, 7, 88 and 97 ms, median 4 ms every time

The maxima in that last row are the floor this bound has to clear, and the
occasional ~90 ms one is not the reader: a read that queues behind a seed pays
the reader's whole remaining timeout, which is why the regression row is a clean
489 ms rather than a spread. 250 ms is between the two, about 2.5x the worst
measured noise and half the regression, and `get_time_scale` stretches it where
the noise is worst.

The statistic is the maximum on purpose. The regression makes only a handful of
the samples slow - the median stays at 4 ms in the contended row above - so a
percentile rule would let it through.
"""

import os
import statistics
import sys
import time
import unittest

import launch_testing
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above, so this
# import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    create_watchdog_test_launch,
    wait_until_watchdog_armed,
)

from ros2_medkit_test_utils.constants import (  # noqa: E402
    ALLOWED_EXIT_CODES,
    get_test_port,
    get_time_scale,
)

PORT = get_test_port()

# 200 ms ticks put an unmeasured node's re-seed every
# LifecycleWatcher::kUnmeasuredSeedInterval (5) ticks, i.e. once a second. Each
# of those seeds spends the reader's 500 ms timeout against a node that never
# replies, so roughly half of the sampling window below overlaps a seed.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3

# Answers GetState immediately.
RESPONSIVE_APP = 'managed_lifecycle_active'
# Advertises get_state and never answers it.
UNREADABLE_APP = 'unreadable_lifecycle'

SAMPLE_WINDOW_SEC = 15.0 * get_time_scale()
SAMPLE_INTERVAL_SEC = 0.05
REQUEST_TIMEOUT_SEC = 10.0 * get_time_scale()

# The reader's own default GetState timeout, which is what a caller queued
# behind one unanswering seed pays on top of its own read.
READER_TIMEOUT_SEC = 0.5

# Half the reader timeout: comfortably above a read that only pays for itself,
# and unreachable by one that also pays for a seed against a node that never
# answers. Scales with the sanitizer factor because every term in a served
# request does.
MAX_STATUS_LATENCY_SEC = (READER_TIMEOUT_SEC / 2) * get_time_scale()


def generate_test_description():
    return create_watchdog_test_launch(
        detector_params={
            'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
            'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
        },
        demo_nodes=[RESPONSIVE_APP, UNREADABLE_APP],
        port=PORT,
    )


class TestLifecycleReaderContentionE2e(unittest.TestCase):
    """A shared reader must not serialise one caller behind another's timeout."""

    def _status_latencies(self, app_id):
        url = f'http://127.0.0.1:{PORT}{API_BASE_PATH}/apps/{app_id}/status'
        latencies = []
        deadline = time.monotonic() + SAMPLE_WINDOW_SEC
        while time.monotonic() < deadline:
            started = time.monotonic()
            response = requests.get(url, timeout=REQUEST_TIMEOUT_SEC)
            elapsed = time.monotonic() - started
            self.assertEqual(
                response.status_code, 200,
                f'GET {url} returned {response.status_code}: {response.text[:200]}',
            )
            latencies.append(elapsed)
            time.sleep(SAMPLE_INTERVAL_SEC)
        return latencies

    def test_status_does_not_queue_behind_a_watchdog_seed(self):
        # Gating on the responsive app by name, not just on "some app armed":
        # it is what proves the gateway has discovered it before the first
        # /status read, and that the watcher is tracking lifecycle nodes.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, app_id=RESPONSIVE_APP),
            f"the watchdog never armed on '{RESPONSIVE_APP}', so its lifecycle "
            'watcher never seeded anything',
        )
        self.assertTrue(
            wait_until_watchdog_armed(PORT, app_id=UNREADABLE_APP),
            f"the watchdog never armed on '{UNREADABLE_APP}', so nothing was "
            'contending for the reader',
        )

        # Anti-vacuity: without a responsive managed node this case would time a
        # branch that never reads lifecycle state at all.
        ready = requests.get(
            f'http://127.0.0.1:{PORT}{API_BASE_PATH}/apps/{RESPONSIVE_APP}/status',
            timeout=REQUEST_TIMEOUT_SEC,
        )
        self.assertEqual(ready.status_code, 200, ready.text[:200])
        self.assertEqual(
            ready.json().get('status'), 'ready',
            f'{RESPONSIVE_APP} must be an active managed node for this case to '
            f'exercise the reader at all; got {ready.text[:200]}',
        )

        latencies = self._status_latencies(RESPONSIVE_APP)
        self.assertGreater(len(latencies), 50, 'too few samples to say anything')

        worst = max(latencies)
        # Printed on every run, not only on failure: the bound is a measurement,
        # and a run that passes with no margin is worth seeing before it fails.
        print(
            f'/status latency over {len(latencies)} samples: '
            f'max {worst * 1000:.0f}ms, median {statistics.median(latencies) * 1000:.0f}ms, '
            f'bound {MAX_STATUS_LATENCY_SEC * 1000:.0f}ms',
            flush=True,
        )
        self.assertLess(
            worst, MAX_STATUS_LATENCY_SEC,
            f'slowest of {len(latencies)} /status reads took {worst:.3f}s '
            f'(median {statistics.median(latencies):.3f}s), over the '
            f'{MAX_STATUS_LATENCY_SEC:.3f}s bound: a read paid for the watchdog '
            f"seed against '{UNREADABLE_APP}', which never answers",
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
