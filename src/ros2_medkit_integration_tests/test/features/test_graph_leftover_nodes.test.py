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

"""Discovery hides the leftover of a node it saw running, and lists everything else.

``ghost_node_injector --leftover`` leaves a real node's leftover (empty enclave, no
endpoints) in the graph. Each case first checks that the node ran and that this process's
graph then lists only the leftover. ``--ghost`` and ``--backed`` build the same shape for
names the gateway never saw, which stay listed.

All cases share one gateway and use names of their own. The last case rechecks the earlier
leftovers after the hold, so it must run last.
"""

import os
import signal
import subprocess
import time
import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import rclpy
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_BASE_URL,
    DEFAULT_DOMAIN_ID,
    get_time_scale,
)
from ros2_medkit_test_utils.graph_fixtures import (
    fixture_path,
    GhostInjection,
    LeftoverNode,
    observed_enclaves,
    split_fqn,
    stop_process,
    wait_observed,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, get_coverage_env

TIME_SCALE = get_time_scale()

# Backstop refresh, so the observation windows below count refreshes that ran.
REFRESH_INTERVAL_MS = 500
REFRESH_DEBOUNCE_MS = 500
# How long a claim is watched: three refreshes. A window, not a give-up bound, so not scaled.
OBSERVE_SEC = 3 * REFRESH_INTERVAL_MS / 1000.0
# Give-up bounds for the fixture (30 s of its own), DDS discovery and process start or exit.
INJECTION_TIMEOUT_SEC = 45.0 * TIME_SCALE
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
PROCESS_EXIT_TIMEOUT_SEC = 30.0 * TIME_SCALE
# From this process's graph listing a node to GET /apps listing it: the debounce, the 100 ms
# graph poll, and up to 4 s by which the gateway's discovery can trail on a loaded host.
LISTED_AFTER_OBSERVED_SEC = ((REFRESH_DEBOUNCE_MS + 100) / 1000.0 + 4.0) * TIME_SCALE
# GraphNodeListReader::kDefaultHold.
LEFTOVER_HOLD_SEC = 10.0
# Delay of the late sample for leftovers that arrive while the gateway remembers the node.
LEFTOVER_DELAY_SEC = 1.0
# Inside the hold but six refreshes after the removal: catches forgetting after a few refreshes.
INSIDE_HOLD_DELAY_SEC = 3.0
# From GET /apps first not listing a departed node until a refresh has run past the hold.
PAST_THE_HOLD_SEC = LEFTOVER_HOLD_SEC + LISTED_AFTER_OBSERVED_SEC
# A --delay longer than any run of this file: the late sample goes out on `publish`.
PUBLISH_ON_COMMAND_DELAY_SEC = 3600.0
# How many leftovers the many-leftovers case leaves behind next to the live nodes.
MANY_LEFTOVERS = 50

# Departs in setUpClass; the last case sends its late sample after the hold.
AFTER_HOLD_FQN = '/leftover_hold_ns/leftover_after'


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'refresh_interval_ms': REFRESH_INTERVAL_MS,
            'discovery.refresh_debounce_ms': REFRESH_DEBOUNCE_MS,
        },
    )
    return (
        LaunchDescription([gateway_node, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway_node},
    )


def _fixture_env():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
    env.update(get_coverage_env())
    return env


def _app_id(fqn):
    return split_fqn(fqn)[0]


def _app_ids():
    response = requests.get(f'{DEFAULT_BASE_URL}/apps', timeout=10)
    response.raise_for_status()
    return [item.get('id') for item in response.json().get('items', [])]


def _poll_app_listed(app_id, timeout, listed=True):
    """Time at which GET /apps first matched `listed` for `app_id`, or None."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if (app_id in _app_ids()) == listed:
                return time.monotonic()
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.1)
    return None


class TestGraphLeftoverNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while True:
            try:
                if requests.get(f'{DEFAULT_BASE_URL}/health', timeout=2).status_code == 200:
                    break
            except requests.exceptions.RequestException:
                pass
            if time.monotonic() > deadline:
                raise AssertionError('the gateway never answered GET /health')
            time.sleep(0.2)
        rclpy.init()
        cls.observer = rclpy.create_node('_graph_leftover_observer')
        # (fqn, App id, when GET /apps first stopped listing it) per hidden leftover.
        cls.hidden_leftovers = []

        # Departs now with no entry left; its late sample waits for the last case.
        cls.after_hold = LeftoverNode(
            AFTER_HOLD_FQN, PUBLISH_ON_COMMAND_DELAY_SEC, env=_fixture_env())
        if not cls.after_hold.wait_ready(INJECTION_TIMEOUT_SEC):
            raise AssertionError(
                f'ghost_node_injector never got {AFTER_HOLD_FQN} running:\n'
                f'{cls.after_hold.output()}')
        if _poll_app_listed(_app_id(AFTER_HOLD_FQN), APPEAR_TIMEOUT_SEC) is None:
            raise AssertionError(f'{AFTER_HOLD_FQN} runs but never appeared in GET /apps')
        cls.after_hold.leave()
        cls.after_hold_departed = _poll_app_listed(
            _app_id(AFTER_HOLD_FQN), LISTED_AFTER_OBSERVED_SEC + PROCESS_EXIT_TIMEOUT_SEC,
            listed=False)
        if cls.after_hold_departed is None:
            raise AssertionError(f'{AFTER_HOLD_FQN} left, but GET /apps kept listing it')

    @classmethod
    def tearDownClass(cls):
        cls.after_hold.stop(PROCESS_EXIT_TIMEOUT_SEC)
        cls.observer.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self._processes = []
        self._fixtures = []

    def tearDown(self):
        for fixture in self._fixtures:
            fixture.stop(PROCESS_EXIT_TIMEOUT_SEC)
        for proc in self._processes:
            stop_process(proc, PROCESS_EXIT_TIMEOUT_SEC)

    # ---- fixtures -----------------------------------------------------------------------

    def _start_node(self, fqn, executable='demo_rpm_sensor'):
        name, namespace = split_fqn(fqn)
        proc = subprocess.Popen(
            [fixture_path(executable), '--ros-args', '-r', f'__ns:={namespace}',
             '-r', f'__node:={name}'],
            env=_fixture_env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._processes.append(proc)
        return proc

    def _start_leftover(self, fqn, delay_sec=LEFTOVER_DELAY_SEC, announce=0):
        """Start a node that can leave a leftover, and wait until GET /apps lists it."""
        leftover = LeftoverNode(fqn, delay_sec, announce=announce, env=_fixture_env())
        self._fixtures.append(leftover)
        self.assertTrue(
            leftover.wait_ready(INJECTION_TIMEOUT_SEC),
            f'ghost_node_injector never got {fqn} running:\n{leftover.output()}')
        self.assertTrue(
            self._wait_listed(_app_id(fqn), APPEAR_TIMEOUT_SEC),
            f'{fqn} runs but never appeared in GET /apps, so the gateway never saw it running')
        return leftover

    def _wait_left_over(self, fqn, timeout):
        """Assert this process's graph lists `fqn` only as a leftover (empty enclave)."""
        self.assertTrue(
            wait_observed(self.observer, fqn, lambda enclaves: enclaves == [''], timeout),
            f'after its participant left, the test process graph lists {fqn} with enclaves '
            f'{observed_enclaves(self.observer, fqn)} rather than once with an empty one, so '
            'no leftover was reproduced')

    def _hidden_since(self, fqn, app_id):
        """Wait until GET /apps does not list `app_id`; record the leftover for the last case."""
        hidden_at = _poll_app_listed(app_id, LISTED_AFTER_OBSERVED_SEC, listed=False)
        self.assertIsNotNone(hidden_at, f'the leftover of {fqn} is listed as {app_id}')
        type(self).hidden_leftovers.append((fqn, app_id, hidden_at))

    # ---- observations -------------------------------------------------------------------

    @staticmethod
    def _ids(collection):
        response = requests.get(f'{DEFAULT_BASE_URL}/{collection}', timeout=10)
        response.raise_for_status()
        return [item.get('id') for item in response.json().get('items', [])]

    @staticmethod
    def _wait_listed(app_id, timeout, listed=True):
        return _poll_app_listed(app_id, timeout, listed) is not None

    @staticmethod
    def _warnings_for(proc_output, gateway_node, fqn):
        text = ''.join(
            output.text.decode(errors='replace') for output in proc_output[gateway_node])
        return text.count(f"Node '{fqn}' is not exposed")

    def _wait_warnings(self, proc_output, gateway_node, fqn, expected,
                       timeout=LISTED_AFTER_OBSERVED_SEC):
        deadline = time.monotonic() + timeout
        count = self._warnings_for(proc_output, gateway_node, fqn)
        while count < expected and time.monotonic() < deadline:
            time.sleep(0.2)
            count = self._warnings_for(proc_output, gateway_node, fqn)
        return count

    @staticmethod
    def _observe(check):
        """Run `check` every 250 ms for OBSERVE_SEC; it fails the test on the first bad sample."""
        deadline = time.monotonic() + OBSERVE_SEC
        while time.monotonic() < deadline:
            check()
            time.sleep(0.25)

    # ---- cases --------------------------------------------------------------------------

    def test_01_leftover_of_a_listed_node_inside_the_hold_is_hidden(
            self, proc_output, gateway_node):
        fqn = '/leftover_solo_ns/leftover_solo'
        leftover = self._start_leftover(fqn, delay_sec=INSIDE_HOLD_DELAY_SEC)
        self.assertIn(
            'leftover_solo_ns', self._ids('functions'),
            f'no Function was derived from the namespace of the running {fqn}, so its absence '
            'below would show nothing')

        leftover.leave()
        self.assertTrue(
            self._wait_listed('leftover_solo', LISTED_AFTER_OBSERVED_SEC, listed=False),
            f'{fqn} left, but GET /apps kept listing it, so the gateway never read the graph '
            'without it')
        self._wait_left_over(fqn, INSIDE_HOLD_DELAY_SEC + INJECTION_TIMEOUT_SEC)
        self._hidden_since(fqn, 'leftover_solo')

        def check():
            app_ids = self._ids('apps')
            self.assertNotIn(
                'leftover_solo', app_ids,
                f'the late sample for {fqn} arrived {INSIDE_HOLD_DELAY_SEC:.1f} s after its '
                f'participant left, inside the {LEFTOVER_HOLD_SEC:.0f} s hold, and is listed')
            detail = requests.get(f'{DEFAULT_BASE_URL}/apps/leftover_solo', timeout=10)
            self.assertEqual(
                detail.status_code, 404,
                f'GET /apps/leftover_solo answered {detail.status_code} for a leftover')
            self.assertNotIn(
                'leftover_solo_ns', self._ids('functions'),
                f'a Function is still derived from the namespace only the leftover of {fqn} '
                'occupies')

        self._observe(check)
        self.assertEqual(
            observed_enclaves(self.observer, fqn), [''],
            f'the leftover of {fqn} left the test process graph during the observation, so the '
            'gateway was not holding it out')
        self.assertEqual(
            self._wait_warnings(proc_output, gateway_node, fqn, 1), 1,
            f'the gateway must warn exactly once about the leftover of {fqn}')

    def test_02_nodes_the_gateway_never_saw_leave_are_listed(self, proc_output, gateway_node):
        far = '/ghost_far_ns/ghost_far'
        twin = '/ghost_twin_ns/ghost_twin'
        quiet = '/quiet_ns/quiet'
        # A never-seen node without an enclave; the same next to a backed twin whose GID sorts
        # after the ghost's; and a running node with no endpoints.
        injection = GhostInjection(ghosts=[far, twin], backed=[twin], env=_fixture_env())
        self._fixtures.append(injection)
        self._start_node(quiet, executable='endpointless_node')
        self.assertIsNotNone(
            injection.wait_status(INJECTION_TIMEOUT_SEC),
            f'ghost_node_injector never published:\n{injection.output()}')
        for fqn, enclaves in ((far, ['']), (twin, ['', '']), (quiet, ['/'])):
            self.assertTrue(
                wait_observed(self.observer, fqn, lambda found, want=enclaves: found == want,
                              INJECTION_TIMEOUT_SEC),
                f'the test process graph lists {fqn} with {observed_enclaves(self.observer, fqn)}'
                f', not {enclaves}')
        name, namespace = split_fqn(quiet)
        endpoints = (
            self.observer.get_publisher_names_and_types_by_node(name, namespace, True)
            + self.observer.get_subscriber_names_and_types_by_node(name, namespace, True))
        self.assertEqual(
            endpoints, [],
            f'the fixture {quiet} is supposed to have no endpoints of its own, so this case '
            f'would not show that endpoints are not the rule: {endpoints}')
        ids = {'ghost_far', 'ghost_twin', 'quiet'}
        deadline = time.monotonic() + LISTED_AFTER_OBSERVED_SEC
        while ids - set(self._ids('apps')) and time.monotonic() < deadline:
            time.sleep(0.1)
        self.assertEqual(
            ids - set(self._ids('apps')), set(),
            'these nodes are not leftovers of nodes the gateway saw running, and never appeared '
            'in GET /apps')

        def check():
            self.assertEqual(ids - set(self._ids('apps')), set(),
                             'a node dropped out of GET /apps')

        self._observe(check)
        for fqn in (far, twin, quiet):
            self.assertEqual(self._warnings_for(proc_output, gateway_node, fqn), 0)

    def test_03_leftover_then_the_node_runs_again_is_listed_once(
            self, proc_output, gateway_node):
        fqn = '/leftover_again_ns/leftover_again'
        leftover = self._start_leftover(fqn)
        leftover.leave()
        self._wait_left_over(fqn, LEFTOVER_DELAY_SEC + INJECTION_TIMEOUT_SEC)
        self.assertTrue(
            self._wait_listed('leftover_again', LISTED_AFTER_OBSERVED_SEC, listed=False),
            f'the leftover of {fqn} is listed')
        self.assertEqual(self._wait_warnings(proc_output, gateway_node, fqn, 1), 1)

        self._start_node(fqn)
        self.assertTrue(
            wait_observed(self.observer, fqn, lambda enclaves: sorted(enclaves) == ['', '/'],
                          APPEAR_TIMEOUT_SEC),
            f'the test process graph does not list {fqn} running next to its leftover: '
            f'{observed_enclaves(self.observer, fqn)}')
        self.assertTrue(
            self._wait_listed('leftover_again', LISTED_AFTER_OBSERVED_SEC),
            f'{fqn} runs again but never appeared in GET /apps')

        def check():
            app_ids = self._ids('apps')
            self.assertEqual(
                app_ids.count('leftover_again'), 1,
                f'{fqn} runs next to its leftover and must be listed exactly once: {app_ids}')

        self._observe(check)
        self.assertEqual(self._warnings_for(proc_output, gateway_node, fqn), 1)

    def test_04_leftover_does_not_rename_a_live_node_sharing_its_bare_name(
            self, proc_output, gateway_node):
        live = '/leftover_collide_b/leftover_collider'
        fqn = '/leftover_collide_a/leftover_collider'
        self._start_node(live)
        self.assertTrue(self._wait_listed('leftover_collider', APPEAR_TIMEOUT_SEC),
                        f'the live node {live} never appeared in GET /apps')
        leftover = LeftoverNode(fqn, LEFTOVER_DELAY_SEC, env=_fixture_env())
        self._fixtures.append(leftover)
        self.assertTrue(leftover.wait_ready(INJECTION_TIMEOUT_SEC), leftover.output())
        self.assertTrue(
            self._wait_listed('leftover_collide_a_leftover_collider', APPEAR_TIMEOUT_SEC),
            'while both nodes run, the gateway never gave them namespace-prefixed ids, so the '
            'collision rule this case depends on did not engage')

        leftover.leave()
        self._wait_left_over(fqn, LEFTOVER_DELAY_SEC + INJECTION_TIMEOUT_SEC)
        self.assertTrue(
            self._wait_listed('leftover_collider', LISTED_AFTER_OBSERVED_SEC),
            f'{live} never got its un-prefixed id back after the other node left')
        # Only the gateway's own read logs this, so its graph holds the leftover too.
        self.assertEqual(
            self._wait_warnings(proc_output, gateway_node, fqn, 1), 1,
            f'the gateway never warned about the leftover of {fqn}, so its graph may never have '
            'listed it and the ids below would show nothing')
        hidden_at = time.monotonic()

        def check():
            app_ids = self._ids('apps')
            self.assertIn(
                'leftover_collider', app_ids,
                f'{live} lost its un-prefixed App id to the leftover of {fqn}: {app_ids}')
            self.assertFalse(
                [app_id for app_id in app_ids if app_id.endswith('_leftover_collider')],
                f'a namespace-prefixed id was derived for a collision with a leftover: {app_ids}')

        self._observe(check)
        # Once the live node is gone, the leftover would take the un-prefixed id.
        type(self).hidden_leftovers.append((fqn, 'leftover_collider', hidden_at))

    def test_05_node_seen_running_that_stays_behind_an_endpoint_is_listed(
            self, proc_output, gateway_node):
        fqn = '/bridged_ns/bridged'
        marker = '/bridged_ns/bridged_marker'
        node = self._start_node(fqn)
        self.assertTrue(self._wait_listed('bridged', APPEAR_TIMEOUT_SEC),
                        f'{fqn} never appeared in GET /apps')
        # A backed entry of the same name, published while the node runs; it is all that is
        # left once the node exits. The marker ghost goes out after it.
        injection = GhostInjection(ghosts=[marker], backed=[fqn], env=_fixture_env())
        self._fixtures.append(injection)
        self.assertIsNotNone(injection.wait_status(INJECTION_TIMEOUT_SEC), injection.output())
        self.assertTrue(
            wait_observed(self.observer, fqn, lambda enclaves: sorted(enclaves) == ['', '/'],
                          INJECTION_TIMEOUT_SEC),
            f'the test process graph does not list {fqn} both running and without an enclave: '
            f'{observed_enclaves(self.observer, fqn)}')
        # The gateway's discovery of the injector can trail this process's by seconds.
        self.assertTrue(
            self._wait_listed(_app_id(marker), APPEAR_TIMEOUT_SEC),
            f'GET /apps never listed {marker}, so the gateway graph may not hold the backed '
            f'entry of {fqn} when the node exits')

        node.send_signal(signal.SIGTERM)
        node.wait(timeout=PROCESS_EXIT_TIMEOUT_SEC)
        self.assertTrue(
            wait_observed(self.observer, fqn, lambda enclaves: enclaves == [''],
                          APPEAR_TIMEOUT_SEC),
            f'after the node exited, the test process graph lists {fqn} with '
            f'{observed_enclaves(self.observer, fqn)}')
        name, namespace = split_fqn(fqn)
        self.assertTrue(
            self.observer.get_publisher_names_and_types_by_node(name, namespace, True),
            f'{fqn} resolves no endpoint in the test process graph, so this case would not show '
            'that an endpoint keeps it listed')

        def check():
            self.assertIn(
                'bridged', self._ids('apps'),
                f'{fqn} was seen running and has an endpoint, but dropped out of GET /apps')

        self._observe(check)
        self.assertEqual(self._warnings_for(proc_output, gateway_node, fqn), 0)

    def test_06_many_leftovers_next_to_live_nodes(self, proc_output, gateway_node):
        live = [f'/leftover_many_live/live_{index}' for index in range(3)]
        for fqn in live:
            self._start_node(fqn)
        for fqn in live:
            self.assertTrue(self._wait_listed(_app_id(fqn), APPEAR_TIMEOUT_SEC),
                            f'the live node {fqn} never appeared in GET /apps')

        leftover = self._start_leftover(
            '/leftover_many_ns/leftover_many', announce=MANY_LEFTOVERS - 1)
        # Announce after GET /apps lists the node, or its own discovery message can replace it.
        self.assertTrue(leftover.announce_nodes(INJECTION_TIMEOUT_SEC), leftover.output())
        fqns = [leftover.fqn] + leftover.announced_fqns()
        ids = {_app_id(fqn) for fqn in fqns}
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while ids - set(self._ids('apps')) and time.monotonic() < deadline:
            time.sleep(0.2)
        self.assertEqual(
            ids - set(self._ids('apps')), set(),
            'the gateway never listed every announced node running, so it did not see them run')

        leftover.leave()
        for fqn in fqns:
            self._wait_left_over(fqn, LEFTOVER_DELAY_SEC + INJECTION_TIMEOUT_SEC)
        live_ids = {_app_id(fqn) for fqn in live}

        def check():
            app_ids = set(self._ids('apps'))
            self.assertEqual(live_ids - app_ids, set(), 'a live node dropped out of GET /apps')
            self.assertEqual(app_ids & ids, set(), 'leftovers are listed')
            self.assertNotIn('leftover_many_ns', self._ids('functions'))

        self._observe(check)

        late = '/leftover_many_live/late'
        self._start_node(late)
        self.assertTrue(
            wait_observed(self.observer, late, lambda enclaves: '/' in enclaves,
                          APPEAR_TIMEOUT_SEC),
            f'the test process graph never listed {late}')
        observed = time.monotonic()
        self.assertTrue(
            self._wait_listed('late', APPEAR_TIMEOUT_SEC),
            f'{late} never appeared in GET /apps next to {MANY_LEFTOVERS} leftovers')
        latency = time.monotonic() - observed
        self.assertLessEqual(
            latency, LISTED_AFTER_OBSERVED_SEC,
            f'{late} took {latency:.2f} s to reach GET /apps after the graph listed it, next '
            f'to {MANY_LEFTOVERS} leftovers')

        for fqn in fqns:
            self.assertEqual(
                self._wait_warnings(proc_output, gateway_node, fqn, 1), 1,
                f'the gateway must warn exactly once about the leftover of {fqn}')

    def test_07_the_hold_runs_out_only_for_a_name_no_entry_lists(
            self, proc_output, gateway_node):
        # Leftovers hidden by earlier cases, listed for longer than the hold, stay hidden: a
        # listed leftover keeps its name remembered.
        self.assertTrue(self.hidden_leftovers, 'no earlier case recorded a hidden leftover')
        oldest = min(hidden_at for _, _, hidden_at in self.hidden_leftovers)
        remaining = PAST_THE_HOLD_SEC - (time.monotonic() - oldest)
        if remaining > 0:
            time.sleep(remaining)
        now = time.monotonic()
        past_the_hold = [
            (fqn, app_id) for fqn, app_id, hidden_at in self.hidden_leftovers
            if now - hidden_at >= PAST_THE_HOLD_SEC]
        app_ids = set(self._ids('apps'))
        for fqn, app_id in past_the_hold:
            self.assertEqual(
                observed_enclaves(self.observer, fqn), [''],
                f'the test process graph no longer lists {fqn} as a leftover only')
            self.assertNotIn(
                app_id, app_ids,
                f'the leftover of {fqn} was hidden and is listed again as {app_id} while the '
                'graph still lists it: the gateway forgot the node after the hold')
            self.assertEqual(self._warnings_for(proc_output, gateway_node, fqn), 1)

        # With no entry left, the first refresh past the hold forgets the name (refreshes run
        # every 0.5 s here), so a later late sample is listed like a node never seen running.
        remaining = PAST_THE_HOLD_SEC - (time.monotonic() - self.after_hold_departed)
        if remaining > 0:
            time.sleep(remaining)
        self.after_hold.publish()
        self._wait_left_over(AFTER_HOLD_FQN, INJECTION_TIMEOUT_SEC)
        after_id = _app_id(AFTER_HOLD_FQN)
        self.assertTrue(
            self._wait_listed(after_id, LISTED_AFTER_OBSERVED_SEC),
            f'the late sample for {AFTER_HOLD_FQN} arrived more than {PAST_THE_HOLD_SEC:.1f} s '
            f'after it departed, past the {LEFTOVER_HOLD_SEC:.0f} s hold, so the gateway no '
            'longer remembers the node; it never appeared in GET /apps')

        def after_listed():
            self.assertIn(after_id, self._ids('apps'),
                          f'the leftover of {AFTER_HOLD_FQN}, past the hold, dropped out of '
                          'GET /apps')

        self._observe(after_listed)
        self.assertEqual(self._warnings_for(proc_output, gateway_node, AFTER_HOLD_FQN), 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
