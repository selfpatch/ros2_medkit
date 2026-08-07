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
"""Lifecycle-expectation e2e: GRAPH_NODE_INACTIVE raises and heals through the REAL stack.

Unlike test_lifecycle_expectation_integration.cpp (which drives the detector against a
fake ReportFault service, a fixture-owned snapshot, and lifecycle labels injected via
ReliabilityGate::set_lifecycle_state_for_test() - it can only puppet the label, never
prove the real lifecycle machinery feeds it), this launches the REAL gateway process
with the graph_watchdog plugin (.so) loaded and a real fault_manager, and polls the
operator-visible ``GET /api/v1/faults`` surface while driving the demo
``managed_lifecycle`` node (``ros2_medkit_test_utils``' ``DEMO_NODE_REGISTRY``) through
REAL ``lifecycle_msgs/srv/ChangeState`` transitions.

Runs as THREE separate CTest targets (see CMakeLists.txt), each launching its OWN
gateway+fault_manager+demo-node stack: the plugin reads its config once at
set_context() time, so different configs need different gateway launches, not test
methods sharing one. WATCHDOG_E2E_SCENARIO selects which launch + assertions run:

- "main": ``require_active: ['managed_lifecycle']`` against the node in its default
  unconfigured state -> the fault raises (globally AND on the entity-scoped surface,
  naming the node), survives a real CONFIGURE (inactive is still not active), survives a
  real gateway RESTART (the withheld-clear guard: the fault_manager is a separate process
  and keeps the fault, while the restarted plugin starts with every detector counter at
  zero), and heals after a real ACTIVATE. This is the only scenario launched with
  ``gateway_respawn``.
- "default_config": NO ``detectors.lifecycle_expectation`` config at all, same demo
  node. The shipped default is an empty ``require_active``, documented as "nothing is
  checked, zero false positives" - this is the only test at any tier that could falsify
  that claim against a live unconfigured lifecycle node.
- "negative_control": ``require_active: ['managed_lifecycle_active']`` with the
  self-activating variant of the same executable. A required node that IS active must
  never raise, over a sustained window, under the same grace and cadence as "main" -
  the entry names the active variant, so the discriminating variable between this
  launch and "main" is the node's actual lifecycle state.

Both absence scenarios gate on three facts BEFORE asserting absence, because absence is
the default outcome of a stack that never came up and every bringup failure mode on this
launch still exits 0 (see the harness docstring): (1) the plugin is live and armed, via
its own GET /x-medkit-watchdog route (harness.wait_until_watchdog_armed); (2) the
gateway reaches the fault_manager in THIS launch, via GET /faults answering 200
(harness.wait_until_faults_endpoint_live) - the watchdog route is served inside the
gateway process and proves nothing about the fault surface the assertion reads, which
answers 503 and polls to None when the fault_manager is gone; (3) the target node's
lifecycle label was actually READ, via the same watchdog route - the tracker treats an
unread label as benign, so without it the silence could just as well mean the label
never arrived. Fact (3) is re-checked AFTER the window as well, since a trigger that
exits mid-window would leave most of the window measuring an empty graph.

The "main" scenario gates on the GLOBAL armed state, not on
``app_id='managed_lifecycle'``: the gate reports a tracked node with a KNOWN non-active
label as ``warming_up`` by design (ReliabilityGate::status_json's suppressed branch ANDs
LifecycleWatcher::node_ok, which is false for exactly the inactive node this detector
exists to catch), so a per-entity armed gate on the target would wait for the fault to
be impossible. The raise itself is gated on the SOURCE entity's arming (the aggregate is
raised under source 'graph_watchdog', see AggregatedFault), for which the global armed
state is the precise precondition. Once the node reaches "active" the per-entity gate
IS reachable, and the heal leg uses it to prove the real lifecycle machinery fed the
watcher before measuring the clear.
"""

import os
import signal
import sys
import time
import unittest

import launch_testing
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.node import Node
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# I100 as well as E402: `harness` is only importable because of the sys.path line above,
# so this import cannot be moved up to where the alphabetical order would put it.
from harness import (  # noqa: E402, I100
    API_BASE_PATH,
    create_watchdog_test_launch,
    poll_cleared,
    poll_entity_faults,
    poll_faults,
    wait_until_faults_endpoint_live,
    wait_until_watchdog_armed,
)

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port  # noqa: E402

# No default on purpose - a default makes this file FAIL OPEN (see
# test_config_plumbing_e2e.test.py's identical rationale): if the CTest ENVIRONMENT
# property never reaches the process, the launch and the assertions would degrade
# together into one scenario and still report 1/1 passed. A KeyError is loud.
SCENARIO = os.environ['WATCHDOG_E2E_SCENARIO']
PORT = get_test_port()

# Fast tick cadence + short warmup so the whole story (gateway/fault_manager startup,
# demo-node discovery, lifecycle label seeding, global bringup grace) comfortably fits
# inside the poll timeouts below - same rationale as the other e2e files here. grace is
# a few ticks so the raise is not instantaneous (the detector counts CONSECUTIVE
# not-active ticks) while staying well inside every poll timeout below.
TICK_INTERVAL_MS = 200
WARMUP_CYCLES = 3
GRACE = 3

FAULT_CODE = 'GRAPH_NODE_INACTIVE'
# Node names from DEMO_NODE_REGISTRY, which are also the App ids the gateway derives
# for nodes in the root namespace - so each constant is at once the launch key, the
# gate entity id, and what the fault description must name.
TARGET_NODE = 'managed_lifecycle'
ACTIVE_NODE = 'managed_lifecycle_active'
CHANGE_STATE_SERVICE = f'/{TARGET_NODE}/change_state'

_DETECTOR_PREFIX = 'plugins.graph_watchdog.detectors.lifecycle_expectation'

# How long a fault must stay ABSENT for the two silence scenarios. Counted from the
# arming gate, not process start, so bringup cannot eat it. At the 200 ms tick this is
# ~100 ticks against a grace of 3 (default 5): a detector that wrongly counted the
# node would have raised - and been CONFIRMED by the harness's
# confirmation_threshold: -2 fault_manager - dozens of times over.
SILENT_WINDOW_SEC = 20.0


def generate_test_description():
    detector_params = {
        'plugins.graph_watchdog.tick_interval_ms': TICK_INTERVAL_MS,
        'plugins.graph_watchdog.warmup_cycles': WARMUP_CYCLES,
    }
    demo_node = TARGET_NODE
    if SCENARIO == 'main':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [TARGET_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
    elif SCENARIO == 'negative_control':
        detector_params[f'{_DETECTOR_PREFIX}.require_active'] = [ACTIVE_NODE]
        detector_params[f'{_DETECTOR_PREFIX}.grace'] = GRACE
        demo_node = ACTIVE_NODE
    # default_config: deliberately NO detectors.lifecycle_expectation keys at all -
    # the launch under test is the shipped default itself.

    return create_watchdog_test_launch(
        detector_params=detector_params,
        # 'managed_lifecycle' stays unconfigured (auto_activate defaults to false):
        # present in the graph, alive, but not active - exactly the silent state this
        # detector exists to catch. 'managed_lifecycle_active' is the same executable
        # launched with auto_activate:=true, so it reaches "active" on its own.
        demo_nodes=[demo_node],
        port=PORT,
        # The clear must reach HEALED, i.e. leave the default active-fault query,
        # which the fault_manager only does when healing is enabled - see harness.py
        # and the README's "Closing the loop".
        healing_enabled=True,
        # Only "main" restarts the gateway (test_02b). The other two scenarios must
        # keep an unexpected gateway death visible as a failure.
        gateway_respawn=(SCENARIO == 'main'),
    )


def _fault_record(port, code, timeout=30.0, interval=0.5):
    """Poll ``GET /faults?status=all`` until `code` appears, whatever its status.

    ``poll_faults`` uses the default (pending+confirmed) filter, so a fault that
    HEALED disappears from it - which is indistinguishable from a fault that was
    never raised, and from a gateway that is up but has not reached the
    fault_manager yet. The restart test needs the record itself, including the
    fields that survive a heal, so it asks for every status.

    Returns the matching item dict, or ``None`` on timeout.
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/faults', params={'status': 'all'}, timeout=5)
            if response.status_code == 200:
                for item in response.json().get('items', []):
                    if item.get('fault_code') == code:
                        return item
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)
    return None


def _wait_until_port_is_down(port, timeout=60.0, interval=0.2):
    """Wait until the gateway's HTTP port stops answering. True once it does."""
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            requests.get(f'{base}/health', timeout=2)
        except requests.exceptions.RequestException:
            return True
        time.sleep(interval)
    return False


def _poll_watchdog_entity(port, app_id, lifecycle, timeout=30.0, interval=0.5):
    """Poll GET /x-medkit-watchdog until `app_id` appears with this lifecycle label.

    The absence scenarios need one fact wait_until_watchdog_armed cannot give them:
    that the target's lifecycle label was actually READ by the live stack. The armed
    gate treats an unread label as ok (node_ok defaults open for unknown state), and
    the tracker treats an unread label as benign - so a run in which the label never
    arrived produces the same silence the assertion is looking for. Pinning the label
    through the plugin's own status route closes that hole: 'unconfigured' proves the
    false-positive trigger was fully present, 'active' proves the negative control is
    actually exercising an active node rather than an unread one.

    Returns the matching entity dict, or ``None`` on timeout (after printing the
    last-seen payload, which is gone once the launch tears down).
    """
    base = f'http://127.0.0.1:{port}{API_BASE_PATH}'
    deadline = time.monotonic() + timeout
    last_seen = 'GET /x-medkit-watchdog was never answered at all'
    while time.monotonic() < deadline:
        try:
            response = requests.get(f'{base}/x-medkit-watchdog', timeout=5)
            if response.status_code != 200:
                last_seen = f'HTTP {response.status_code} from GET /x-medkit-watchdog'
            else:
                status = response.json().get('x-medkit-watchdog', {})
                last_seen = str(status)
                for entity in status.get('entities') or []:
                    if entity.get('id') == app_id and entity.get('lifecycle') == lifecycle:
                        return entity
        except requests.exceptions.RequestException as exc:
            last_seen = f'GET /x-medkit-watchdog failed: {exc}'
        time.sleep(interval)
    print(f'_poll_watchdog_entity(app_id={app_id!r}, lifecycle={lifecycle!r}) timed out '
          f'after {timeout}s; last watchdog status: {last_seen}')
    return None


class TestLifecycleExpectationMain(unittest.TestCase):
    """GRAPH_NODE_INACTIVE raises for a required-but-unconfigured node, heals on activate."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._client_node = Node('lifecycle_expectation_e2e_client')
        cls._change_state = cls._client_node.create_client(ChangeState, CHANGE_STATE_SERVICE)

    @classmethod
    def tearDownClass(cls):
        cls._client_node.destroy_node()
        rclpy.shutdown()

    def _call_transition(self, transition_id, reached_label, timeout=30.0, attempts=3):
        """Drive one real lifecycle transition, retrying a call that never comes back.

        The whole scenario hangs off two of these, and a single-shot call makes one lost
        response (or a demo-node executor stalled under a sanitizer at the wrong moment)
        fail the entire multi-minute run on one RPC. The retry does NOT widen the budget:
        the same total wait is split across `attempts`, so a genuinely dead service still
        fails after the same wall time, with the last attempt's outcome named.

        `reached_label` is the lifecycle state this transition ends in. It is needed
        because a retry introduces a case a single-shot call does not have: if the FIRST
        attempt was applied and only its response was lost, the retried transition is
        invalid from the state the node has already reached and comes back rejected. That
        is a success, not a failure, and the only way to tell it from a genuinely refused
        transition is to look at where the node actually is.
        """
        self.assertTrue(
            type(self)._change_state.wait_for_service(timeout_sec=timeout),
            f'{CHANGE_STATE_SERVICE} never became available',
        )
        per_attempt = timeout / attempts
        result = None
        retried = False
        for attempt in range(attempts):
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = type(self)._change_state.call_async(request)
            rclpy.spin_until_future_complete(
                type(self)._client_node, future, timeout_sec=per_attempt)
            result = future.result()
            if result is not None:
                break
            # Drop the request the server may still answer, so the next attempt's future
            # cannot be completed by a straggling response to this one.
            type(self)._change_state.remove_pending_request(future)
            retried = True
            print(f'ChangeState (transition {transition_id}) attempt {attempt + 1} of '
                  f'{attempts} did not complete within {per_attempt:.1f}s; retrying')
        self.assertIsNotNone(
            result,
            f'ChangeState call (transition {transition_id}) never completed in {attempts} '
            f'attempts over {timeout}s',
        )
        if result.success:
            return
        self.assertTrue(
            retried,
            f'ChangeState call (transition {transition_id}) was rejected on its first '
            'attempt - the node refused a transition that must be valid from where it was',
        )
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, reached_label, timeout=10.0),
            f'ChangeState (transition {transition_id}) was rejected after a retry and '
            f'{TARGET_NODE} is not in "{reached_label}" either - the transition neither '
            'applied on the lost attempt nor succeeded on the retry',
        )

    def test_01_present_but_unconfigured_raises_naming_it(self):
        # Gate on the plugin being live and globally armed BEFORE polling for the raise.
        # Not on app_id=TARGET_NODE, deliberately: the gate reports a tracked node with a
        # known non-active label as 'warming_up' (its node_ok is false - the exact
        # condition under test), so that per-entity state stays suppressed for as long
        # as the fault condition exists. What the raise actually needs is the SOURCE
        # entity's arming (the aggregate goes out under 'graph_watchdog'), for which
        # global armed is the precise precondition - and the gate still proves the .so
        # loaded and the tick loop ran, so a bringup failure dies here naming itself
        # instead of as a 60 s poll timeout blaming the detector.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load, its tick loop never ran, or the bringup grace never elapsed, so no '
            'raise below could mean anything',
        )

        # managed_lifecycle launches unconfigured: present in the graph, alive, but not
        # active. That alone must raise once it persists past the configured grace.
        fault = poll_faults(PORT, FAULT_CODE, timeout=60.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} never raised while {TARGET_NODE} stayed unconfigured',
        )
        self.assertIn(TARGET_NODE, fault.get('description', ''))

        # The flat /faults list carries a fault whatever its source is; only the
        # entity-scoped surface proves an operator can OPEN it somewhere (see
        # test_qos_e2e.test.py's identical rationale for the same entity).
        self.assertIsNotNone(
            poll_entity_faults(PORT, 'apps/graph_watchdog', FAULT_CODE, timeout=30.0),
            f'{FAULT_CODE} is not reachable at /apps/graph_watchdog/faults - the '
            'entity the plugin publishes does not own the fault it raises',
        )

    def test_02_configured_but_inactive_still_raises(self):
        # Explicitly reach "inactive" (unconfigured -> configuring -> inactive) and
        # deliberately do NOT activate. Inactive is still not active, so the
        # level-triggered raise must keep the fault active - a detector that treated
        # "configured" as good enough would heal it here.
        self._call_transition(Transition.TRANSITION_CONFIGURE, reached_label='inactive')

        fault = poll_faults(PORT, FAULT_CODE, timeout=30.0)
        self.assertIsNotNone(
            fault,
            f'{FAULT_CODE} did not stay raised once {TARGET_NODE} reached "inactive"',
        )
        self.assertIn(TARGET_NODE, fault.get('description', ''))

    def test_02b_gateway_restart_never_heals_a_still_inactive_node(self, gateway_node):
        """The withheld-clear guard, at the tier that counts: a real restart.

        The guard exists because a gateway restart brings every detector counter back
        to zero while the fault it raised is still in the fault_manager's store (a
        separate process, so it survives). For a whole `grace` window after the
        restart the tracker reports nothing affected even though the detector's own
        reads say the node is still not active, and the level-triggered emitter turns
        that into a clear - which, with healing enabled, walks a CONFIRMED
        GRAPH_NODE_INACTIVE to HEALED on a robot that never moved.

        Nothing below this tier can reach that state: the C++ integration test rebuilds
        the detector against a fake ReportFault sink, and the other e2e scenarios run
        one gateway process from start to finish.

        The instrument is ``last_passed``, not the fault's current status. A heal here
        is TRANSIENT - the re-raise follows within a few ticks - so a status sample
        taken after the fact sees CONFIRMED again and proves nothing. ``last_passed``
        is set by the fault_manager on the FIRST PASSED report a fault ever receives
        and is never unset, so it catches a single spurious clear whether or not it
        ever reached HEALED.
        """
        before = _fault_record(PORT, FAULT_CODE, timeout=30.0)
        self.assertIsNotNone(
            before,
            f'{FAULT_CODE} is not in the store before the restart, so there is nothing '
            'a restart could wrongly heal',
        )
        self.assertIsNone(
            before.get('last_passed'),
            'the detector had already emitted a clear for this fault BEFORE the restart '
            f'(last_passed={before.get("last_passed")!r}), so the assertion after the '
            'restart could not attribute anything to it',
        )

        old_pid = gateway_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        self.assertTrue(
            _wait_until_port_is_down(PORT, timeout=60.0),
            f'the gateway (pid {old_pid}) kept answering after SIGTERM - nothing was '
            'restarted, so the rest of this test would measure the original process',
        )

        # launch brings the same configuration back. Gate on the plugin being live and
        # globally armed again: a raise (or a clear) is only possible past that point,
        # so measuring before it would measure a stack that had not started detecting.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=90.0),
            'the gateway never came back armed after the restart',
        )
        new_pid = gateway_node.process_details['pid']
        self.assertNotEqual(
            new_pid, old_pid,
            'the gateway process id did not change, so this test never restarted anything',
        )

        # Long enough for the whole spurious-heal sequence to have played out: the
        # detector needs GRACE ticks to walk its fresh streak back up, and the
        # fault_manager needs healing_threshold (default 3) PASSED reports to heal.
        time.sleep(max(4.0, (GRACE + 8) * TICK_INTERVAL_MS / 1000.0))

        after = _fault_record(PORT, FAULT_CODE, timeout=30.0)
        self.assertIsNotNone(
            after,
            f'{FAULT_CODE} vanished from the store entirely after the restart',
        )
        self.assertIsNone(
            after.get('last_passed'),
            'the restarted gateway reported GRAPH_NODE_INACTIVE as PASSED while '
            f'{TARGET_NODE} was still inactive (last_passed='
            f'{after.get("last_passed")!r}) - the restart healed a fault that is still '
            'real',
        )
        self.assertNotEqual(
            after.get('status'), 'healed',
            f'{FAULT_CODE} is HEALED while {TARGET_NODE} has never left "inactive"',
        )

        # And the fault is still on the operator-visible active list, i.e. the withhold
        # preserved it rather than merely delaying the damage.
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=30.0),
            f'{FAULT_CODE} is no longer an active fault after the gateway restart',
        )

    def test_03_activated_arms_the_gate_and_heals(self):
        # "Cleared" is an absence, and it is the default state of a fault that was never
        # raised: on a stack test_01 just failed against, poll_cleared answers True
        # immediately. Confirm the fault is actually THERE first - BEFORE the activate,
        # because after it the heal is racing this check - so what poll_cleared measures
        # is a heal of a proven-present fault (same pairing as
        # test_param_drift_e2e.test.py's heal leg).
        self.assertIsNotNone(
            poll_faults(PORT, FAULT_CODE, timeout=30.0),
            f'{FAULT_CODE} is not active at the start of the heal test, so there is '
            'nothing here that could heal and the clear below would pass without '
            'measuring anything',
        )

        self._call_transition(Transition.TRANSITION_ACTIVATE, reached_label='active')

        # Now - and only now - the per-entity armed gate is reachable: the entity leaves
        # 'warming_up' exactly when the watcher has read "active" for it. Waiting on it
        # splits the two ways the heal could fail: if the REAL lifecycle machinery never
        # fed the label to the gate, the failure is here and names the node; if the
        # label arrived but the clear never flowed, poll_cleared below is the one that
        # fails.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0, app_id=TARGET_NODE),
            f'the gate never armed {TARGET_NODE} after a successful ACTIVATE - the '
            'live lifecycle machinery (transition_event / GetState) never delivered '
            'the "active" label to the watcher',
        )

        cleared = poll_cleared(PORT, FAULT_CODE, timeout=60.0)
        self.assertTrue(
            cleared,
            f'{FAULT_CODE} did not heal after {TARGET_NODE} reached "active"',
        )


class TestLifecycleExpectationDefaultConfig(unittest.TestCase):
    """The shipped default (no config at all) raises nothing for an inactive node."""

    def test_default_config_stays_silent(self):
        # Gate BEFORE asserting absence: a stack that never came up produces exactly the
        # same "no fault" result, and every bringup failure mode on this launch still
        # exits 0 (see the harness docstring).
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0),
            'graph_watchdog never reported an armed global state - the plugin did not '
            'load or its tick loop never ran, so an absent fault proves nothing',
        )
        # The armed gate is served by the plugin inside the gateway process, so it says
        # nothing about the surface the assertion below actually reads. A launch whose
        # fault_manager died, hung, or never DDS-matched answers GET /faults with 503,
        # poll_faults swallows that and returns None, and the absence assertion passes
        # for the one reason it must never pass.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so an absent fault proves nothing about the '
            'detector',
        )
        # Prove the false-positive trigger is fully present: the stack discovered the
        # node AND read its non-active label. Without this the silence below could just
        # as well mean the label never arrived.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=30.0),
            f'the gate never reported {TARGET_NODE} with lifecycle "unconfigured" - '
            'the node was not discovered or its label was never read, so the silence '
            'below would be vacuous',
        )
        # Proving absence means waiting out the full window: with no
        # detectors.lifecycle_expectation config, require_active is empty and the
        # detector must check nothing - the README's zero-false-positive default.
        self.assertIsNone(
            poll_faults(PORT, FAULT_CODE, timeout=SILENT_WINDOW_SEC),
            f'{FAULT_CODE} raised with NO lifecycle_expectation config at all - the '
            'shipped default is documented as checking nothing, so this is a false '
            'positive on every robot that loads the plugin',
        )
        # The trigger was pinned at the START of the window; nothing observed it after
        # that. A demo node that exits 0 two seconds in is tolerated by the exit-code
        # check and invisible to the tracker (a departed node is the presence class's
        # problem), which would leave 2 s of trigger and 18 s of empty graph.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, TARGET_NODE, 'unconfigured', timeout=15.0),
            f'{TARGET_NODE} is no longer reported as "unconfigured" at the END of the '
            'silence window - the trigger did not survive it, so most of the window '
            'proved nothing',
        )


class TestLifecycleExpectationNegativeControl(unittest.TestCase):
    """A required node that IS active never raises."""

    def test_active_required_node_never_raises(self):
        # Gate on THIS app being armed: for an active node the per-entity state is
        # reachable, and it is the strongest form of the arming gate - it proves the
        # .so loaded, the tick loop ran, the node was discovered, its warmup elapsed,
        # and the watcher considers it ok.
        self.assertTrue(
            wait_until_watchdog_armed(PORT, timeout=60.0, app_id=ACTIVE_NODE),
            f'graph_watchdog never reported {ACTIVE_NODE} as an armed entity - the '
            'plugin did not load, its tick loop never ran, or the gateway never '
            'discovered the node, so an absent fault proves nothing',
        )
        # And that the surface the assertion reads is alive in THIS launch: this is the
        # scenario whose job is "a buggy detector raising for an active node must be
        # caught", and a raise that is emitted but lost - fault_manager dead, hung, or
        # never DDS-matched - leaves GET /faults answering 503, which poll_faults
        # swallows into exactly the None the assertion below wants.
        self.assertTrue(
            wait_until_faults_endpoint_live(PORT, timeout=30.0),
            'GET /faults never answered 200 - the gateway never reached the '
            'fault_manager in this launch, so a wrong raise would have been lost rather '
            'than caught',
        )
        # Armed alone is not enough for THIS absence: node_ok also holds for an unread
        # label, and the tracker treats unread as benign - a run whose label never
        # arrived would stay silent for the wrong reason. Pin that "active" was READ.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, ACTIVE_NODE, 'active', timeout=30.0),
            f'the gate never reported {ACTIVE_NODE} with lifecycle "active" - its '
            'label was never read, so the silence below would be vacuous',
        )
        # Same grace and cadence as the "main" scenario, so the only variable between
        # the launch that must raise and this one is the node's actual lifecycle state.
        self.assertIsNone(
            poll_faults(PORT, FAULT_CODE, timeout=SILENT_WINDOW_SEC),
            f'{FAULT_CODE} raised for {ACTIVE_NODE}, which is required active and IS '
            'active - the detector reports the healthy state it exists to verify',
        )
        # The required node must still be there, and still active, at the END of the
        # window: a node that exited early leaves an empty graph, which is silent for a
        # reason that has nothing to do with the detector being right.
        self.assertIsNotNone(
            _poll_watchdog_entity(PORT, ACTIVE_NODE, 'active', timeout=15.0),
            f'{ACTIVE_NODE} is no longer reported as "active" at the END of the silence '
            'window - the required node did not survive it, so most of the window was '
            'not exercising an active node at all',
        )


# Each CTest target launches this file with one scenario, so only that scenario's case
# may run. Removing the others from the module (rather than skipping them) means each
# run reports exactly one case, and a missing result is a real failure rather than an
# expected line of output - see test_config_plumbing_e2e.test.py's identical rationale.
_SCENARIO_CASES = {
    'main': 'TestLifecycleExpectationMain',
    'default_config': 'TestLifecycleExpectationDefaultConfig',
    'negative_control': 'TestLifecycleExpectationNegativeControl',
}
if SCENARIO not in _SCENARIO_CASES:
    raise RuntimeError(
        f'WATCHDOG_E2E_SCENARIO={SCENARIO!r} is not one of {sorted(_SCENARIO_CASES)}; '
        'the CTest target and this file disagree about which scenarios exist')
for _scenario, _case_name in _SCENARIO_CASES.items():
    if _scenario != SCENARIO:
        del globals()[_case_name]


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway/fault_manager/demo stack exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
