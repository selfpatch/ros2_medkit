#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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
State-register gates reach a real fault manager.

A gateway and fault manager run against the fixture server with a node map in
the shape of a machine state model: MachineState is a fault_enum with several
healthy states (ok_values), inhibited while UnitMode is a maintenance mode, and
StopReason counts only while MachineState is stopped or aborted. The test
drives the three registers and checks the fault manager status of each fault
after every step. auto_browse walks the same server, an Int32 array included,
which must be skipped rather than read as a scalar.

A second gateway then runs in subscription mode with the gated point listed
before its gate. MachineState and UnitMode change in one step, as a PLC sets
them in one scan, and the fault must be judged against the new UnitMode.

No step waits a fixed time. After a write the test waits until the plugin has
evaluated it (poll cycles counted by poll_count), checks the plugin's own
verdict in active_alarms, and then waits for the fault manager to show it.

Usage: test_opcua_state_gates.test.py <test_alarm_server>
"""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    'values_test', HERE / 'test_opcua_subscription_values.test.py')
helpers = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(helpers)
alarm_fixture = helpers.alarm_fixture

COMPONENT = 'state_runtime'
ENTITY = 'machine'
ABORTED = 'MACHINE_ABORTED'
UNMAPPED = 'MACHINE_STATE_UNMAPPED'
JAM = 'MACHINE_JAM'
STOPPED, IDLE, EXECUTE, ABORTED_STATE, HELD = 2, 4, 6, 9, 11
PRODUCTION, MAINTENANCE = 1, 2
SUB_ABORTED = 'MACHINE_ABORTED_SUBSCRIBED'


def write_files(workdir, plugin, gateway_port, server_port):
    node_map = workdir / 'nodes.yaml'
    # auto_browse walks the whole fixture, Recipe.Steps (an Int32 array) included.
    node_map.write_text(f"""area_id: plc_systems
component_id: {COMPONENT}
auto_browse: true
nodes:
  - node_id: "ns=2;s=UnitMode"
    entity_id: {ENTITY}
    data_name: unit_mode
    data_type: int
  - node_id: "ns=2;s=MachineState"
    entity_id: {ENTITY}
    data_name: state
    data_type: int
    fault_enum:
      ok_value: 0
      ok_values: [{STOPPED}, {IDLE}, {EXECUTE}]
      unknown_fault_code: {UNMAPPED}
      codes:
        - code: {ABORTED_STATE}
          fault_code: {ABORTED}
    inhibit_when:
      node_id: "ns=2;s=UnitMode"
      values: [{MAINTENANCE}]
  - node_id: "ns=2;s=StopReason"
    entity_id: {ENTITY}
    data_name: stop_reason
    data_type: int
    fault_enum:
      ok_values: [10]
      codes:
        - code: 101
          fault_code: {JAM}
    only_when:
      node_id: "ns=2;s=MachineState"
      values: [{STOPPED}, {ABORTED_STATE}]
""")
    return write_params(workdir, 'gateway.yaml', plugin, gateway_port, server_port, node_map,
                        ['    plugins.opcua.poll_interval_ms: 200'])


def write_subscription_files(workdir, plugin, gateway_port, server_port):
    node_map = workdir / 'nodes_subscription.yaml'
    # The gated point first, its gate after it: the order of the publish response.
    node_map.write_text(f"""area_id: plc_systems
component_id: {COMPONENT}
nodes:
  - node_id: "ns=2;s=MachineState"
    entity_id: {ENTITY}
    data_name: state
    data_type: int
    fault_enum:
      ok_value: {EXECUTE}
      ok_values: [{STOPPED}, {IDLE}]
      codes:
        - code: {ABORTED_STATE}
          fault_code: {SUB_ABORTED}
    inhibit_when:
      node_id: "ns=2;s=UnitMode"
      values: [{MAINTENANCE}]
  - node_id: "ns=2;s=UnitMode"
    entity_id: {ENTITY}
    data_name: unit_mode
    data_type: int
  - node_id: "ns=2;s=StatusWord"
    entity_id: {ENTITY}
    data_name: marker
    data_type: int
""")
    # A poll every 60 s: nothing below is seen through a poll.
    return write_params(workdir, 'gateway_subscription.yaml', plugin, gateway_port, server_port,
                        node_map, ['    plugins.opcua.poll_interval_ms: 60000',
                                   '    plugins.opcua.prefer_subscriptions: true',
                                   '    plugins.opcua.subscription_interval_ms: 250.0'])


def write_params(workdir, name, plugin, gateway_port, server_port, node_map, extra):
    manifest = workdir / 'manifest.yaml'
    manifest.write_text('manifest_version: "1.0"\n')
    params = workdir / name
    params.write_text('\n'.join([
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    server:',
        '      host: "127.0.0.1"',
        f'      port: {gateway_port}',
        '    plugins: ["opcua"]',
        f'    plugins.opcua.path: "{plugin}"',
        f'    plugins.opcua.endpoint_url: "opc.tcp://127.0.0.1:{server_port}"',
        f'    plugins.opcua.node_map_path: "{node_map}"',
        *extra,
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]) + '\n')
    return params


def start_gateway(params, log_path, env):
    log = open(log_path, 'w')
    gateway = subprocess.Popen(
        ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
         '--ros-args', '--params-file', str(params)],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    gateway._log = log
    return gateway


class Leg:
    """One gateway against the fixture: writes registers and checks the faults."""

    def __init__(self, server, server_log, gateway_port, codes):
        self.server = server
        self.server_log = server_log
        self.base = f'http://127.0.0.1:{gateway_port}/api/v1'
        self.codes = codes

    def plc_status(self):
        return helpers.http_json(f'{self.base}/components/{COMPONENT}/x-plc-status')

    def put(self, name, value):
        helpers.check(helpers.set_register(self.server, self.server_log, name, value),
                      f'{name} set to {value}')

    def statuses(self):
        return {code: helpers.fault_status(self.base, code) for code in self.codes}

    def evaluated(self):
        """Wait until the plugin has evaluated the registers as they are now."""
        # The cycle in flight may have read a value from before the write, so
        # the one after it is the first that must have the new one, and it is
        # evaluated before the next one is counted.
        start = (self.plc_status() or {}).get('poll_count', 0)
        got, ok = helpers.wait_for(
            self.plc_status, lambda s: s.get('poll_count', 0) >= start + 3,
            5 * helpers.SCALE, period=0.05)
        helpers.check(ok, f'three poll cycles ran after the write (got {got!r})')

    def expect(self, step, want):
        """After the writes of <step>: the plugin's verdict, then the fault manager's."""
        self.evaluated()
        active = (self.plc_status() or {}).get('active_alarms', [])
        verdict = {code: code in active for code in want}
        helpers.check(verdict == {c: w == 'CONFIRMED' for c, w in want.items()},
                      f'{step}: the plugin holds {verdict}')
        got, ok = helpers.wait_for(
            self.statuses, lambda s: all(s[c] == w for c, w in want.items()),
            5 * helpers.SCALE, period=0.05)
        helpers.check(ok, f'{step}: {want} (got {got!r})')


class SubscriptionLeg(Leg):
    """A leg whose gateway gets values as data changes."""

    marker = 1000

    def put_together(self, pairs):
        """Write several registers in one fixture step."""
        log = Path(self.server_log)
        needles = [f'OK {name}={value}' for name, value in pairs]
        before = [log.read_text(errors='replace').count(n) for n in needles]
        self.server.stdin.write('set ' + ' '.join(f'{n} {v}' for n, v in pairs) + '\n')
        self.server.stdin.flush()
        _, ok = helpers.wait_for(
            lambda: [log.read_text(errors='replace').count(n) for n in needles],
            lambda counts: all(c > b for c, b in zip(counts, before)),
            5 * helpers.SCALE, period=0.05)
        helpers.check(ok, f'{pairs} set in one step')

    def marker_value(self):
        data = helpers.http_json(f'{self.base}/apps/{ENTITY}/x-plc-data') or {}
        for item in data.get('items', []):
            if item.get('name') == 'marker':
                return item.get('value')
        return None

    def evaluated(self):
        # Two marker changes one after the other. The second arrives in a later
        # publish response than the first, and the first arrives with the
        # writes of the step or after them. The plugin evaluates a response
        # before it reads the next one.
        for _ in range(2):
            SubscriptionLeg.marker += 1
            want = SubscriptionLeg.marker
            self.put('StatusWord', want)
            got, ok = helpers.wait_for(self.marker_value, lambda v: v == want,
                                       5 * helpers.SCALE, period=0.05)
            helpers.check(ok, f'the marker change {want} arrived (got {got!r})')


def subscription_leg(leg):
    leg.put_together([('UnitMode', PRODUCTION), ('MachineState', EXECUTE)])
    leg.expect('running in production', {SUB_ABORTED: 'ABSENT'})

    # MachineState is first in the publish response, UnitMode second.
    leg.put_together([('MachineState', ABORTED_STATE), ('UnitMode', MAINTENANCE)])
    leg.expect('abort and maintenance mode in one scan raise nothing', {SUB_ABORTED: 'ABSENT'})

    leg.put('UnitMode', PRODUCTION)
    leg.expect('still aborted when production resumes raises', {SUB_ABORTED: 'CONFIRMED'})

    leg.put_together([('MachineState', EXECUTE), ('UnitMode', MAINTENANCE)])
    leg.expect('the state clears it', {SUB_ABORTED: 'CLEARED'})

    leg.put_together([('MachineState', ABORTED_STATE), ('UnitMode', PRODUCTION)])
    leg.expect('abort and production mode in one scan raise', {SUB_ABORTED: 'CONFIRMED'})


def poll_leg(leg, gateway):
    # An array value is not a data point, and reading one must not take
    # the gateway down.
    walked = helpers.http_json(f'{leg.base}/apps') or {}
    points = []
    for app in walked.get('items', []):
        data = helpers.http_json(f"{leg.base}/apps/{app['id']}/x-plc-data") or {}
        points += [i.get('node_id') for i in data.get('items', [])]
    helpers.check('ns=2;s=StatusWord' in points and 'ns=2;s=Recipe.Steps' not in points,
                  f'auto_browse maps the scalars and skips the array (got {sorted(points)})')
    helpers.check(gateway.poll() is None, 'the gateway is still running after the walk')

    put, expect = leg.put, leg.expect
    put('UnitMode', PRODUCTION)
    put('MachineState', EXECUTE)
    expect('running in production', {ABORTED: 'ABSENT', UNMAPPED: 'ABSENT', JAM: 'ABSENT'})

    for state in (STOPPED, IDLE):
        put('MachineState', state)
        expect(f'healthy state {state}', {ABORTED: 'ABSENT', UNMAPPED: 'ABSENT'})

    put('MachineState', ABORTED_STATE)
    expect('aborted in production raises', {ABORTED: 'CONFIRMED', UNMAPPED: 'ABSENT'})

    put('UnitMode', MAINTENANCE)
    expect('maintenance mode keeps a raised fault', {ABORTED: 'CONFIRMED'})

    put('MachineState', STOPPED)
    expect('the state clears it, inhibited or not', {ABORTED: 'CLEARED'})

    put('MachineState', ABORTED_STATE)
    expect('aborted during maintenance is not raised',
           {ABORTED: 'CLEARED', UNMAPPED: 'ABSENT'})

    put('UnitMode', PRODUCTION)
    expect('still aborted when production resumes raises', {ABORTED: 'CONFIRMED'})

    put('MachineState', HELD)
    expect('an unmapped state raises the catch-all',
           {ABORTED: 'CLEARED', UNMAPPED: 'CONFIRMED'})

    put('MachineState', EXECUTE)
    put('StopReason', 101)
    expect('a latched stop reason while running is ignored',
           {JAM: 'ABSENT', UNMAPPED: 'CLEARED'})

    put('MachineState', STOPPED)
    expect('the same reason while stopped raises', {JAM: 'CONFIRMED'})

    put('MachineState', EXECUTE)
    expect('running again clears it', {JAM: 'CLEARED'})

    put('StopReason', 10)
    put('MachineState', STOPPED)
    expect('a planned stop reason raises nothing', {JAM: 'CLEARED', ABORTED: 'CLEARED'})


def main():
    if len(sys.argv) < 2:
        print('usage: test_opcua_state_gates.test.py <test_alarm_server>', file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: run this test through CTest', file=sys.stderr)
        return 1
    plugin = helpers.find_plugin()
    if shutil.which('ros2') is None or plugin is None or not server_bin.is_file():
        print('FAIL: ros2, the plugin or the fixture server is missing', file=sys.stderr)
        return 1

    scale = helpers.SCALE
    check = helpers.check
    workdir = Path(tempfile.mkdtemp(prefix='opcua_state_gates_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    server_log = workdir / 'alarm_server.log'
    gateway_log = workdir / 'gateway.log'
    server, server_port = alarm_fixture.start_fixture(server_bin, server_log, deadline=25 * scale)
    gateway_port = helpers.free_port()
    fault_manager = gateway = None
    passed = False
    try:
        if server is None:
            print('FAIL: fixture server did not start', file=sys.stderr)
            return 1
        fault_manager = helpers.start_fault_manager(workdir / 'fault_manager.log', env)
        if fault_manager is None:
            print('FAIL: fault_manager_node did not start', file=sys.stderr)
            return 1
        params = write_files(workdir, plugin, gateway_port, server_port)
        gateway = start_gateway(params, gateway_log, env)
        leg = Leg(server, server_log, gateway_port, (ABORTED, UNMAPPED, JAM))
        got, ok = helpers.wait_for(
            leg.plc_status, lambda s: s.get('connected') and s.get('poll_count', 0) > 2,
            90 * scale)
        if not check(ok, f'the gateway polls the fixture (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        poll_leg(leg, gateway)

        helpers.terminate(gateway)
        gateway_log = workdir / 'gateway_subscription.log'
        gateway = start_gateway(
            write_subscription_files(workdir, plugin, gateway_port, server_port), gateway_log, env)
        leg = SubscriptionLeg(server, server_log, gateway_port, (SUB_ABORTED,))
        got, ok = helpers.wait_for(
            leg.plc_status,
            lambda s: s.get('mode') == 'subscription' and s.get('connected')
            and s.get('poll_count', 0) > 0, 90 * scale)
        if not check(ok, f'the second gateway gets values by subscription (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        subscription_leg(leg)
        if helpers.failures:
            print(f'FAIL: {len(helpers.failures)} expectation(s) not met', file=sys.stderr)
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        print('PASS: state gates decide raise, hold and clear')
        passed = True
        return 0
    finally:
        helpers.terminate(gateway)
        helpers.terminate(fault_manager)
        if server is not None:
            alarm_fixture.stop(server)
            if not passed:
                print('fixture server log:\n' + alarm_fixture.read_log(server_log),
                      file=sys.stderr)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
