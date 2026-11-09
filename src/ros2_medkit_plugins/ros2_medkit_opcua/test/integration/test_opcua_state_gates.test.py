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

Usage: test_opcua_state_gates.test.py <test_alarm_server>
"""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

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
    manifest = workdir / 'manifest.yaml'
    manifest.write_text('manifest_version: "1.0"\n')
    params = workdir / 'gateway.yaml'
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
        '    plugins.opcua.poll_interval_ms: 200',
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]) + '\n')
    return params


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
        log = open(gateway_log, 'w')
        gateway = subprocess.Popen(
            ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
             '--ros-args', '--params-file', str(params)],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        gateway._log = log
        base = f'http://127.0.0.1:{gateway_port}/api/v1'

        def plc_status():
            return helpers.http_json(f'{base}/components/{COMPONENT}/x-plc-status')

        got, ok = helpers.wait_for(
            plc_status, lambda s: s.get('connected') and s.get('poll_count', 0) > 2, 90 * scale)
        if not check(ok, f'the gateway polls the fixture (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1

        def put(name, value):
            check(helpers.set_register(server, server_log, name, value), f'{name} set to {value}')

        def statuses():
            return {code: helpers.fault_status(base, code) for code in (ABORTED, UNMAPPED, JAM)}

        def expect(step, want, settle=1.5):
            # A raise must arrive within the deadline, a non-raise must hold for
            # several poll cycles, so both wait the same settle time first.
            time.sleep(settle * scale)
            got, ok = helpers.wait_for(
                statuses, lambda s: all(s[c] == w for c, w in want.items()), 3 * scale)
            check(ok, f'{step}: {want} (got {got!r})')

        # An array value is not a data point, and reading one must not take
        # the gateway down.
        walked = helpers.http_json(f'{base}/apps') or {}
        points = []
        for app in walked.get('items', []):
            data = helpers.http_json(f"{base}/apps/{app['id']}/x-plc-data") or {}
            points += [i.get('node_id') for i in data.get('items', [])]
        check('ns=2;s=StatusWord' in points and 'ns=2;s=Recipe.Steps' not in points,
              f'auto_browse maps the scalars and skips the array (got {sorted(points)})')
        check(gateway.poll() is None, 'the gateway is still running after the walk')

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
