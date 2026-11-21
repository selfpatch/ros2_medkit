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
The clear of a fault code leaves the plugin only after its report was answered.

A gateway runs against the fixture server and against fault_service_stub.py in
place of the fault manager. The stub answers the report of one code late and
never answers the report of another. For each code the test raises the fault,
waits for the stub to receive the report, clears the fault and reads from the
stub's log when the clear arrived:

  * late answer: the clear arrives after the stub answered the report, although
    the plugin had it ready while the report was still held.
  * no answer: the clear arrives once the report is given up as stale (5 s),
    not before and not never.

Usage: test_opcua_fault_order.test.py <test_alarm_server>
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

COMPONENT = 'order_runtime'
ENTITY = 'plc_app'
LATE = 'ANSWERED_LATE'
NEVER = 'NEVER_ANSWERED'
LATE_SECONDS = 2.0
# FaultCodeLanes gives an unanswered call up after this long.
STALE_AFTER = 5.0


def write_files(workdir, plugin, gateway_port, server_port):
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(f"""area_id: plc_systems
component_id: {COMPONENT}
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: {ENTITY}
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 0
        fault_code: {LATE}
      - bit: 1
        fault_code: {NEVER}
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
        '    plugins.opcua.poll_interval_ms: 100',
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]) + '\n')
    return params


def events(stub_log):
    """Return the stub's log as (seconds, event, code) tuples."""
    out = []
    for line in Path(stub_log).read_text(errors='replace').splitlines():
        parts = line.split()
        if len(parts) == 3:
            out.append((float(parts[0]), parts[1], parts[2]))
    return out


def first(stub_log, event, code):
    """Return when the stub logged <event> for <code> first, or None."""
    for at, name, fault_code in events(stub_log):
        if name == event and fault_code == code:
            return at
    return None


def main():
    if len(sys.argv) < 2:
        print('usage: test_opcua_fault_order.test.py <test_alarm_server>', file=sys.stderr)
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
    workdir = Path(tempfile.mkdtemp(prefix='opcua_fault_order_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    server_log = workdir / 'alarm_server.log'
    gateway_log = workdir / 'gateway.log'
    stub_log = workdir / 'stub_events.log'
    stub_output = workdir / 'stub_output.log'
    server, server_port = alarm_fixture.start_fixture(server_bin, server_log, deadline=25 * scale)
    gateway_port = helpers.free_port()
    stub = gateway = None
    passed = False
    try:
        if server is None:
            print('FAIL: fixture server did not start', file=sys.stderr)
            return 1
        out = open(stub_output, 'w')
        stub = subprocess.Popen(
            [sys.executable, str(HERE / 'fault_service_stub.py'), str(stub_log),
             '--late', LATE, '--never', NEVER, '--late-seconds', str(LATE_SECONDS)],
            stdout=out, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        stub._log = out
        if not check(helpers.wait_log(stub_log, ' ready ', 30 * scale, period=0.1),
                     'the fault service stub offers its services'):
            print(stub_output.read_text(errors='replace'), file=sys.stderr)
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

        check(helpers.set_register(server, server_log, 'StatusWord', 0), 'StatusWord starts at 0')
        got, ok = helpers.wait_for(
            plc_status, lambda s: s.get('connected') and s.get('poll_count', 0) > 2, 90 * scale)
        if not check(ok, f'the gateway polls the fixture (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1

        def raise_then_clear(code, bit, clear_deadline):
            """Raise <code>, clear it once its report reached the stub; return both times."""
            check(helpers.set_register(server, server_log, 'StatusWord', 1 << bit),
                  f'bit {bit} set')
            received, ok = helpers.wait_for(
                lambda: first(stub_log, 'report_received', code), lambda at: True,
                30 * scale, period=0.05)
            check(ok, f'the report of {code} reached the stub')
            check(helpers.set_register(server, server_log, 'StatusWord', 0), f'bit {bit} cleared')
            # The plugin has worked the clear out: it is ready to go while the
            # report is still unanswered.
            got, ok = helpers.wait_for(
                plc_status, lambda s: code not in s.get('active_alarms', [code]),
                10 * scale, period=0.05)
            check(ok, f'the plugin holds {code} as cleared (got {got!r})')
            cleared, ok = helpers.wait_for(
                lambda: first(stub_log, 'clear_received', code), lambda at: True,
                clear_deadline, period=0.05)
            check(ok, f'the clear of {code} reached the stub')
            return received, cleared

        received, cleared = raise_then_clear(LATE, 0, (LATE_SECONDS + 10) * scale)
        answered = first(stub_log, 'report_answered', LATE)
        if received is not None and cleared is not None:
            check(answered is not None and cleared > answered,
                  f'the clear of {LATE} arrived after its report was answered '
                  f'(received {received:.3f}, answered {answered}, clear {cleared:.3f})')
            check(cleared - received >= LATE_SECONDS,
                  f'the clear waited out the held report ({cleared - received:.3f} s)')

        received, cleared = raise_then_clear(NEVER, 1, (STALE_AFTER + 15) * scale)
        if received is not None and cleared is not None:
            waited = cleared - received
            check(first(stub_log, 'report_answered', NEVER) is None,
                  f'the report of {NEVER} was never answered')
            # The report left the plugin a moment before the stub logged it.
            check(waited >= STALE_AFTER - 0.5,
                  f'the clear of {NEVER} waited for the report to go stale ({waited:.3f} s)')

        if helpers.failures:
            print(f'FAIL: {len(helpers.failures)} expectation(s) not met', file=sys.stderr)
            print('stub events:\n' + stub_log.read_text(errors='replace'), file=sys.stderr)
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        print('PASS: a clear leaves after its report was answered or given up')
        passed = True
        return 0
    finally:
        helpers.terminate(gateway)
        helpers.terminate(stub)
        if server is not None:
            alarm_fixture.stop(server)
            if not passed:
                print('fixture server log:\n' + alarm_fixture.read_log(server_log),
                      file=sys.stderr)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
