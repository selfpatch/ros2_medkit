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
browse_path entries against a real server, through restarts that move nodes.

A gateway and fault manager run against robot_server.py (asyncua), which has
the namespaces and BrowseNames of OPC UA DI and OPC 40010 Robotics. The node
map addresses the robot's EmergencyStop and SpeedOverride by browse path, and
auto_browse is on. The server is restarted four times:

  1. The gateway starts before the server: the first connect fails, and the
     entries resolve once the server is up.
  2. The server restarts with its namespaces in another order. The NodeId the
     EmergencyStop entry held now names a spare variable, and the NodeId it
     moves to is held by the auto-browsed entry of that spare. The entry must
     follow its node and keep its fault.
  3. The server restarts unchanged: the link is lost and restored, and the
     fault stays.
  4. The server restarts in the first order with no robot yet. The paths are
     not found, which is logged, and the entries keep NodeIds that now name
     the spare variables.

In a write-capable build the test also writes SpeedOverride. During restart 2
the server answers Browse slowly and the test writes all the time: no write
may land while the paths are walked, and none may ever reach the spare
variable. After restart 4 a write must be refused, because the NodeId the
entry kept names the spare setpoint.

Usage: test_opcua_browse_path.test.py <robot_server.py> <read-only|write-capable>
"""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    'values_test', HERE / 'test_opcua_subscription_values.test.py')
helpers = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(helpers)

COMPONENT = 'cell'
ENTITY = 'robot'
ESTOP = 'ROBOT_EMERGENCY_STOP'
ESTOP_PATH = 'di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop'
SPEED_PATH = 'di:DeviceSet/*/di:ParameterSet/rob:SpeedOverride'
# Namespace order of the server: DI, Robotics, cell, line are 2..5 in the first
# layout and cell, line, DI, Robotics in the swapped one.
FIRST = {'estop': 'ns=3;i=6001', 'speed': 'ns=3;i=6002', 'spare': 'ns=5;i=6001'}
SWAPPED = {'estop': 'ns=5;i=6001', 'speed': 'ns=5;i=6002', 'spare': 'ns=3;i=6001'}
SKIP = 77


def write_files(workdir, plugin, gateway_port, server_port):
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(f"""area_id: plc_systems
component_id: {COMPONENT}
auto_browse: true
namespaces:
  di: "http://opcfoundation.org/UA/DI/"
  rob: "http://opcfoundation.org/UA/Robotics/"
nodes:
  - browse_path: "{ESTOP_PATH}"
    entity_id: {ENTITY}
    data_name: emergency_stop
    data_type: bool
    alarm: {{fault_code: {ESTOP}, severity: ERROR, threshold: 0.5, above_threshold: true}}
  - browse_path: "{SPEED_PATH}"
    entity_id: {ENTITY}
    data_name: speed_override
    data_type: float
    writable: true
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


class RobotServer:
    """robot_server.py as a child process, restarted on one port."""

    def __init__(self, script, port, workdir):
        self.script = script
        self.port = port
        self.workdir = workdir
        self.proc = None
        self.log_path = None
        self.starts = 0

    def start(self, *args):
        self.stop()
        self.starts += 1
        self.log_path = self.workdir / f'robot_server_{self.starts}.log'
        log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(
            [sys.executable, str(self.script), '--port', str(self.port), *args],
            stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT, text=True,
            start_new_session=True)
        self.proc._log = log
        return helpers.wait_log(self.log_path, 'READY ', 30 * helpers.SCALE, period=0.05)

    def stop(self):
        if self.proc is not None:
            helpers.terminate(self.proc)
            self.proc = None

    def command(self, line, answer):
        """Send one command and wait for the answer that starts with <answer>."""
        log = Path(self.log_path)
        before = log.read_text(errors='replace').count(answer)
        self.proc.stdin.write(line + '\n')
        self.proc.stdin.flush()
        _, ok = helpers.wait_for(lambda: log.read_text(errors='replace').count(answer),
                                 lambda n: n > before, 10 * helpers.SCALE, period=0.05)
        return ok

    def write(self, name, value):
        return self.command(f'set {name} {value}', f'OK {name}={value}')

    def get(self, name):
        """Return the value the server holds for <name>, as text."""
        if not self.command(f'get {name}', f'VALUE {name}='):
            return None
        lines = [line for line in Path(self.log_path).read_text(errors='replace').splitlines()
                 if line.startswith(f'VALUE {name}=')]
        return lines[-1].split('=', 1)[1]

    def log(self):
        return Path(self.log_path).read_text(errors='replace') if self.log_path else ''


def put(url, value, timeout=5):
    """PUT {'value': value}; return the HTTP status, or None when nothing answered."""
    req = urllib.request.Request(url, data=json.dumps({'value': value}).encode(), method='PUT',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code
    except (urllib.error.URLError, OSError):
        return None


def main():
    if len(sys.argv) < 3 or sys.argv[2] not in ('read-only', 'write-capable'):
        print('usage: test_opcua_browse_path.test.py <robot_server.py> <read-only|write-capable>',
              file=sys.stderr)
        return 2
    server_script = Path(sys.argv[1]).resolve()
    write_capable = sys.argv[2] == 'write-capable'
    try:
        import asyncua  # noqa: F401
    except ImportError:
        if os.environ.get('ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE'):
            print('FAIL: asyncua is not installed and ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE is set',
                  file=sys.stderr)
            return 1
        print('asyncua is not installed - skipping (CTest skip code)')
        return SKIP
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: run this test through CTest', file=sys.stderr)
        return 1
    plugin = helpers.find_plugin()
    if shutil.which('ros2') is None or plugin is None or not server_script.is_file():
        print('FAIL: ros2, the plugin or the server script is missing', file=sys.stderr)
        return 1

    scale = helpers.SCALE
    check = helpers.check
    wait_for = helpers.wait_for
    workdir = Path(tempfile.mkdtemp(prefix='opcua_browse_path_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    gateway_log = workdir / 'gateway.log'
    server = RobotServer(server_script, helpers.free_port(), workdir)
    gateway_port = helpers.free_port()
    base = f'http://127.0.0.1:{gateway_port}/api/v1'
    speed_url = f'{base}/apps/{ENTITY}/data/speed_override'
    fault_manager = gateway = None
    passed = False

    def plc_status():
        return helpers.http_json(f'{base}/components/{COMPONENT}/x-plc-status')

    def point(name):
        """Return the x-plc-data item of the robot entity called <name>, or {}."""
        data = helpers.http_json(f'{base}/apps/{ENTITY}/x-plc-data') or {}
        for item in data.get('items', []):
            if item.get('name') == name:
                return item
        return {}

    def mapped_node_ids():
        ids = []
        for app in (helpers.http_json(f'{base}/apps') or {}).get('items', []):
            data = helpers.http_json(f"{base}/apps/{app['id']}/x-plc-data") or {}
            ids += [i.get('node_id') for i in data.get('items', [])]
        return ids

    def estop_status():
        return helpers.fault_status(base, ESTOP)

    def log_count(needle):
        return gateway_log.read_text(errors='replace').count(needle)

    def polled():
        """Wait until three poll cycles ran, so the values as they are now were evaluated."""
        start = (plc_status() or {}).get('poll_count', 0)
        got, ok = wait_for(plc_status,
                           lambda s: s.get('connected') and s.get('poll_count', 0) >= start + 3,
                           30 * scale, period=0.05)
        check(ok, f'three poll cycles ran (got {got!r})')

    def entries_at(ids, deadline):
        """Wait until both entries are served under the NodeIds of <ids> on a live session."""
        got, ok = wait_for(
            lambda: (point('emergency_stop').get('node_id'),
                     point('speed_override').get('node_id'),
                     (plc_status() or {}).get('connected')),
            lambda g: g == (ids['estop'], ids['speed'], True), deadline, period=0.1)
        return got, ok

    try:
        fault_manager = helpers.start_fault_manager(workdir / 'fault_manager.log', env)
        if fault_manager is None:
            print('FAIL: fault_manager_node did not start', file=sys.stderr)
            return 1
        params = write_files(workdir, plugin, gateway_port, server.port)
        log = open(gateway_log, 'w')
        gateway = subprocess.Popen(
            ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
             '--ros-args', '--params-file', str(params)],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        gateway._log = log

        # --- 1. No server yet: the first connect fails -------------------------
        got, ok = wait_for(plc_status,
                           lambda s: s.get('connected') is False and s.get('last_error'),
                           60 * scale)
        if not check(ok, f'the first connect fails with no server (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        check(not point('emergency_stop'), 'an unresolved browse_path entry is not served')

        if not check(server.start('--layout', 'first'), 'the server starts, first layout'):
            print(server.log(), file=sys.stderr)
            return 1
        got, ok = entries_at(FIRST, 60 * scale)
        if not check(ok, f'both paths resolve once the server is up (got {got!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        got, ok = wait_for(mapped_node_ids, lambda ids: FIRST['spare'] in ids, 30 * scale)
        check(ok, f'auto_browse maps the spare variable at {FIRST["spare"]} (got {got!r})')
        check(point('speed_override').get('writable') is write_capable,
              f'speed_override is writable only in a write-capable build ({write_capable})')

        check(server.write('estop', 1), 'EmergencyStop set')
        got, ok = wait_for(estop_status, lambda s: s == 'CONFIRMED', 15 * scale, period=0.1)
        check(ok, f'{ESTOP} is raised from the browse_path entry (got {got!r})')
        if write_capable:
            check(put(speed_url, 40.0) == 200, 'a write to SpeedOverride succeeds')
            check(server.get('speed') == '40.0', 'the server holds SpeedOverride 40')
        cleared_before = log_count(f'Alarm cleared: {ESTOP}')

        # --- 2. Restart with the namespaces in another order -------------------
        # Browse is slow, so the walk of the two paths takes a few seconds.
        slow = ['--slow', '0.4'] if write_capable else []
        if not check(server.start('--layout', 'swapped', '--estop', '1', *slow),
                     'the server restarts, swapped layout'):
            print(server.log(), file=sys.stderr)
            return 1
        if write_capable:
            # Write all the time. Until the paths are walked on the new session
            # the old NodeId of SpeedOverride names SpareSetpoint.
            statuses = []

            def write():
                statuses.append(put(speed_url, 55.0))
                return statuses[-1]

            wait_for(write, lambda status: status == 200, 60 * scale, period=0.02)
            check(server.command('slow 0', 'OK slow=0'), 'Browse answers at once again')
            check(statuses[-1] == 200, f'a write succeeds once the paths are walked '
                                       f'(statuses seen: {sorted(set(map(str, statuses)))})')
            check(503 in statuses, 'writes during the walk were refused with 503 '
                                   f'(statuses seen: {sorted(set(map(str, statuses)))})')
            check(server.get('setpoint') == '0.0',
                  'no write reached SpareSetpoint, which the old NodeId names now')
            check(server.get('speed') == '55.0', 'the write that succeeded reached SpeedOverride')
        got, ok = entries_at(SWAPPED, 60 * scale)
        check(ok, f'both entries follow their nodes to the new NodeIds (got {got!r})')
        got, ok = wait_for(mapped_node_ids, lambda ids: SWAPPED['spare'] in ids, 30 * scale)
        check(ok, f'auto_browse maps the spare variable at its new NodeId (got {got!r})')
        polled()
        check(estop_status() == 'CONFIRMED'
              and ESTOP in (plc_status() or {}).get('active_alarms', []),
              f'{ESTOP} is still raised: the entry reads EmergencyStop, not the spare variable')
        check(log_count(f'Alarm cleared: {ESTOP}') == cleared_before,
              'the fault was never cleared across the restart')

        check(server.write('spare', 1) and server.write('estop', 0),
              'spare set, EmergencyStop reset')
        got, ok = wait_for(estop_status, lambda s: s == 'CLEARED', 15 * scale, period=0.1)
        check(ok, f'{ESTOP} clears with EmergencyStop although the spare is set (got {got!r})')
        check(server.write('spare', 0) and server.write('estop', 1),
              'spare reset, EmergencyStop set')
        got, ok = wait_for(estop_status, lambda s: s == 'CONFIRMED', 15 * scale, period=0.1)
        check(ok, f'{ESTOP} is raised again (got {got!r})')
        cleared_before = log_count(f'Alarm cleared: {ESTOP}')

        # --- 3. Link lost and restored, nothing moved ---------------------------
        moved_before = log_count('browse_path node moved')
        if not check(server.start('--layout', 'swapped', '--estop', '1'),
                     'the server restarts unchanged'):
            print(server.log(), file=sys.stderr)
            return 1
        got, ok = wait_for(plc_status, lambda s: s.get('connected') is True, 60 * scale)
        check(ok, f'the link is back (got {got!r})')
        polled()
        check(entries_at(SWAPPED, 5 * scale)[1], 'both entries keep their NodeIds')
        check(log_count('browse_path node moved') == moved_before, 'no entry moved')
        check(estop_status() == 'CONFIRMED'
              and log_count(f'Alarm cleared: {ESTOP}') == cleared_before,
              f'{ESTOP} stays raised across the lost link')

        # --- 4. Restart in the first order, the robot not there yet ------------
        if not check(server.start('--layout', 'first', '--no-robot'),
                     'the server restarts with no robot'):
            print(server.log(), file=sys.stderr)
            return 1
        for path, node_id in ((ESTOP_PATH, SWAPPED['estop']), (SPEED_PATH, SWAPPED['speed'])):
            needle = f"browse_path '{path}' is not found on this session - keeping {node_id}"
            check(helpers.wait_log(gateway_log, needle, 60 * scale, period=0.1),
                  f'the log names the path that is not found and the NodeId it keeps: {path}')
        polled()
        if write_capable:
            # ns=5;i=6002 is SpareSetpoint on this session.
            status = put(speed_url, 77.0)
            check(status == 503, f'a write to the entry whose path is not found is 503 '
                                 f'(got {status})')
            check(server.get('setpoint') == '0.0', 'SpareSetpoint was not written')
        check(gateway.poll() is None, 'the gateway is still running')

        if helpers.failures:
            print(f'FAIL: {len(helpers.failures)} expectation(s) not met', file=sys.stderr)
            print(gateway_log.read_text(errors='replace')[-6000:], file=sys.stderr)
            return 1
        print('PASS: browse_path entries follow their nodes on a real server')
        passed = True
        return 0
    finally:
        helpers.terminate(gateway)
        helpers.terminate(fault_manager)
        if not passed:
            print('robot server log:\n' + server.log(), file=sys.stderr)
        server.stop()
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
