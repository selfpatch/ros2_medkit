#!/usr/bin/env python3
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

"""
Subscription mode with only plain nodes delivers values.

A real gateway runs with prefer_subscriptions and a node map that has scalar
nodes and no event_alarms, against the fixture server. The test checks over
HTTP that the plugin reports subscription mode, that the values arrive, that a
threshold alarm on a constant node is active, and that a register written on
the server reaches the plugin and raises and clears its status-bit fault.

Usage: test_opcua_subscription_values.test.py <test_alarm_server>
"""

import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

COMPONENT = 'subscription_runtime'
ENTITY = 'plc_app'
CONSTANT = 'TANK_ABOVE_40'
BIT = 'STATUS_BIT0'

failures = []


def time_scale():
    """Return MEDKIT_TEST_TIME_SCALE, the factor the sanitizer jobs apply to wall-clock budgets."""
    try:
        scale = float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1'))
    except ValueError:
        return 1.0
    return scale if scale >= 1.0 else 1.0


SCALE = time_scale()


def check(condition, message):
    """Record a failed expectation and keep going, so one run reports them all."""
    if condition:
        print(f'  OK   {message}')
    else:
        print(f'  FAIL {message}', file=sys.stderr)
        failures.append(message)
    return bool(condition)


def free_port():
    """Grab an OS-assigned free TCP port on the loopback and release it."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def find_plugin():
    """Locate libros2_medkit_opcua_plugin.so under AMENT_PREFIX_PATH."""
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        for root, _dirs, files in os.walk(prefix):
            if 'libros2_medkit_opcua_plugin.so' in files:
                return os.path.join(root, 'libros2_medkit_opcua_plugin.so')
    return None


def http_json(url, timeout=5):
    """Return the parsed JSON of a GET, or None on any failure."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def wait_for(fetch, predicate, deadline, period=0.25):
    """Poll fetch() until predicate holds; return the last result and whether it held."""
    last = None
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        last = fetch()
        try:
            if last is not None and predicate(last):
                return last, True
        except (KeyError, TypeError, ValueError):
            pass
        time.sleep(period)
    return last, False


def wait_log(path, needle, deadline=30, period=0.5):
    """Poll a log file until <needle> appears; return True/False."""
    for _ in range(int(deadline / period) + 1):
        try:
            if needle in Path(path).read_text(errors='replace'):
                return True
        except OSError:
            pass
        time.sleep(period)
    return False


def terminate(proc):
    """SIGTERM then SIGKILL a child process group started with start_new_session."""
    if proc is None:
        return
    pgid = None
    if proc.returncode is None:
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pgid = None

    def signal_group(sig):
        if pgid is None:
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass

    signal_group(signal.SIGTERM)
    proc.terminate()
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGKILL)
        proc.kill()
        proc.wait()
    signal_group(signal.SIGKILL)
    log = getattr(proc, '_log', None)
    if log is not None:
        log.close()


def start_server(server_bin, port, log_path):
    """Start the fixture server and wait for its READY line."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        [str(server_bin), '--port', str(port)],
        stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
        text=True, start_new_session=True,
    )
    proc._log = log
    if not wait_log(log_path, 'READY ', deadline=25):
        terminate(proc)
        return None
    return proc


def set_register(server, log_path, name, value):
    """Write one Int32 register through the fixture and wait for its OK line."""
    needle = f'OK {name}={value}'
    before = Path(log_path).read_text(errors='replace').count(needle)
    server.stdin.write(f'set {name} {value}\n')
    server.stdin.flush()
    for _ in range(40):
        if Path(log_path).read_text(errors='replace').count(needle) > before:
            return True
        time.sleep(0.05)
    return False


def write_files(workdir, plugin, gateway_port, server_port):
    """Write the node map, manifest and gateway params; return the params path."""
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(
        'area_id: plc_systems\n'
        f'component_id: {COMPONENT}\n'
        'nodes:\n'
        '  - node_id: "ns=2;s=Tank.Level"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: tank_level\n'
        '    data_type: float\n'
        '    alarm:\n'
        f'      fault_code: {CONSTANT}\n'
        '      threshold: 40.0\n'
        '  - node_id: "ns=2;s=StatusWord"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: status_word\n'
        '    data_type: int\n'
        '    status_bits:\n'
        '      - bit: 0\n'
        f'        fault_code: {BIT}\n'
    )
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
        '    plugins.opcua.poll_interval_ms: 500',
        '    plugins.opcua.prefer_subscriptions: true',
        '    plugins.opcua.subscription_interval_ms: 250.0',
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]) + '\n')
    return params


def main():
    if len(sys.argv) < 2:
        print('usage: test_opcua_subscription_values.test.py <test_alarm_server>', file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()

    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: this test must be launched by CTest, which '
              'runs it behind medkit_run_with_domain.py', file=sys.stderr)
        return 1
    if shutil.which('ros2') is None:
        print('FAIL: ros2 not on PATH', file=sys.stderr)
        return 1
    if not (server_bin.is_file() and os.access(server_bin, os.X_OK)):
        print(f'FAIL: fixture server missing: {server_bin}', file=sys.stderr)
        return 1
    plugin = find_plugin()
    if plugin is None:
        print('FAIL: libros2_medkit_opcua_plugin.so not found under AMENT_PREFIX_PATH',
              file=sys.stderr)
        return 1

    workdir = Path(tempfile.mkdtemp(prefix='opcua_subscription_values_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    server_port = free_port()
    gateway_port = free_port()
    server_log = workdir / 'alarm_server.log'
    gateway_log = workdir / 'gateway.log'
    server = start_server(server_bin, server_port, server_log)
    gateway = None
    try:
        if server is None:
            print('FAIL: fixture server did not become READY', file=sys.stderr)
            print(server_log.read_text(errors='replace'), file=sys.stderr)
            return 1
        params = write_files(workdir, plugin, gateway_port, server_port)
        log = open(gateway_log, 'w')
        gateway = subprocess.Popen(
            ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
             '--ros-args', '--params-file', str(params)],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
        )
        gateway._log = log

        base = f'http://127.0.0.1:{gateway_port}/api/v1'
        status_url = f'{base}/components/{COMPONENT}/x-plc-status'

        def status():
            return http_json(status_url)

        def data():
            return http_json(f'{base}/apps/{ENTITY}/x-plc-data')

        first, served = wait_for(status, lambda s: s.get('mode') == 'subscription', 90 * SCALE)
        if not check(served, f'the plugin runs in subscription mode (got {first!r})'):
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1

        got, ok = wait_for(status,
                           lambda s: s.get('connected') is True and s.get('poll_count', 0) > 0,
                           15 * SCALE)
        check(ok, f'values arrive: connected and poll_count > 0 (got {got!r})')

        def tank_level(payload):
            for item in payload.get('items', []):
                if item.get('name') == 'tank_level':
                    return item.get('value')
            return None

        got, ok = wait_for(data, lambda d: tank_level(d) == 42.0, 15 * SCALE)
        check(ok, f'tank_level reads 42.0 from the server (got {got!r})')

        got, ok = wait_for(status, lambda s: CONSTANT in s.get('active_alarms', []), 15 * SCALE)
        check(ok, f'{CONSTANT} is active (got {got!r})')

        check(set_register(server, server_log, 'StatusWord', 1), 'StatusWord set to 1')
        got, raised = wait_for(status, lambda s: BIT in s.get('active_alarms', []), 15 * SCALE)
        check(raised, f'the change reaches the plugin: {BIT} is active (got {got!r})')

        # The clear only means something once the raise was seen.
        if raised:
            check(set_register(server, server_log, 'StatusWord', 0), 'StatusWord set back to 0')
            got, ok = wait_for(status, lambda s: BIT not in s.get('active_alarms', []), 15 * SCALE)
            check(ok, f'{BIT} clears again (got {got!r})')

        if failures:
            print(f'FAIL: {len(failures)} expectation(s) not met:', file=sys.stderr)
            for f in failures:
                print(f'  - {f}', file=sys.stderr)
            print(gateway_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        print('PASS: subscription mode delivers values for plain nodes')
        return 0
    finally:
        terminate(gateway)
        terminate(server)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
