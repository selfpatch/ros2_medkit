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
raise_after_samples and raise_after_ms through a real gateway and OPC UA server.

The fixture server holds two writable Int32 registers and a constant
Tank.Level of 42.0. The node map gives each a detection rule:

  * Tank.Level > 40, no qualifier: active from the first read. The control
    that shows the detection path is live in each leg.
  * StatusWord bit 0 with raise_after_ms: 3000.
  * FaultCode > 0 with raise_after_samples: 4.

The test writes the registers through the fixture's stdin and reads which
faults the plugin holds active from x-plc-status. It runs once with polling
and once with subscriptions, the transport where a steady value sends no
further updates. Per leg:

  * a StatusWord blip shorter than the window never shows up,
  * a held StatusWord shows up no sooner than 3 s after the write,
  * the clear is immediate,
  * a held FaultCode shows up after four samples, not on the first.

Usage: test_opcua_raise_qualifier.test.py <test_alarm_server>
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

COMPONENT = 'qualifier_runtime'
ENTITY = 'plc_app'
CONTROL = 'TANK_ABOVE_40'
HELD = 'STATUS_BIT0_HELD'
REPEATED = 'FAULT_CODE_REPEATED'
HOLD_MS = 3000
SAMPLES = 4
POLL_MS = 500

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
    if not wait_log(log_path, 'READY ', deadline=25 * SCALE):
        terminate(proc)
        return None
    return proc


def set_register(server, log_path, name, value):
    """Write one Int32 register through the fixture and wait for its OK line."""
    needle = f'OK {name}={value}'
    before = Path(log_path).read_text(errors='replace').count(needle)
    server.stdin.write(f'set {name} {value}\n')
    server.stdin.flush()
    for _ in range(int(40 * SCALE)):
        if Path(log_path).read_text(errors='replace').count(needle) > before:
            return True
        time.sleep(0.05)
    return False


def node_map_text():
    """Return the map with one plain rule and two qualified ones."""
    return (
        'area_id: plc_systems\n'
        f'component_id: {COMPONENT}\n'
        'nodes:\n'
        '  - node_id: "ns=2;s=Tank.Level"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: tank_level\n'
        '    data_type: float\n'
        '    alarm:\n'
        f'      fault_code: {CONTROL}\n'
        '      threshold: 40.0\n'
        '  - node_id: "ns=2;s=StatusWord"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: status_word\n'
        '    data_type: int\n'
        f'    raise_after_ms: {HOLD_MS}\n'
        '    status_bits:\n'
        '      - bit: 0\n'
        f'        fault_code: {HELD}\n'
        '  - node_id: "ns=2;s=FaultCode"\n'
        f'    entity_id: {ENTITY}\n'
        '    data_name: fault_code\n'
        '    data_type: int\n'
        f'    raise_after_samples: {SAMPLES}\n'
        '    alarm:\n'
        f'      fault_code: {REPEATED}\n'
        '      threshold: 0.0\n'
    )


def write_params(path, *, port, plugin, server_port, node_map, manifest, subscriptions):
    """Render a gateway params file for one transport."""
    path.write_text('\n'.join([
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    server:',
        '      host: "127.0.0.1"',
        f'      port: {port}',
        '    plugins: ["opcua"]',
        f'    plugins.opcua.path: "{plugin}"',
        f'    plugins.opcua.endpoint_url: "opc.tcp://127.0.0.1:{server_port}"',
        f'    plugins.opcua.node_map_path: "{node_map}"',
        f'    plugins.opcua.poll_interval_ms: {POLL_MS}',
        f'    plugins.opcua.prefer_subscriptions: {"true" if subscriptions else "false"}',
        f'    plugins.opcua.subscription_interval_ms: {POLL_MS}.0',
        '    discovery.mode: "hybrid"',
        f'    discovery.manifest_path: "{manifest}"',
        '    discovery.manifest_strict_validation: false',
    ]) + '\n')


def start_gateway(params_file, log_path, env):
    """Launch a gateway_node bound to the given params file."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
         '--ros-args', '--params-file', str(params_file)],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    proc._log = log
    return proc


def active(status_url):
    """Return the set of faults the plugin holds active, or None if unreadable."""
    status = http_json(status_url)
    if not isinstance(status, dict):
        return None
    return set(status.get('active_alarms', []))


def first_active(status_url, code, deadline, period=0.1):
    """Seconds until <code> is active, or None if it is not within <deadline>."""
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        now = active(status_url)
        if now is not None and code in now:
            return time.monotonic() - start
        time.sleep(period)
    return None


def stays_inactive(status_url, code, seconds, period=0.1):
    """Return True when <code> is never active during <seconds>."""
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        now = active(status_url)
        if now is not None and code in now:
            return False
        time.sleep(period)
    return True


def run_leg(workdir, env, plugin, server, server_log, server_port, manifest, subscriptions):
    """One gateway on one transport, through every case."""
    mode = 'subscription' if subscriptions else 'poll'
    print(f'--- {mode} leg ---')
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(node_map_text())
    port = free_port()
    params = workdir / f'gateway_{mode}.yaml'
    write_params(params, port=port, plugin=plugin, server_port=server_port,
                 node_map=node_map, manifest=manifest, subscriptions=subscriptions)
    log = workdir / f'gateway_{mode}.log'
    gw = start_gateway(params, log, env)
    status_url = f'http://127.0.0.1:{port}/api/v1/components/{COMPONENT}/x-plc-status'
    try:
        # Connected, on the transport this leg is about, and the control rule
        # without a qualifier already active.
        ready = False
        for _ in range(int(90 * SCALE / 0.5)):
            status = http_json(status_url)
            if isinstance(status, dict) and status.get('connected') is True \
                    and CONTROL in status.get('active_alarms', []):
                ready = True
                break
            time.sleep(0.5)
        if not check(ready, f'{mode}: connected and {CONTROL} active with no qualifier'):
            print(log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return
        check(status.get('mode') == mode,
              f'{mode}: the plugin runs in {mode} mode (got {status.get("mode")!r})')

        # 1. A blip shorter than the window. Its real length includes the time the
        #    fixture takes to confirm each write, so it is measured, not assumed.
        blip_start = time.monotonic()
        check(set_register(server, server_log, 'StatusWord', 1),
              f'{mode}: StatusWord set to 1 for the blip')
        time.sleep(1.0)
        check(set_register(server, server_log, 'StatusWord', 0),
              f'{mode}: StatusWord back to 0 after the blip')
        blip = time.monotonic() - blip_start
        check(blip < HOLD_MS / 1000.0,
              f'{mode}: the blip lasted {blip:.2f} s, less than {HOLD_MS} ms')
        check(stays_inactive(status_url, HELD, (HOLD_MS / 1000.0 + 2.0) * SCALE),
              f'{mode}: a 1 s StatusWord blip never raises {HELD}')

        # 2. A held condition. The wait starts at the first sample, which is no
        #    earlier than the write, so the fault cannot show up sooner.
        check(set_register(server, server_log, 'StatusWord', 1), f'{mode}: StatusWord set to 1')
        elapsed = first_active(status_url, HELD, (HOLD_MS / 1000.0 + 5.0) * SCALE)
        check(elapsed is not None, f'{mode}: a held StatusWord raises {HELD}')
        if elapsed is not None:
            check(elapsed >= HOLD_MS / 1000.0 - 0.1,
                  f'{mode}: {HELD} shows up no sooner than {HOLD_MS} ms (after {elapsed:.2f} s)')

        # 3. The clear does not wait.
        set_register(server, server_log, 'StatusWord', 0)
        cleared = None
        start = time.monotonic()
        while time.monotonic() - start < 3.0 * SCALE:
            now = active(status_url)
            if now is not None and HELD not in now:
                cleared = time.monotonic() - start
                break
            time.sleep(0.1)
        check(cleared is not None, f'{mode}: {HELD} clears once StatusWord is 0')

        # 4. Samples. Under a subscription the register sends one change and
        #    the poll loop counts one more sample per cycle while it holds.
        check(set_register(server, server_log, 'FaultCode', 5), f'{mode}: FaultCode set to 5')
        elapsed = first_active(status_url, REPEATED, (SAMPLES * POLL_MS / 1000.0 + 6.0) * SCALE)
        check(elapsed is not None, f'{mode}: a held FaultCode raises {REPEATED}')
        if elapsed is not None:
            least = (SAMPLES - 1) * POLL_MS / 1000.0 - 0.3
            check(elapsed >= least,
                  f'{mode}: {REPEATED} needs {SAMPLES} samples, not one (after {elapsed:.2f} s)')
        set_register(server, server_log, 'FaultCode', 0)
        check(first_active(status_url, CONTROL, 5 * SCALE) is not None,
              f'{mode}: {CONTROL} still active at the end of the leg')
    finally:
        terminate(gw)
        # Leave the registers at 0 for the next leg; a server that is already
        # gone is reported by the next leg's own checks.
        try:
            set_register(server, server_log, 'StatusWord', 0)
            set_register(server, server_log, 'FaultCode', 0)
        except (BrokenPipeError, ValueError, OSError):
            pass


def main():
    if len(sys.argv) < 2:
        print('usage: test_opcua_raise_qualifier.test.py <test_alarm_server>', file=sys.stderr)
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

    workdir = Path(tempfile.mkdtemp(prefix='opcua_raise_qualifier_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    manifest = workdir / 'manifest.yaml'
    manifest.write_text('manifest_version: "1.0"\n')
    server_port = free_port()
    server_log = workdir / 'alarm_server.log'
    server = start_server(server_bin, server_port, server_log)
    try:
        if server is None:
            print('FAIL: fixture server did not become READY', file=sys.stderr)
            print(server_log.read_text(errors='replace'), file=sys.stderr)
            return 1
        for subscriptions in (False, True):
            run_leg(workdir, env, plugin, server, server_log, server_port, manifest, subscriptions)

        if failures:
            print(f'FAIL: {len(failures)} expectation(s) not met:', file=sys.stderr)
            for f in failures:
                print(f'  - {f}', file=sys.stderr)
            return 1
        print('PASS: raise qualifiers hold back a raise on both transports')
        return 0
    finally:
        terminate(server)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
