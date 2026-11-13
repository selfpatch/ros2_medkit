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

"""
A device alarm of a per-source code clears the gateway's own record, through a real gateway.

The node map gives StatusWord bit 0 a fault code that the fault manager keeps one record per
source for, and a second source holds a record of the same code. Raising the bit raises the
gateway's record and clearing it clears that record alone: the plugin names the entity it
reports under in its ClearFault, so the clear is not refused as ambiguous.

Usage: test_opcua_per_source_clear.test.py <test_alarm_server>
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

COMPONENT = 'per_source_runtime'
ENTITY = 'plc_app'
CODE = 'PS_STATUS_BIT0'
OTHER = '/other_source'
POLL_MS = 200

SCALE = 1.0


def time_scale():
    """Return MEDKIT_TEST_TIME_SCALE, the factor the sanitizer jobs apply to wall-clock budgets."""
    try:
        scale = float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1'))
    except ValueError:
        return 1.0
    return scale if scale >= 1.0 else 1.0


SCALE = time_scale()


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


def start_fault_manager(log_path, env):
    """Start fault_manager_node with memory storage and wait until its services exist."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'ros2_medkit_fault_manager', 'fault_manager_node',
         '--ros-args', '-p', 'storage_type:=memory'],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    proc._log = log
    # Logged at the end of the constructor, after the services exist.
    started = wait_log(log_path, 'FaultManager node started (', 30 * SCALE)
    if not started or proc.poll() is not None:
        terminate(proc)
        return None
    return proc


FAULT_MANAGER_CLIENT = """
import json
import sys

import rclpy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

rclpy.init()
node = rclpy.create_node('per_source_clear_test_client')


def call(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


if sys.argv[1] == 'report':
    client = node.create_client(ReportFault, '/fault_manager/report_fault')
    client.wait_for_service(timeout_sec=10.0)
    request = ReportFault.Request()
    request.fault_code = sys.argv[2]
    request.event_type = ReportFault.Request.EVENT_FAILED
    request.severity = Fault.SEVERITY_ERROR
    request.description = 'second source'
    request.source_id = sys.argv[3]
    print(json.dumps({'accepted': call(client, request).accepted}))
else:
    client = node.create_client(ListFaults, '/fault_manager/list_faults')
    client.wait_for_service(timeout_sec=10.0)
    request = ListFaults.Request()
    request.statuses = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']
    print(json.dumps({f.reporting_sources[0]: f.status for f in call(client, request).faults
                      if f.fault_code == sys.argv[2] and f.reporting_sources}))
node.destroy_node()
rclpy.shutdown()
"""


def fault_manager_client(env, *args):
    result = subprocess.run([sys.executable, '-c', FAULT_MANAGER_CLIENT, *args], env=env,
                            capture_output=True, text=True, timeout=60 * SCALE)
    if result.returncode != 0:
        raise AssertionError(f'fault manager client {args} failed:\n{result.stderr}')
    return json.loads(result.stdout)


def wait_records(env, accept, deadline):
    """Poll a scaled deadline until accept(records) holds. Return the last records."""
    end = time.monotonic() + deadline * SCALE
    records = {}
    while time.monotonic() < end:
        records = fault_manager_client(env, 'list', CODE)
        if accept(records):
            return records
        time.sleep(0.3)
    return records


def main():
    global SCALE
    if len(sys.argv) < 2:
        print('usage: test_opcua_per_source_clear.test.py <test_alarm_server>', file=sys.stderr)
        return 2
    SCALE = time_scale()
    server_bin = Path(sys.argv[1]).resolve()
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: run this test through CTest', file=sys.stderr)
        return 1
    plugin = find_plugin()
    if plugin is None or not server_bin.is_file():
        print('plugin or fixture missing', file=sys.stderr)
        return 1

    workdir = Path(tempfile.mkdtemp(prefix='opcua_per_source_clear_'))
    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    procs = []
    try:
        node_map = workdir / 'nodes.yaml'
        node_map.write_text(
            'area_id: plc_systems\n'
            f'component_id: {COMPONENT}\n'
            'nodes:\n'
            '  - node_id: "ns=2;s=StatusWord"\n'
            f'    entity_id: {ENTITY}\n'
            '    data_name: status_word\n'
            '    data_type: int\n'
            '    status_bits:\n'
            '      - bit: 0\n'
            f'        fault_code: {CODE}\n')
        manifest = workdir / 'manifest.yaml'
        manifest.write_text('manifest_version: "1.0"\n')
        thresholds = workdir / 'thresholds.yaml'
        thresholds.write_text(f'{CODE}:\n  per_source: true\n')

        server_port = free_port()
        server_log = workdir / 'alarm_server.log'
        server = start_server(server_bin, server_port, server_log)
        if server is None:
            print('fixture server did not become READY', file=sys.stderr)
            return 1
        procs.append(server)

        fm_log = workdir / 'fault_manager.log'
        fm_proc_log = open(fm_log, 'w')
        fm = subprocess.Popen(
            ['ros2', 'run', 'ros2_medkit_fault_manager', 'fault_manager_node', '--ros-args',
             '-p', 'storage_type:=memory', '-p', f'fault_thresholds.config_file:={thresholds}'],
            stdout=fm_proc_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        fm._log = fm_proc_log
        procs.append(fm)
        if not wait_log(fm_log, 'FaultManager node started (', 30 * SCALE):
            print(fm_log.read_text(errors='replace'), file=sys.stderr)
            return 1

        port = free_port()
        params = workdir / 'gateway.yaml'
        params.write_text('\n'.join([
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
            '    discovery.mode: "hybrid"',
            f'    discovery.manifest_path: "{manifest}"',
            '    discovery.manifest_strict_validation: false',
        ]) + '\n')
        gateway_log = workdir / 'gateway.log'
        gateway = start_gateway(params, gateway_log, env)
        procs.append(gateway)
        status_url = f'http://127.0.0.1:{port}/api/v1/components/{COMPONENT}/x-plc-status'
        end = time.monotonic() + 90 * SCALE
        while time.monotonic() < end:
            status = http_json(status_url)
            if isinstance(status, dict) and status.get('connected') is True:
                break
            time.sleep(0.5)
        else:
            tail = gateway_log.read_text(errors='replace')[-3000:]
            print('the gateway never connected:\n' + tail, file=sys.stderr)
            return 1

        # A second source holds a record of the same code.
        if not fault_manager_client(env, 'report', CODE, OTHER)['accepted']:
            print('the second source was refused', file=sys.stderr)
            return 1

        set_register(server, server_log, 'StatusWord', 1)
        records = wait_records(
            env, lambda r: r.get(ENTITY) == 'CONFIRMED' and r.get(OTHER) == 'CONFIRMED', 30)
        if records.get(ENTITY) != 'CONFIRMED':
            print(f'the alarm never raised the gateway record: {records}', file=sys.stderr)
            return 1

        set_register(server, server_log, 'StatusWord', 0)
        records = wait_records(env, lambda r: r.get(ENTITY) == 'CLEARED', 30)
        if records.get(ENTITY) != 'CLEARED':
            print(f"the cleared alarm did not clear the gateway's record: {records}\n"
                  + fm_log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return 1
        if records.get(OTHER) != 'CONFIRMED':
            print(f"the other source's record changed: {records}", file=sys.stderr)
            return 1
        print('PASS: a device alarm clear reaches only the gateway record of a per-source code')
        return 0
    finally:
        for proc in reversed(procs):
            terminate(proc)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
