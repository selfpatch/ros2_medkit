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
PLC_COMMS_LOST heal decisions against a real fault manager.

Each scenario runs test_alarm_server, fault_manager_node and gateway_node with
the opcua plugin, and reads the fault over HTTP. A real outage raises the fault.

  * pinned stand-in: a gateway that started before its PLC raised the fault
    under the stand-in of its endpoint. After a restart with the PLC up, the
    fault is cleared.
  * foreign stand-in: a fault another endpoint's gateway raised stays.
  * late nameplate: a fault raised under the nameplate id stays while the device
    serves no nameplate, and is cleared when a later read names the device.
  * binding directory: a binding file whose directory cannot be looked up is
    reported, and the plugin stays connected. Once at start, and once on the
    poll thread when a rescan adopts a PLC that came up later.

Usage: test_opcua_comms_lost.test.py <test_alarm_server>
"""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

COMMS_LOST = 'PLC_COMMS_LOST'
HELD_BY_OTHERS = f'{COMMS_LOST} is held by sources this bridge did not report under ['
CLEARING = f"clearing this bridge's {COMMS_LOST}"
# Discovery identifies OPC-UA only on this port.
DISCOVERY_PORT = 4840


def free_port():
    """Return a free TCP port on the loopback."""
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


def http_json(url, timeout=2):
    """GET <url> and parse JSON, or return None on any failure."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def wait_log(path, needle, deadline):
    """Poll a log file until <needle> appears."""
    end = time.monotonic() + deadline
    while time.monotonic() < end:
        try:
            if needle in Path(path).read_text(errors='replace'):
                return True
        except OSError:
            pass
        time.sleep(0.2)
    return False


def terminate(proc):
    """SIGTERM, then SIGKILL, the process group of <proc>."""
    if proc is None:
        return
    pgid = None
    if proc.returncode is None:
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pgid = None

    def signal_group(sig):
        if pgid is not None:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass

    signal_group(signal.SIGTERM)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGKILL)
        proc.wait()
    # ros2 run can exit before the node it started.
    signal_group(signal.SIGKILL)
    log = getattr(proc, '_log', None)
    if log is not None:
        log.close()


class Gateway:
    """One gateway_node process and its HTTP base URL."""

    def __init__(self, proc, port, log):
        self.proc = proc
        self.port = port
        self.log = log
        self.base = f'http://127.0.0.1:{port}/api/v1'


class Run:
    """The processes of one scenario. stop_all() ends every one of them."""

    def __init__(self, workdir, server_bin, plugin, env):
        self.workdir = workdir
        self.server_bin = server_bin
        self.plugin = plugin
        self.env = env
        self.procs = []

    def _start(self, cmd, log_path, env=None, stdin=False):
        log = open(log_path, 'w')
        proc = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT, env=env,
            stdin=subprocess.PIPE if stdin else None, text=True,
            start_new_session=True,
        )
        proc._log = log
        self.procs.append(proc)
        return proc

    def stop(self, proc):
        terminate(proc)
        if proc in self.procs:
            self.procs.remove(proc)

    def stop_all(self):
        for proc in reversed(self.procs):
            terminate(proc)
        self.procs = []

    def start_server(self, name, port, *extra):
        """Start the fixture PLC and wait for READY."""
        log = self.workdir / f'{name}_server.log'
        proc = self._start([str(self.server_bin), '--port', str(port), *extra], log, stdin=True)
        if not wait_log(log, 'READY ', 20):
            raise AssertionError(f'fixture on port {port} never became READY:\n'
                                 + log.read_text(errors='replace'))
        return proc

    def start_fault_manager(self, name):
        """Start fault_manager_node with memory storage and wait for its services."""
        log = self.workdir / f'{name}_fault_manager.log'
        proc = self._start(['ros2', 'run', 'ros2_medkit_fault_manager', 'fault_manager_node',
                            '--ros-args', '-p', 'storage_type:=memory'], log, env=self.env)
        end = time.monotonic() + 30
        while time.monotonic() < end:
            if proc.poll() is not None:
                break
            out = subprocess.run(['ros2', 'service', 'list', '--no-daemon'], capture_output=True,
                                 text=True, env=self.env, timeout=15, check=False)
            services = out.stdout
            if '/fault_manager/get_fault' in services and '/fault_manager/clear_fault' in services:
                return proc
            time.sleep(0.5)
        raise AssertionError('fault_manager_node never offered its services:\n'
                             + log.read_text(errors='replace'))

    def start_gateway(self, name, endpoint=None, poll_ms=200, extra_env=None):
        """Start gateway_node with the opcua plugin and wait for HTTP."""
        port = free_port()
        params = self.workdir / f'{name}_gateway.yaml'
        lines = [
            'ros2_medkit_gateway:',
            '  ros__parameters:',
            '    server:',
            '      host: "127.0.0.1"',
            f'      port: {port}',
            '    plugins: ["opcua"]',
            f'    plugins.opcua.path: "{self.plugin}"',
            f'    plugins.opcua.poll_interval_ms: {poll_ms}',
            '    plugins.opcua.comms_lost_debounce_ms: 0',
        ]
        if endpoint is not None:
            lines.append(f'    plugins.opcua.endpoint_url: "{endpoint}"')
        params.write_text('\n'.join(lines) + '\n')
        env = dict(self.env)
        env.update(extra_env or {})
        log = self.workdir / f'{name}_gateway.log'
        proc = self._start(['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
                            '--ros-args', '--params-file', str(params)], log, env=env)
        gateway = Gateway(proc, port, log)
        end = time.monotonic() + 30
        while time.monotonic() < end:
            if http_json(f'{gateway.base}/health') is not None:
                return gateway
            if proc.poll() is not None:
                break
            time.sleep(0.3)
        raise AssertionError(f'gateway {name} never answered /health:\n'
                             + log.read_text(errors='replace')[-4000:])


def tail(gateway):
    """Return the end of a gateway's log."""
    return gateway.log.read_text(errors='replace')[-4000:]


def comms_lost_status(gateway):
    """Return the PLC_COMMS_LOST status the fault manager reports, or None."""
    listing = http_json(f'{gateway.base}/faults?status=all')
    if listing is None:
        return None
    for item in listing.get('items', []):
        if item.get('fault_code') == COMMS_LOST:
            return item.get('status')
    return 'ABSENT'


def wait_status(gateway, wanted, deadline):
    """Poll until PLC_COMMS_LOST has status <wanted>. Return the last status."""
    end = time.monotonic() + deadline
    status = None
    while time.monotonic() < end:
        status = comms_lost_status(gateway)
        if status == wanted:
            return status
        time.sleep(0.3)
    return status


def expect_status(gateway, wanted, deadline, what):
    status = wait_status(gateway, wanted, deadline)
    if status != wanted:
        raise AssertionError(f'{what}: {COMMS_LOST} is {status}, expected {wanted}. '
                             'Gateway log tail:\n' + tail(gateway))


def raise_while_down(run, name, endpoint):
    """Run a gateway against an endpoint nobody serves until it raises PLC_COMMS_LOST."""
    gateway = run.start_gateway(name, endpoint=endpoint)
    expect_status(gateway, 'CONFIRMED', 60, f'{name}: the outage was never reported')
    run.stop(gateway.proc)


def scenario_pinned_stand_in(run):
    port = free_port()
    endpoint = f'opc.tcp://127.0.0.1:{port}'
    run.start_fault_manager('pinned')
    raise_while_down(run, 'pinned_before', endpoint)

    run.start_server('pinned', port)
    after = run.start_gateway('pinned_after', endpoint=endpoint)
    expect_status(after, 'CLEARED', 60,
                  'a fault raised under the stand-in of the pinned endpoint '
                  'was not cleared after a restart')


def scenario_foreign_stand_in(run):
    port = free_port()
    run.start_fault_manager('foreign')
    raise_while_down(run, 'foreign_before', f'opc.tcp://127.0.0.2:{free_port()}')

    run.start_server('foreign', port)
    after = run.start_gateway('foreign_after', endpoint=f'opc.tcp://127.0.0.1:{port}')
    if not wait_log(after.log, HELD_BY_OTHERS, 60):
        raise AssertionError('the gateway never decided on the standing fault:\n'
                             + tail(after))
    time.sleep(3)
    expect_status(after, 'CONFIRMED', 1, "a fault another endpoint's gateway raised was cleared")


def scenario_late_nameplate(run):
    port = free_port()
    endpoint = f'opc.tcp://127.0.0.1:{port}'
    run.start_fault_manager('late')
    named = run.start_server('late_named', port)
    before = run.start_gateway('late_before', endpoint=endpoint)
    if not wait_log(before.log, 'Connected to OPC-UA server', 30):
        raise AssertionError('the first gateway never connected:\n'
                             + tail(before))
    # No fault stands yet, so the connect decision has nothing to report.
    time.sleep(3)
    if HELD_BY_OTHERS in before.log.read_text(errors='replace'):
        raise AssertionError('a connect with no fault standing reported one '
                             'held by other sources:\n'
                             + tail(before))
    run.stop(named)
    expect_status(before, 'CONFIRMED', 60, 'the outage of a named device was never reported')
    run.stop(before.proc)

    unnamed = run.start_server('late_unnamed', port, '--no-nameplate')
    # Five identity reads at 1500 ms leave room to name the device after the decision.
    after = run.start_gateway('late_after', endpoint=endpoint, poll_ms=1500)
    if not wait_log(after.log, HELD_BY_OTHERS, 60):
        raise AssertionError('the gateway never left the nameplate row standing at connect:\n'
                             + tail(after))
    expect_status(after, 'CONFIRMED', 1,
                  'the row was cleared while the device served no nameplate')

    unnamed.stdin.write('nameplate\n')
    unnamed.stdin.flush()
    expect_status(after, 'CLEARED', 30,
                  'a fault raised under the nameplate id was not cleared '
                  'after the device named itself')


def start_discovering_gateway(run, name):
    """Start a config-less gateway whose binding file sits under a symlink loop."""
    loop_a = run.workdir / 'loop_a'
    loop_b = run.workdir / 'loop_b'
    loop_a.symlink_to(loop_b)
    loop_b.symlink_to(loop_a)
    binding = loop_a / 'binding'
    gateway = run.start_gateway(name, extra_env={
        'OPCUA_DISCOVERY_ENABLED': '1',
        'OPCUA_DISCOVERY_SUBNETS': '127.0.0.1/32',
        'OPCUA_DISCOVERY_BINDING_FILE': str(binding),
    })
    return gateway, binding


def plc_connected(gateway):
    """Return True when some component's x-plc-status reports a live session."""
    listing = http_json(f'{gateway.base}/components') or {}
    for item in listing.get('items', []):
        status = http_json(f"{gateway.base}/components/{item.get('id')}/x-plc-status")
        if status and status.get('connected') is True:
            return True
    return False


def expect_binding_failure_survived(gateway, binding, deadline):
    if not wait_log(gateway.log, f'but could not keep it in {binding}', deadline):
        raise AssertionError('no failed binding write was reported '
                             f'(gateway rc={gateway.proc.poll()}):\n'
                             + tail(gateway))
    time.sleep(3)
    if gateway.proc.poll() is not None or http_json(f'{gateway.base}/health') is None:
        raise AssertionError(f'the gateway did not survive the failed binding write '
                             f'(rc={gateway.proc.poll()}):\n'
                             + tail(gateway))
    if not plc_connected(gateway):
        raise AssertionError('no PLC session is up after the failed binding write:\n'
                             + tail(gateway))


def scenario_binding_directory_at_start(run):
    run.start_server('binding_start', DISCOVERY_PORT)
    gateway, binding = start_discovering_gateway(run, 'binding_start')
    expect_binding_failure_survived(gateway, binding, 90)


def scenario_binding_directory_on_reconnect(run):
    # The PLC comes up after the start-up sweep, so a rescan in the reconnect arm adopts it.
    gateway, binding = start_discovering_gateway(run, 'binding_reconnect')
    if not wait_log(gateway.log, 'OPC-UA discovery summary: 0 data server(s)', 30):
        raise AssertionError('the start-up sweep did not come back empty:\n'
                             + tail(gateway))
    run.start_server('binding_reconnect', DISCOVERY_PORT)
    expect_binding_failure_survived(gateway, binding, 120)


SCENARIOS = [
    ('pinned stand-in', scenario_pinned_stand_in),
    ('foreign stand-in', scenario_foreign_stand_in),
    ('late nameplate', scenario_late_nameplate),
    ('binding directory at start', scenario_binding_directory_at_start),
    ('binding directory on reconnect', scenario_binding_directory_on_reconnect),
]


def main():
    if len(sys.argv) < 2:
        print('usage: test_opcua_comms_lost.test.py <test_alarm_server>', file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
    if not ros_domain_id:
        print('ROS_DOMAIN_ID is not set: run this test through CTest', file=sys.stderr)
        return 1
    if not (server_bin.is_file() and os.access(server_bin, os.X_OK)):
        print(f'fixture missing: {server_bin}', file=sys.stderr)
        return 1
    plugin = find_plugin()
    if plugin is None:
        print('libros2_medkit_opcua_plugin.so not found', file=sys.stderr)
        return 1

    env = dict(os.environ, ROS_DOMAIN_ID=ros_domain_id)
    only = os.environ.get('OPCUA_COMMS_LOST_SCENARIO')
    failures = 0
    for name, scenario in SCENARIOS:
        if only and only != name:
            continue
        workdir = Path(tempfile.mkdtemp(prefix='opcua_comms_lost_'))
        run = Run(workdir, server_bin, plugin, env)
        try:
            scenario(run)
            print(f'  OK {name}')
        except AssertionError as e:
            failures += 1
            print(f'FAIL {name}: {e}', file=sys.stderr)
        finally:
            run.stop_all()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
