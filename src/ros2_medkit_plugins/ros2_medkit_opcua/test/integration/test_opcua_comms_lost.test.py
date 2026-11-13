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
  * per-source record: PLC_COMMS_LOST configured with one record per source and
    a second source that reported it. The restored link clears this gateway's own
    record and leaves the other source's alone.
  * per-source record under a stand-in: the gateway raises the fault under the stand-in
    of its endpoint, the PLC comes up and names the component, and the same gateway
    clears the record it raised under the stand-in.
  * per-source pinned stand-in: the same, with the gateway restarted in between, so it remembers
    nothing and the record is found under the stand-in derived from its endpoint.
  * explicit component id: a gateway whose node map names a component_id leaves the
    per-source record a default-id bridge holds under 'opcua_device' alone.
  * explicit component id, stand-in record: the same, for a record held under the
    endpoint-derived stand-in of a config-less bridge on the same host, per-source and shared.
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'fixtures' / 'test_alarm_server'))
import alarm_fixture  # noqa: E402

COMMS_LOST = 'PLC_COMMS_LOST'
HELD_BY_OTHERS = f'{COMMS_LOST} is held by sources this bridge did not report under ['
# Discovery identifies OPC-UA only on this port.
DISCOVERY_PORT = 4840


def time_scale():
    """Return MEDKIT_TEST_TIME_SCALE, the factor the sanitizer jobs apply to wall-clock budgets."""
    try:
        scale = float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1'))
    except ValueError:
        return 1.0
    return scale if scale >= 1.0 else 1.0


SCALE = time_scale()


def settle(seconds):
    """Sleep a scaled settle window."""
    time.sleep(seconds * SCALE)


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
        for root, _, files in os.walk(prefix):
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
    """Poll a log file until <needle> appears, for a scaled deadline."""
    end = time.monotonic() + deadline * SCALE
    while time.monotonic() < end:
        try:
            if needle in Path(path).read_text(errors='replace'):
                return True
        except OSError:
            pass
        time.sleep(0.2)
    return False


def signal_group(pgid, sig):
    """Send <sig> to a process group. Return False when the group has no process left."""
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return False
    return True


def group_running(pgid):
    """
    Return True while a process of group <pgid> runs.

    A zombie does not count: PID 1 of a CI job container does not reap orphans.
    """
    for stat in Path('/proc').glob('[0-9]*/stat'):
        try:
            # After the command name: state, parent pid, process group.
            fields = stat.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
                return True
        except (OSError, IndexError, ValueError):
            continue
    return False


def terminate(proc):
    """
    SIGTERM the process group of <proc> and wait until all of it has exited.

    A node killed during its shutdown leaves its Fast DDS files in /dev/shm. SIGKILL
    comes only after a scaled grace period.
    """
    if proc.poll() is not None:
        return
    # The fixture's stdin reader ends only on EOF.
    if proc.stdin is not None:
        try:
            proc.stdin.close()
        except OSError:
            pass
    # start_new_session makes the child the group leader.
    pgid = proc.pid
    signal_group(pgid, signal.SIGTERM)
    end = time.monotonic() + 15 * SCALE
    # ros2 run exits before the node it started, so wait for the group.
    while time.monotonic() < end:
        proc.poll()
        if not group_running(pgid):
            proc.wait()
            return
        time.sleep(0.1)
    signal_group(pgid, signal.SIGKILL)
    proc.wait()


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
        self.logs = {}
        # Logs of fixtures that printed READY. A failed start printed its own.
        self.server_logs = []

    def _start(self, cmd, log_path, env=None, stdin=False):
        log = open(log_path, 'w')
        proc = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT, env=env,
            stdin=subprocess.PIPE if stdin else None, text=True,
            start_new_session=True,
        )
        self.logs[proc.pid] = log
        self.procs.append(proc)
        return proc

    def stop(self, proc):
        terminate(proc)
        self.logs.pop(proc.pid).close()
        self.procs.remove(proc)

    def stop_all(self):
        for proc in reversed(self.procs):
            terminate(proc)
            self.logs.pop(proc.pid).close()
        self.procs = []

    def start_server(self, name, *extra, port=None):
        """
        Start the fixture PLC and return (proc, port) once it listens.

        Without port it runs on a newly reserved one. With port it must keep that
        port, and fails when another process took it.
        """
        log = self.workdir / f'{name}_server.log'
        proc, port = alarm_fixture.start_fixture(self.server_bin, log, *extra, port=port,
                                                 deadline=20 * SCALE)
        if proc is None:
            raise AssertionError(f'fixture {name} printed no READY (last port {port})')
        self.logs[proc.pid] = proc._log
        self.procs.append(proc)
        self.server_logs.append(log)
        return proc, port

    def start_fault_manager(self, name, per_source_codes=()):
        """Start fault_manager_node with memory storage."""
        log = self.workdir / f'{name}_fault_manager.log'
        args = ['-p', 'storage_type:=memory']
        if per_source_codes:
            thresholds = self.workdir / f'{name}_fault_thresholds.yaml'
            thresholds.write_text(''.join(f'{code}:\n  per_source: true\n'
                                          for code in per_source_codes))
            args += ['-p', f'fault_thresholds.config_file:={thresholds}']
        proc = self._start(['ros2', 'run', 'ros2_medkit_fault_manager', 'fault_manager_node',
                            '--ros-args', *args], log, env=self.env)
        # Logged at the end of the constructor, after the services exist.
        if not wait_log(log, 'FaultManager node started (', 30) or proc.poll() is not None:
            raise AssertionError(f'fault_manager_node did not start (rc={proc.poll()}):\n'
                                 + log.read_text(errors='replace'))
        return proc

    def start_gateway(self, name, endpoint=None, poll_ms=200, extra_env=None, node_map=None):
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
        if node_map is not None:
            lines.append(f'    plugins.opcua.node_map_path: "{node_map}"')
        params.write_text('\n'.join(lines) + '\n')
        env = dict(self.env)
        env.update(extra_env or {})
        log = self.workdir / f'{name}_gateway.log'
        proc = self._start(['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node',
                            '--ros-args', '--params-file', str(params)], log, env=env)
        gateway = Gateway(proc, port, log)
        end = time.monotonic() + 30 * SCALE
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
    """Poll a scaled deadline until PLC_COMMS_LOST has status <wanted>. Return the last one."""
    end = time.monotonic() + deadline * SCALE
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

    # The gateway above was pinned to this port, so the fixture must take it.
    run.start_server('pinned', port=port)
    after = run.start_gateway('pinned_after', endpoint=endpoint)
    expect_status(after, 'CLEARED', 60,
                  'a fault raised under the stand-in of the pinned endpoint '
                  'was not cleared after a restart')


def scenario_foreign_stand_in(run):
    run.start_fault_manager('foreign')
    raise_while_down(run, 'foreign_before', f'opc.tcp://127.0.0.2:{free_port()}')

    _, port = run.start_server('foreign')
    after = run.start_gateway('foreign_after', endpoint=f'opc.tcp://127.0.0.1:{port}')
    if not wait_log(after.log, HELD_BY_OTHERS, 60):
        raise AssertionError('the gateway never decided on the standing fault:\n'
                             + tail(after))
    settle(3)
    expect_status(after, 'CONFIRMED', 1, "a fault another endpoint's gateway raised was cleared")


def scenario_late_nameplate(run):
    run.start_fault_manager('late')
    named, port = run.start_server('late_named')
    endpoint = f'opc.tcp://127.0.0.1:{port}'
    before = run.start_gateway('late_before', endpoint=endpoint)
    if not wait_log(before.log, 'Connected to OPC-UA server', 30):
        raise AssertionError('the first gateway never connected:\n'
                             + tail(before))
    # No fault stands yet, so the connect decision has nothing to report.
    settle(3)
    if HELD_BY_OTHERS in before.log.read_text(errors='replace'):
        raise AssertionError('a connect with no fault standing reported one '
                             'held by other sources:\n'
                             + tail(before))
    run.stop(named)
    expect_status(before, 'CONFIRMED', 60, 'the outage of a named device was never reported')
    run.stop(before.proc)

    # The same device reboots, so it keeps its port.
    unnamed, _ = run.start_server('late_unnamed', '--no-nameplate', port=port)
    # Five identity reads at this interval leave room to name the device after the decision.
    after = run.start_gateway('late_after', endpoint=endpoint, poll_ms=int(1500 * SCALE))
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


# Talks to the fault manager as a second source: reports a fault, lists the records.
FAULT_MANAGER_CLIENT = """
import json
import sys

import rclpy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

rclpy.init()
node = rclpy.create_node('comms_lost_test_client')


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
    print(json.dumps([{'code': f.fault_code, 'status': f.status,
                       'sources': list(f.reporting_sources)}
                      for f in call(client, request).faults]))
node.destroy_node()
rclpy.shutdown()
"""


def fault_manager_client(run, *args):
    """Run the fault manager client script on the scenario's ROS domain and return its JSON."""
    result = subprocess.run([sys.executable, '-c', FAULT_MANAGER_CLIENT, *args], env=run.env,
                            capture_output=True, text=True, timeout=60 * SCALE)
    if result.returncode != 0:
        raise AssertionError(f'fault manager client {args} failed:\n{result.stderr}')
    return json.loads(result.stdout)


def comms_lost_records(run):
    """Return {first reporting source: status} of every PLC_COMMS_LOST record."""
    return {f['sources'][0]: f['status'] for f in fault_manager_client(run, 'list')
            if f['code'] == COMMS_LOST and f['sources']}


def wait_records(run, accept, deadline):
    """Poll a scaled deadline until accept(records) holds. Return the last records."""
    end = time.monotonic() + deadline * SCALE
    records = {}
    while time.monotonic() < end:
        records = comms_lost_records(run)
        if accept(records):
            return records
        time.sleep(0.3)
    return records


def scenario_per_source_record(run):
    other = '/other_plc'
    run.start_fault_manager('persrc', per_source_codes=[COMMS_LOST])
    named, port = run.start_server('persrc_named')
    endpoint = f'opc.tcp://127.0.0.1:{port}'
    gateway = run.start_gateway('persrc', endpoint=endpoint)
    if not wait_log(gateway.log, 'Connected to OPC-UA server', 30):
        raise AssertionError('the gateway never connected:\n' + tail(gateway))

    # A second source holds a record of the same code before the outage.
    if not fault_manager_client(run, 'report', COMMS_LOST, other)['accepted']:
        raise AssertionError('the second source was refused')

    run.stop(named)
    records = wait_records(
        run, lambda r: len(r) == 2 and all(s == 'CONFIRMED' for s in r.values()), 60)
    if len(records) != 2 or any(s != 'CONFIRMED' for s in records.values()):
        raise AssertionError(f'the outage was never reported as its own record: {records}\n'
                             + tail(gateway))
    own = next(source for source in records if source != other)

    # The same device comes back on its port: the link is restored.
    run.start_server('persrc_back', port=port)
    records = wait_records(run, lambda r: r.get(own) == 'CLEARED', 60)
    if records.get(own) != 'CLEARED':
        raise AssertionError('the restored link did not clear this gateway record: '
                             f'{records}\n' + tail(gateway))
    if records.get(other) != 'CONFIRMED':
        raise AssertionError(f"the other source's record was cleared too: {records}")


def scenario_per_source_placeholder_rename(run):
    run.start_fault_manager('rename', per_source_codes=[COMMS_LOST])
    port = free_port()
    gateway = run.start_gateway('rename', endpoint=f'opc.tcp://127.0.0.1:{port}')
    records = wait_records(run, lambda r: len(r) == 1 and 'CONFIRMED' in r.values(), 60)
    if len(records) != 1 or 'CONFIRMED' not in records.values():
        raise AssertionError(f'the outage was never reported: {records}\n' + tail(gateway))
    stand_in = next(iter(records))

    # The PLC comes up on the endpoint the gateway was pinned to and names the component.
    run.start_server('rename', port=port)
    records = wait_records(run, lambda r: r.get(stand_in) == 'CLEARED', 60)
    if records.get(stand_in) != 'CLEARED':
        raise AssertionError(f'the record raised under the stand-in {stand_in} was stranded '
                             f'after the component was renamed: {records}\n' + tail(gateway))


def scenario_per_source_pinned_stand_in(run):
    run.start_fault_manager('persrc_pinned', per_source_codes=[COMMS_LOST])
    port = free_port()
    endpoint = f'opc.tcp://127.0.0.1:{port}'
    gateway = run.start_gateway('persrc_pinned_before', endpoint=endpoint)
    records = wait_records(run, lambda r: len(r) == 1 and 'CONFIRMED' in r.values(), 60)
    if len(records) != 1 or 'CONFIRMED' not in records.values():
        raise AssertionError(f'the outage was never reported: {records}\n' + tail(gateway))
    stand_in = next(iter(records))
    run.stop(gateway.proc)

    # The restarted gateway remembers nothing, and the PLC names the component.
    run.start_server('persrc_pinned', port=port)
    after = run.start_gateway('persrc_pinned_after', endpoint=endpoint)
    records = wait_records(run, lambda r: r.get(stand_in) == 'CLEARED', 60)
    if records.get(stand_in) != 'CLEARED':
        raise AssertionError(f'the record raised under the stand-in {stand_in} stayed standing '
                             f'after a restart: {records}\n' + tail(after))


def scenario_explicit_component_id_keeps_the_default_id_record(run):
    run.start_fault_manager('explicit_id', per_source_codes=[COMMS_LOST])
    default_id = 'opcua_device'
    # A bridge whose node map names no component_id holds an outage record under the default id.
    if not fault_manager_client(run, 'report', COMMS_LOST, default_id)['accepted']:
        raise AssertionError('the seeded record was refused')
    node_map = run.workdir / 'explicit_id_nodes.yaml'
    node_map.write_text('area_id: plc_area\ncomponent_id: plc_a\nnodes: []\n')
    _, port = run.start_server('explicit_id')
    gateway = run.start_gateway('explicit_id', endpoint=f'opc.tcp://127.0.0.1:{port}',
                                node_map=node_map)
    # The gateway asked for its own record, found none, and went through every id it reports under.
    if not wait_log(gateway.log, 'nothing to clear', 30):
        raise AssertionError('the gateway never decided on PLC_COMMS_LOST:\n' + tail(gateway))
    records = comms_lost_records(run)
    if records != {default_id: 'CONFIRMED'}:
        raise AssertionError('a bridge with an explicit component_id touched the record of a '
                             f'default-id bridge: {records}\n' + tail(gateway))


def scenario_explicit_component_id_keeps_a_stand_in_record(run):
    stand_in = 'opcua-127_0_0_1'
    run.start_fault_manager('stand_in_id', per_source_codes=[COMMS_LOST])
    # A config-less bridge on the same host holds an outage record under its stand-in.
    if not fault_manager_client(run, 'report', COMMS_LOST, stand_in)['accepted']:
        raise AssertionError('the seeded record was refused')
    node_map = run.workdir / 'stand_in_id_nodes.yaml'
    node_map.write_text('area_id: plc_area\ncomponent_id: plc_a\nnodes: []\n')
    _, port = run.start_server('stand_in_id')
    gateway = run.start_gateway('stand_in_id', endpoint=f'opc.tcp://127.0.0.1:{port}',
                                node_map=node_map)
    if not wait_log(gateway.log, 'nothing to clear', 30):
        raise AssertionError('the gateway never decided on PLC_COMMS_LOST:\n' + tail(gateway))
    records = comms_lost_records(run)
    if records != {stand_in: 'CONFIRMED'}:
        raise AssertionError('a bridge with a node map touched the record of a stand-in bridge: '
                             f'{records}\n' + tail(gateway))


def scenario_explicit_component_id_keeps_a_shared_stand_in_record(run):
    stand_in = 'opcua-127_0_0_1'
    run.start_fault_manager('stand_in_shared')
    if not fault_manager_client(run, 'report', COMMS_LOST, stand_in)['accepted']:
        raise AssertionError('the seeded record was refused')
    node_map = run.workdir / 'stand_in_shared_nodes.yaml'
    node_map.write_text('area_id: plc_area\ncomponent_id: plc_a\nnodes: []\n')
    _, port = run.start_server('stand_in_shared')
    gateway = run.start_gateway('stand_in_shared', endpoint=f'opc.tcp://127.0.0.1:{port}',
                                node_map=node_map)
    if not wait_log(gateway.log, HELD_BY_OTHERS, 30):
        raise AssertionError('the gateway never left the shared record standing:\n'
                             + tail(gateway))
    records = comms_lost_records(run)
    if records != {stand_in: 'CONFIRMED'}:
        raise AssertionError('a bridge with a node map cleared the shared record of a stand-in '
                             f'bridge: {records}\n' + tail(gateway))


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
        # Rescan every 2 s while disconnected.
        'OPCUA_DISCOVERY_INTERVAL_S': '2',
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
    settle(3)
    if gateway.proc.poll() is not None or http_json(f'{gateway.base}/health') is None:
        raise AssertionError(f'the gateway did not survive the failed binding write '
                             f'(rc={gateway.proc.poll()}):\n'
                             + tail(gateway))
    if not plc_connected(gateway):
        raise AssertionError('no PLC session is up after the failed binding write:\n'
                             + tail(gateway))


def scenario_binding_directory_at_start(run):
    run.start_server('binding_start', port=DISCOVERY_PORT)
    gateway, binding = start_discovering_gateway(run, 'binding_start')
    expect_binding_failure_survived(gateway, binding, 60)


def scenario_binding_directory_on_reconnect(run):
    # The PLC comes up after the start-up sweep, so a rescan in the reconnect arm adopts it.
    gateway, binding = start_discovering_gateway(run, 'binding_reconnect')
    if not wait_log(gateway.log, 'OPC-UA discovery summary: 0 data server(s)', 30):
        raise AssertionError('the start-up sweep did not come back empty:\n'
                             + tail(gateway))
    run.start_server('binding_reconnect', port=DISCOVERY_PORT)
    expect_binding_failure_survived(gateway, binding, 60)


SCENARIOS = [
    ('pinned stand-in', scenario_pinned_stand_in),
    ('foreign stand-in', scenario_foreign_stand_in),
    ('late nameplate', scenario_late_nameplate),
    ('per-source record', scenario_per_source_record),
    ('per-source record under a stand-in', scenario_per_source_placeholder_rename),
    ('per-source pinned stand-in', scenario_per_source_pinned_stand_in),
    ('explicit component id', scenario_explicit_component_id_keeps_the_default_id_record),
    ('explicit component id, stand-in record',
     scenario_explicit_component_id_keeps_a_stand_in_record),
    ('explicit component id, shared stand-in record',
     scenario_explicit_component_id_keeps_a_shared_stand_in_record),
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
        started = time.monotonic()
        error = None
        try:
            scenario(run)
        except (AssertionError, OSError, subprocess.SubprocessError) as e:
            error = e
        finally:
            run.stop_all()
        elapsed = time.monotonic() - started
        if error is None:
            print(f'  OK {name} ({elapsed:.0f} s)')
        else:
            failures += 1
            print(f'FAIL {name} ({elapsed:.0f} s): {error}', file=sys.stderr)
            for log in run.server_logs:
                print(f'{log.name}:\n' + alarm_fixture.read_log(log), file=sys.stderr)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
