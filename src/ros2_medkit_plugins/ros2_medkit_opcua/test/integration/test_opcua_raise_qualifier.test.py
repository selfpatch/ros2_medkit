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
raise_after_samples and raise_after_ms through a real gateway, fault manager and OPC UA server.

The fixture server holds two writable Int32 registers and a constant
Tank.Level of 42.0. The node map gives each a detection rule:

  * Tank.Level > 40, no qualifier: active from the first read. The control
    that shows the detection path is live in each leg.
  * StatusWord bit 0 with raise_after_ms: 3000.
  * FaultCode > 0 with raise_after_samples: 3.

The test writes the registers through the fixture's stdin and reads which
faults are CONFIRMED from the fault manager over the gateway's /faults, so a
fault only counts once its report has left the plugin. It runs once with
polling and once with subscriptions, the transport where a steady value sends
no further updates. Each leg has its own fault manager, and the plugin clears a
fault through it, so a fault that was raised and cleared comes back with
occurrence_count 2. Per leg:

  * a StatusWord blip shorter than the window, then a held StatusWord: the
    fault shows up no sooner than 3 s after the write, with occurrence_count
    1, so the blip never raised it,
  * the clear is immediate,
  * ten FaultCode blips shorter than one poll, each with two changes (5, 6,
    then 0), then a held FaultCode: the fault shows up after three samples' time,
    not on the first, with occurrence_count 1, so no blip raised it.

The poll leg then freezes the server with SIGSTOP part way through a 3 s wait,
and checks that after SIGCONT and the reconnect the wait starts over.

The subscription leg uses a subscription interval well below the poll
interval, so data changes arrive between polls, as they do in the field. A
third leg runs subscriptions with a 60 s poll and checks that a held StatusWord
still shows up just after 3 s, not at the next poll.

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
# Under a subscription this is a wait of (SAMPLES - 1) polls, 1 s. A blip
# reaches the plugin up to about one publish and one pump step late, so the
# wait has to be well above the longest blip (about 0.4 s with the writes).
SAMPLES = 3
POLL_MS = 500
# Subscription leg: changes arrive between polls, every SUB_MS.
SUB_MS = 50
BLIPS = 10
BLIP_S = 0.25
BLIP_GAP_S = 0.8  # longer than one poll
# Duration leg: a poll so slow that only the recheck between polls can end
# raise_after_ms on time.
SLOW_POLL_MS = 60000

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


def write_params(path, *, port, plugin, server_port, node_map, manifest, subscriptions,
                 poll_ms=POLL_MS):
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
        f'    plugins.opcua.poll_interval_ms: {poll_ms}',
        f'    plugins.opcua.prefer_subscriptions: {"true" if subscriptions else "false"}',
        f'    plugins.opcua.subscription_interval_ms: {SUB_MS if subscriptions else POLL_MS}.0',
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


def active(faults_url):
    """Return the fault codes the fault manager has CONFIRMED, or None if unreadable."""
    listing = http_json(faults_url)
    if not isinstance(listing, dict):
        return None
    return {item.get('fault_code') for item in listing.get('items', [])
            if item.get('status') == 'CONFIRMED'}


def occurrences(faults_url, code):
    """Return the occurrence_count of <code>: 0 when there is no such fault, None if unreadable."""
    listing = http_json(faults_url)
    if not isinstance(listing, dict):
        return None
    for item in listing.get('items', []):
        if item.get('fault_code') == code:
            return item.get('occurrence_count')
    return 0


def plugin_sees(data_url, want, deadline, period=0.05):
    """Return True once x-plc-data shows <want>, the value the plugin last got for the point."""
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        point = http_json(data_url, timeout=2)
        if isinstance(point, dict) and point.get('value') == want:
            return True
        time.sleep(period)
    return False


def plc_connected(status_url, want, deadline, period=0.1):
    """Seconds until x-plc-status reports connected == <want>, or None if not within <deadline>."""
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        status = http_json(status_url, timeout=2)
        if isinstance(status, dict) and status.get('connected') is want:
            return time.monotonic() - start
        time.sleep(period)
    return None


def first_active(faults_url, code, deadline, period=0.1):
    """Seconds until <code> is active, or None if it is not within <deadline>."""
    start = time.monotonic()
    while time.monotonic() - start < deadline:
        now = active(faults_url)
        if now is not None and code in now:
            return time.monotonic() - start
        time.sleep(period)
    return None


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
    fm_log = workdir / f'fault_manager_{mode}.log'
    fm = start_fault_manager(fm_log, env)
    if not check(fm is not None, f'{mode}: fault_manager_node started'):
        print(fm_log.read_text(errors='replace')[-3000:], file=sys.stderr)
        return
    gw = start_gateway(params, log, env)
    status_url = f'http://127.0.0.1:{port}/api/v1/components/{COMPONENT}/x-plc-status'
    faults_url = f'http://127.0.0.1:{port}/api/v1/faults?status=all'
    status_word_url = f'http://127.0.0.1:{port}/api/v1/apps/{ENTITY}/x-plc-data/status_word'
    try:
        # Connected, on the transport this leg is about, and the control rule
        # without a qualifier already CONFIRMED in the fault manager.
        ready = False
        status = None
        for _ in range(int(90 * SCALE / 0.5)):
            status = http_json(status_url)
            now = active(faults_url)
            if isinstance(status, dict) and status.get('connected') is True \
                    and now is not None and CONTROL in now:
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
        # Step 2 sets 1 again, so the 0 has to reach the plugin first, or the
        # plugin sees one long 1 and step 2 measures from the wrong start.
        check(plugin_sees(status_word_url, 0, 5 * SCALE),
              f'{mode}: the plugin got the 0 that ends the blip')
        check(blip < HOLD_MS / 1000.0,
              f'{mode}: the blip lasted {blip:.2f} s, less than {HOLD_MS} ms')

        # 2. A held condition. The wait starts at the first sample, which is no
        #    earlier than the write, so the fault cannot show up sooner. A blip
        #    that raised the fault would have been cleared again, so this raise
        #    would be its second occurrence.
        check(set_register(server, server_log, 'StatusWord', 1), f'{mode}: StatusWord set to 1')
        elapsed = first_active(faults_url, HELD, (HOLD_MS / 1000.0 + 5.0) * SCALE)
        check(elapsed is not None, f'{mode}: a held StatusWord raises {HELD}')
        if elapsed is not None:
            check(elapsed >= HOLD_MS / 1000.0 - 0.1,
                  f'{mode}: {HELD} shows up no sooner than {HOLD_MS} ms (after {elapsed:.2f} s)')
            count = occurrences(faults_url, HELD)
            check(count == 1,
                  f'{mode}: {HELD} is on its first occurrence, so the 1 s blip never raised it '
                  f'(occurrence_count {count})')

        # 3. The clear does not wait.
        set_register(server, server_log, 'StatusWord', 0)
        cleared = None
        start = time.monotonic()
        while time.monotonic() - start < 3.0 * SCALE:
            now = active(faults_url)
            if now is not None and HELD not in now:
                cleared = time.monotonic() - start
                break
            time.sleep(0.1)
        check(cleared is not None, f'{mode}: {HELD} clears once StatusWord is 0')

        # 4. Short blips. Each one is shorter than a poll, so polling never
        #    reads it twice, and under a subscription it is shorter than the
        #    wait the samples rule becomes. The gap between two blips is longer
        #    than a poll, so a healthy read always lands in between: with a
        #    shorter gap, two reads in a row can each catch a different blip,
        #    and polling rightly raises.
        longest = 0.0
        for _ in range(BLIPS):
            blip_start = time.monotonic()
            set_register(server, server_log, 'FaultCode', 5)
            time.sleep(BLIP_S / 2)
            set_register(server, server_log, 'FaultCode', 6)
            time.sleep(BLIP_S / 2)
            set_register(server, server_log, 'FaultCode', 0)
            longest = max(longest, time.monotonic() - blip_start)
            time.sleep(BLIP_GAP_S)
        # Polling needs a blip shorter than a poll. A subscription needs it shorter
        # than the wait, less one publish and one pump step it can arrive late.
        limit = POLL_MS / 1000.0
        if subscriptions:
            limit = ((SAMPLES - 1) * POLL_MS - 2 * SUB_MS) / 1000.0
        check(longest < limit,
              f'{mode}: the longest FaultCode blip lasted {longest:.2f} s, under {limit:.1f} s')

        # 5. Samples. Under a subscription the register sends one change, and
        #    the rule waits (SAMPLES - 1) polls from it. A blip that got through
        #    was raised and cleared at once, so this raise would not be the first.
        check(set_register(server, server_log, 'FaultCode', 5), f'{mode}: FaultCode set to 5')
        elapsed = first_active(faults_url, REPEATED, (SAMPLES * POLL_MS / 1000.0 + 6.0) * SCALE)
        check(elapsed is not None, f'{mode}: a held FaultCode raises {REPEATED}')
        if elapsed is not None:
            least = (SAMPLES - 1) * POLL_MS / 1000.0 - 0.3
            check(elapsed >= least,
                  f'{mode}: {REPEATED} needs {SAMPLES} samples, not one (after {elapsed:.2f} s)')
            count = occurrences(faults_url, REPEATED)
            check(count == 1,
                  f'{mode}: {REPEATED} is on its first occurrence, so no short blip raised it '
                  f'(occurrence_count {count})')
        set_register(server, server_log, 'FaultCode', 0)
        if not subscriptions:
            # 6. A frozen server. Only polling sees the link go down; under a
            #    subscription a frozen server is not noticed yet.
            frozen_wait(server, server_log, status_url, faults_url, status_word_url)
        check(first_active(faults_url, CONTROL, 5 * SCALE) is not None,
              f'{mode}: {CONTROL} still active at the end of the leg')
    finally:
        terminate(gw)
        terminate(fm)
        # Leave the registers at 0 for the next leg; a server that is already
        # gone is reported by the next leg's own checks.
        try:
            set_register(server, server_log, 'StatusWord', 0)
            set_register(server, server_log, 'FaultCode', 0)
        except (BrokenPipeError, ValueError, OSError):
            pass


def frozen_wait(server, server_log, status_url, faults_url, status_word_url):
    """Freeze the server part way through a wait: after the reconnect the wait starts over."""
    if not check(set_register(server, server_log, 'StatusWord', 1),
                 'poll: StatusWord set to 1 before the freeze'):
        return
    # The wait has to be running before the freeze, or the test proves nothing.
    if not check(plugin_sees(status_word_url, 1, 5 * SCALE),
                 'poll: the plugin got the 1 before the freeze'):
        return
    time.sleep(HOLD_MS / 2000.0)
    os.kill(server.pid, signal.SIGSTOP)
    try:
        down = plc_connected(status_url, False, 15 * SCALE)
        check(down is not None, 'poll: the link is seen down while the server is frozen')
    finally:
        os.kill(server.pid, signal.SIGCONT)
    if down is None:
        return
    up = plc_connected(status_url, True, 30 * SCALE)
    if not check(up is not None, 'poll: the plugin reconnects once the server runs again'):
        return
    # The wait started before the freeze. If it carried over, the fault would
    # show up at the first read after the reconnect.
    elapsed = first_active(faults_url, HELD, (HOLD_MS / 1000.0 + 5.0) * SCALE)
    check(elapsed is not None, f'poll: a StatusWord held over the freeze raises {HELD}')
    if elapsed is not None:
        check(elapsed >= HOLD_MS / 1000.0 - 0.5,
              f'poll: the wait starts over after the reconnect, {HELD} shows up no sooner '
              f'than {HOLD_MS} ms after it (after {elapsed:.2f} s)')
    set_register(server, server_log, 'StatusWord', 0)


def run_duration_leg(workdir, env, plugin, server, server_log, server_port, manifest):
    """Subscriptions with a 60 s poll: raise_after_ms must still end on time."""
    print('--- duration leg (subscriptions, 60 s poll) ---')
    node_map = workdir / 'nodes.yaml'
    node_map.write_text(node_map_text())
    port = free_port()
    params = workdir / 'gateway_duration.yaml'
    write_params(params, port=port, plugin=plugin, server_port=server_port, node_map=node_map,
                 manifest=manifest, subscriptions=True, poll_ms=SLOW_POLL_MS)
    log = workdir / 'gateway_duration.log'
    fm_log = workdir / 'fault_manager_duration.log'
    fm = start_fault_manager(fm_log, env)
    if not check(fm is not None, 'duration: fault_manager_node started'):
        print(fm_log.read_text(errors='replace')[-3000:], file=sys.stderr)
        return
    gw = start_gateway(params, log, env)
    status_url = f'http://127.0.0.1:{port}/api/v1/components/{COMPONENT}/x-plc-status'
    faults_url = f'http://127.0.0.1:{port}/api/v1/faults?status=all'
    try:
        # Waiting for the control rule also waits until reports reach the fault
        # manager, so the timing below starts with the sink ready.
        ready = first_active(faults_url, CONTROL, 90 * SCALE, period=0.5) is not None
        if not check(ready, f'duration: {CONTROL} CONFIRMED with a 60 s poll'):
            print(log.read_text(errors='replace')[-3000:], file=sys.stderr)
            return
        status = http_json(status_url) or {}
        if not check(status.get('mode') == 'subscription',
                     'duration: the plugin runs in subscription mode '
                     f'(got {status.get("mode")!r})'):
            return
        check(set_register(server, server_log, 'StatusWord', 1), 'duration: StatusWord set to 1')
        elapsed = first_active(faults_url, HELD, (HOLD_MS / 1000.0 + 5.0) * SCALE)
        check(elapsed is not None, f'duration: a held StatusWord raises {HELD}')
        if elapsed is not None:
            check(HOLD_MS / 1000.0 - 0.1 <= elapsed <= HOLD_MS / 1000.0 + 1.5 * SCALE,
                  f'duration: {HELD} shows up just after {HOLD_MS} ms, not at the next '
                  f'60 s poll (after {elapsed:.2f} s)')
    finally:
        terminate(gw)
        terminate(fm)
        try:
            set_register(server, server_log, 'StatusWord', 0)
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
        run_duration_leg(workdir, env, plugin, server, server_log, server_port, manifest)

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
