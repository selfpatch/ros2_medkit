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
A SIGINT or SIGTERM ends the OPC-UA startup sweep and the gateway.

The sweep runs inside the gateway node's constructor. A silent TCP listener on
port 4840 accepts every connect on 127.0.0.0/24 and never answers. Discovery
identifies only port 4840, so each identify waits out its timeout and the sweep
would take minutes. The test sends
the signal once the sweep has started and expects the gateway to exit well
before that.

Usage: test_opcua_startup_sweep_signal.test.py
"""

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

# The only port discovery identifies. Other ports are leads and end the sweep at once.
OPCUA_PORT = 4840
IDENTIFY_TIMEOUT_MS = 3000
# 254 hosts, one at a time: the sweep alone takes over 12 minutes.
SCAN_CONCURRENCY = 1
CONNECTS_BEFORE_SIGNAL = 3
SWEEP_START_TIMEOUT_SEC = 60.0
# Two identify timeouts for the sweep to notice, plus shutdown.
EXIT_TIMEOUT_SEC = 2 * IDENTIFY_TIMEOUT_MS / 1000.0 + 15.0


def free_port():
    """Return a free TCP port on the loopback."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def find_installed(name):
    """Return the first file called <name> under AMENT_PREFIX_PATH, or None."""
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        for root, _dirs, files in os.walk(prefix):
            if name in files:
                return os.path.join(root, name)
    return None


class SilentListener:
    """Accept TCP connections on all addresses and never send a byte."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.listen(512)
        self.port = self.sock.getsockname()[1]
        self.accepted = []
        self._lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, _addr = self.sock.accept()
            except OSError:
                return
            with self._lock:
                self.accepted.append(conn)

    def count(self):
        with self._lock:
            return len(self.accepted)

    def drop_accepted(self):
        with self._lock:
            for conn in self.accepted:
                conn.close()
            self.accepted = []


def run_case(sig, listener, gateway_bin, plugin, workdir):
    """Start a gateway whose startup sweep hangs, send <sig>, return an error or None."""
    listener.drop_accepted()
    params = workdir / f'gateway_{sig.name}.yaml'
    params.write_text('\n'.join([
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    server:',
        '      host: "127.0.0.1"',
        f'      port: {free_port()}',
        '    plugins: ["opcua"]',
        f'    plugins.opcua.path: "{plugin}"',
        '    plugins.opcua.discovery.enabled: true',
        '    plugins.opcua.discovery.subnets: ["127.0.0.0/24"]',
        f'    plugins.opcua.discovery.ports: [{listener.port}]',
        f'    plugins.opcua.discovery.scan_concurrency: {SCAN_CONCURRENCY}',
        f'    plugins.opcua.discovery.identify_timeout_ms: {IDENTIFY_TIMEOUT_MS}',
        '    plugins.opcua.discovery.interval_s: 0',
        '    plugins.opcua.discovery.binding_file: ""',
    ]) + '\n')
    log_path = workdir / f'gateway_{sig.name}.log'
    with open(log_path, 'wb') as log:
        proc = subprocess.Popen(
            [gateway_bin, '--ros-args', '--params-file', str(params)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + SWEEP_START_TIMEOUT_SEC
        while (time.monotonic() < deadline and listener.count() < CONNECTS_BEFORE_SIGNAL
               and proc.poll() is None):
            time.sleep(0.1)
        if listener.count() < CONNECTS_BEFORE_SIGNAL:
            return (f'{sig.name}: the startup sweep never reached the listener '
                    f'({listener.count()} connects, gateway exit {proc.poll()})')
        os.kill(proc.pid, sig)
        try:
            code = proc.wait(timeout=EXIT_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            return (f'{sig.name}: the gateway still runs {EXIT_TIMEOUT_SEC:.0f} s after the '
                    f'signal ({listener.count()} connects so far)')
        if code not in (0, -sig.value):
            return f'{sig.name}: the gateway exited with {code}'
        return None
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        if proc.returncode not in (0, -signal.SIGINT.value, -signal.SIGTERM.value):
            print(log_path.read_text(errors='replace')[-3000:], file=sys.stderr)


def main():
    if not os.environ.get('ROS_DOMAIN_ID'):
        print('ROS_DOMAIN_ID is not set: run this test through CTest', file=sys.stderr)
        return 1
    plugin = find_installed('libros2_medkit_opcua_plugin.so')
    gateway_bin = find_installed('gateway_node')
    if plugin is None or gateway_bin is None:
        print(f'missing: plugin={plugin} gateway_node={gateway_bin}', file=sys.stderr)
        return 1
    workdir = Path(tempfile.mkdtemp(prefix='opcua_sweep_signal_'))
    # One listener for both cases: a socket blocked in accept() keeps the port bound.
    listener = SilentListener(OPCUA_PORT)
    failures = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        started = time.monotonic()
        error = run_case(sig, listener, gateway_bin, plugin, workdir)
        elapsed = time.monotonic() - started
        if error is None:
            print(f'  OK   {sig.name} ({elapsed:.1f} s)')
        else:
            print(f'  FAIL {error}', file=sys.stderr)
            failures.append(error)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
