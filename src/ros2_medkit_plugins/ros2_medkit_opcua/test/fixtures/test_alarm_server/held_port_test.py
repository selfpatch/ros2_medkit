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
test_alarm_server on a port another process listens on.

The fixture must print no READY, print its EXIT line and exit non-zero while
its stdin stays open. start_fixture, handed that port first, must end up on a
new port where the fixture answers an OPC UA Hello, and must give up after
ATTEMPTS ports that are all taken.

Usage: held_port_test.py <test_alarm_server>
"""

import contextlib
import io
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile

import alarm_fixture

failures = []


def check(condition, message):
    """Record a failed expectation and keep going, so one run reports them all."""
    if condition:
        print(f'  OK   {message}')
    else:
        print(f'  FAIL {message}', file=sys.stderr)
        failures.append(message)
    return bool(condition)


def opcua_hello(port):
    """Send an OPC UA Hello to 127.0.0.1:<port>; return the reply's message type, or b''."""
    url = f'opc.tcp://127.0.0.1:{port}'.encode()
    # ProtocolVersion, ReceiveBufferSize, SendBufferSize, MaxMessageSize, MaxChunkCount.
    body = struct.pack('<5I', 0, 65536, 65536, 0, 0) + struct.pack('<i', len(url)) + url
    message = b'HELF' + struct.pack('<I', 8 + len(body)) + body
    reply = b''
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=5) as s:
            s.sendall(message)
            while len(reply) < 4:
                chunk = s.recv(4 - len(reply))
                if not chunk:
                    break
                reply += chunk
    except OSError:
        pass
    return reply


def run_on_held_port(server_bin, held, workdir):
    """Start the fixture on the held port with its stdin kept open."""
    print(f'--- the fixture on held port {held} ---')
    log_path = workdir / 'held.log'
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(
            [str(server_bin), '--port', str(held)],
            stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        rc = proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        rc = None
    alarm_fixture.stop(proc)
    text = alarm_fixture.read_log(log_path)
    print(text)
    check('READY ' not in text, 'no READY line')
    check('EXIT ' in text, 'an EXIT line')
    check(rc is not None and rc != 0, f'exits non-zero with stdin open (rc={rc})')


def run_start_fixture(server_bin, held, workdir):
    """Hand start_fixture the held port first; it must move to a new one."""
    print(f'--- start_fixture, first reserved port {held} ---')
    ports = [held]

    def reserve():
        return ports.pop(0) if ports else alarm_fixture.reserve_port()

    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        proc, port = alarm_fixture.start_fixture(server_bin, workdir / 'moved.log',
                                                 reserve=reserve, deadline=20)
    print(err.getvalue())
    try:
        check(proc is not None and port != held,
              f'the fixture runs on a port other than {held} (got {port})')
        check(f'on port {held} printed no READY' in err.getvalue() and 'EXIT ' in err.getvalue(),
              'the failed attempt and its EXIT line are reported')
        check(opcua_hello(port) == b'ACKF', f'the fixture on port {port} answers an OPC UA Hello')
    finally:
        if proc is not None:
            alarm_fixture.stop(proc)


def run_give_up(server_bin, held, workdir):
    """Every reserved port is taken: start_fixture stops after ATTEMPTS."""
    print('--- start_fixture, every reserved port held ---')
    calls = []

    def reserve():
        calls.append(held)
        return held

    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        proc, _port = alarm_fixture.start_fixture(server_bin, workdir / 'given_up.log',
                                                  reserve=reserve, deadline=20)
    print(err.getvalue())
    if proc is not None:
        alarm_fixture.stop(proc)
    check(proc is None, 'start_fixture gives up')
    check(len(calls) == alarm_fixture.ATTEMPTS,
          f'after {alarm_fixture.ATTEMPTS} attempts (got {len(calls)})')
    check(err.getvalue().count('EXIT ') == alarm_fixture.ATTEMPTS,
          "and reports each attempt's EXIT line")


def main():
    if len(sys.argv) < 2:
        print('usage: held_port_test.py <test_alarm_server>', file=sys.stderr)
        return 2
    server_bin = Path(sys.argv[1]).resolve()
    workdir = Path(tempfile.mkdtemp(prefix='alarm_server_held_port_'))
    holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        holder.bind(('127.0.0.1', 0))
        holder.listen()
        held = holder.getsockname()[1]
        run_on_held_port(server_bin, held, workdir)
        run_start_fixture(server_bin, held, workdir)
        run_give_up(server_bin, held, workdir)
    finally:
        holder.close()
        shutil.rmtree(workdir, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)} expectation(s) not met:', file=sys.stderr)
        for f in failures:
            print(f'  - {f}', file=sys.stderr)
        return 1
    print('PASS: a taken port fails the fixture start and start_fixture moves on')
    return 0


if __name__ == '__main__':
    sys.exit(main())
