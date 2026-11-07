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
Start test_alarm_server and wait until it listens.

The fixture prints READY once its listen socket is bound, and exits non-zero
when the bind fails. A port from reserve_port() is released before the fixture
binds it, so another process can take it first.
"""

import os
import signal
import socket
import subprocess
import sys
import time

# Starts on newly reserved ports before start_fixture gives up.
ATTEMPTS = 3


def reserve_port():
    """Return a free loopback TCP port. It is released, not held."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def read_log(path):
    """Return the text of a log file, or '' when it cannot be read."""
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def stop(proc):
    """Close the fixture's stdin, SIGTERM its process group, SIGKILL after 10 s."""
    # The fixture's stdin reader ends only on EOF.
    if proc.stdin is not None:
        try:
            proc.stdin.close()
        except OSError:
            pass
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        except ProcessLookupError:
            proc.wait()
    log = getattr(proc, '_log', None)
    if log is not None:
        log.close()


def _launch(server_bin, port, log_path, args, deadline):
    """Start one fixture. Return it once READY is logged, or None once it is stopped."""
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        [str(server_bin), '--port', str(port), *args],
        stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
        text=True, start_new_session=True,
    )
    proc._log = log
    end = time.monotonic() + deadline
    while time.monotonic() < end:
        # Read the exit first: output written before the exit is then in the log.
        exited = proc.poll() is not None
        if 'READY ' in read_log(log_path):
            return proc
        if exited:
            break
        time.sleep(0.05)
    stop(proc)
    print(f'test_alarm_server on port {port} printed no READY (rc={proc.returncode}):\n'
          + read_log(log_path), file=sys.stderr)
    return None


def start_fixture(server_bin, log_path, *args, port=None, deadline=25,
                  reserve=reserve_port, attempts=ATTEMPTS):
    """
    Start the fixture and return (proc, port) once it listens, or (None, port).

    Without port, each attempt runs on a port from reserve(). With port, one
    attempt runs on that port, so a restart that must keep it fails when the
    port was taken. Every failed attempt's output goes to stderr. The log file
    holds the output of the last attempt; proc._log is its open handle.
    """
    if port is not None:
        return _launch(server_bin, port, log_path, args, deadline), port
    for _ in range(attempts):
        port = reserve()
        proc = _launch(server_bin, port, log_path, args, deadline)
        if proc is not None:
            return proc, port
    return None, port
