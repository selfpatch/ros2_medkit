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
CTest entry point for the test_alarm_server smoke check (issue #386).

Spawns the freshly-built test_alarm_server binary on an ephemeral port,
waits for the ``READY`` line, runs the AlarmConditionType smoke test
against it via ``asyncua``, then terminates the binary cleanly.

Skips (exits 77 - the CTest convention for a skipped test) when
``asyncua`` is not importable, so contributors who only iterate on the
plugin sources do not need a Python pip install. CI installs ``asyncua``
in the integration job and observes the test as a real pass / fail.
"""

import os
from pathlib import Path
import socket
import subprocess
import sys
import time

CTEST_SKIP = 77


def find_free_port():
    """Bind ``:0`` on localhost, return the OS-assigned port and release it."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for_ready(proc, deadline_s=15):
    """
    Block until the binary prints 'READY ' to stdout, or the deadline expires.

    The server emits 'READY port=NNNN namespace=N' once the OPC UA listen
    socket is bound. We avoid sleeping; we read line-by-line.
    """
    start = time.monotonic()
    while time.monotonic() - start < deadline_s:
        if proc.poll() is not None:
            raise RuntimeError(
                f'test_alarm_server exited early (rc={proc.returncode})'
            )
        line = proc.stdout.readline()
        if not line:
            time.sleep(0.05)
            continue
        if line.startswith('READY '):
            return
    raise TimeoutError(
        f'test_alarm_server did not print READY within {deadline_s}s'
    )


def main():
    """Run smoke test against a freshly-spawned test_alarm_server."""
    if len(sys.argv) < 2:
        print(
            'usage: run_ctest.py <path/to/test_alarm_server> [<smoke_test.py>]',
            file=sys.stderr,
        )
        return 2

    binary = Path(sys.argv[1]).resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        print(f'test_alarm_server binary missing or not executable: {binary}',
              file=sys.stderr)
        return 1

    smoke_test = (
        Path(sys.argv[2]).resolve()
        if len(sys.argv) >= 3
        else Path(__file__).resolve().parent / 'smoke_test.py'
    )
    if not smoke_test.is_file():
        print(f'smoke_test.py not found at {smoke_test}', file=sys.stderr)
        return 1

    try:
        import asyncua  # noqa: F401  pylint: disable=unused-import,import-outside-toplevel
    except ImportError:
        print('asyncua not installed - skipping smoke test (CTest skip code)')
        return CTEST_SKIP

    port = find_free_port()
    proc = subprocess.Popen(
        [str(binary), '--port', str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        wait_for_ready(proc, deadline_s=15)
        result = subprocess.run(
            [sys.executable, str(smoke_test), f'opc.tcp://127.0.0.1:{port}'],
            check=False,
            timeout=30,
        )
        return result.returncode
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


if __name__ == '__main__':
    sys.exit(main())
