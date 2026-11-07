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
against it via ``asyncua``, then terminates the binary cleanly. The
fixture's output is printed when the smoke test fails.

Skips (exits 77 - the CTest convention for a skipped test) when
``asyncua`` is not importable AND the env var
``ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE`` is unset / 0. Contributors iterating
on the plugin sources can keep the skip; CI sets the env var so a
missing ``asyncua`` becomes a hard failure (exit 1) instead of a silent
skip. Closes the regression gap Copilot flagged on PR #387 where a CI
step that drops the ``asyncua`` install would silently mask smoke
regressions.
"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import alarm_fixture

CTEST_SKIP = 77


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
        # Default: skip with CTest convention (77) so local dev iteration
        # without pip install just passes the build.
        # CI sets ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE=1 to turn the skip into
        # a hard failure - so a CI job that loses its asyncua install
        # cannot silently bypass the smoke check.
        require = os.environ.get('ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE', '0')
        if require not in ('', '0'):
            print('asyncua not installed and ROS2_MEDKIT_OPCUA_SMOKE_REQUIRE set - failing',
                  file=sys.stderr)
            return 1
        print('asyncua not installed - skipping smoke test (CTest skip code)')
        return CTEST_SKIP

    workdir = Path(tempfile.mkdtemp(prefix='alarm_server_smoke_'))
    log_path = workdir / 'alarm_server.log'
    proc, port = alarm_fixture.start_fixture(binary, log_path, deadline=15)
    try:
        if proc is None:
            print('test_alarm_server printed no READY on '
                  f'{alarm_fixture.ATTEMPTS} reserved ports', file=sys.stderr)
            return 1
        try:
            returncode = subprocess.run(
                [sys.executable, str(smoke_test), f'opc.tcp://127.0.0.1:{port}'],
                check=False,
                timeout=30,
            ).returncode
        except subprocess.TimeoutExpired:
            print('smoke test timed out after 30 s', file=sys.stderr)
            returncode = 1
        if returncode != 0:
            print('test_alarm_server output:\n' + alarm_fixture.read_log(log_path),
                  file=sys.stderr)
        return returncode
    finally:
        if proc is not None:
            alarm_fixture.stop(proc)
        shutil.rmtree(workdir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
